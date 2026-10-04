#include "physics/core/fluids/periodic_mac_grid.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace PhysicsEngine {
namespace {
double Checked(double value) {
    if (!std::isfinite(value)) throw std::overflow_error("MAC diffusion arithmetic exceeds float64 range.");
    return value;
}
// Stage the exponent separately: e.g. rho*V*velocity^2 can be finite even
// when velocity^2 by itself overflows or underflows.
double Product(std::initializer_list<double> factors) {
    double mantissa=1; int exponent=0;
    for(double factor:factors) {
        if(factor==0) return 0;
        int e=0; mantissa*=std::frexp(factor,&e); exponent+=e;
    }
    const double result=Checked(std::ldexp(mantissa,exponent));
    if(result==0) throw std::overflow_error("MAC diffusion diagnostic or coefficient underflows float64 range.");
    return result;
}
double NormalizedDifference(double a,double b,double scale) {
    const double difference=a-b;
    const double result=std::isfinite(difference)?difference/scale:a/scale-b/scale;
    if(difference!=0&&result==0)
        throw std::overflow_error("MAC stored difference underflows normalized float64 range.");
    return Checked(result);
}
struct Work {
    std::size_t count=0,limit,n;
    void charge() {
        if(n>limit-count) throw std::runtime_error("MAC diffusion cell-visit budget exhausted.");
        count+=n;
    }
};
double Dot(const std::vector<double>& a,const std::vector<double>& b,Work& work) {
    work.charge(); double sum=0;
    for(std::size_t k=0;k<a.size();++k) sum=Checked(sum+Checked(a[k]*b[k]));
    return sum;
}
double Rms(const std::vector<double>& a,Work& work) {
    work.charge(); double norm=0; bool nonzero=false;
    const double root=std::sqrt(double(a.size()));
    for(double value:a) { nonzero=nonzero||value!=0; norm=std::hypot(norm,value/root); }
    if(nonzero&&norm==0) throw std::overflow_error("MAC diffusion RMS underflows float64 range.");
    return Checked(norm);
}
double Mean(const std::vector<double>& a,Work& work) {
    work.charge(); double sum=0;
    for(double value:a) sum=Checked(sum+value/a.size());
    return sum;
}
double PhysicalDot(const std::vector<double>& a,const std::vector<double>& b,
                   double factor,double density,double volume,double scale,Work& work) {
    work.charge(); double sum=0;
    for(std::size_t k=0;k<a.size();++k)
        sum=Checked(sum+Product({factor,density,volume,scale,scale,a[k],b[k]}));
    return sum;
}
struct Operator {
    const PeriodicMacGridConfig& c;
    double ax,ay,diagonal;
    std::vector<double> apply(const std::vector<double>& x,Work& work,bool scaled,
                              const std::vector<double>* rhs=nullptr) const {
        work.charge(); std::vector<double> out(x.size());
        const double wx=scaled?ax/diagonal:ax,wy=scaled?ay/diagonal:ay;
        for(std::size_t j=0;j<c.rows;++j) for(std::size_t i=0;i<c.columns;++i) {
            const auto k=i+c.columns*j;
            const double dx=Checked(Checked(x[k]-x[(i+1)%c.columns+c.columns*j])
                +Checked(x[k]-x[(i+c.columns-1)%c.columns+c.columns*j]));
            const double dy=Checked(Checked(x[k]-x[i+c.columns*((j+1)%c.rows)])
                +Checked(x[k]-x[i+c.columns*((j+c.rows-1)%c.rows)]));
            const double identity=rhs?Checked(x[k]-(*rhs)[k]):x[k];
            out[k]=Checked((scaled?identity/diagonal:identity)+Checked(wx*dx)+Checked(wy*dy));
        }
        return out;
    }
    std::vector<double> residual(const std::vector<double>& x,const std::vector<double>& rhs,Work& work) const {
        // Form x-rhs before adding diffusion, avoiding cancellation of two
        // nearly equal large identity terms when diffusion is very small.
        return apply(x,work,false,&rhs);
    }
    std::vector<double> storedResidual(const std::vector<double>& next,const std::vector<double>& old,
                                       double scale,Work& work) const {
        work.charge(); std::vector<double> out(next.size());
        for(std::size_t j=0;j<c.rows;++j) for(std::size_t i=0;i<c.columns;++i) {
            const auto k=i+c.columns*j;
            const double dx=Checked(NormalizedDifference(next[k],next[(i+1)%c.columns+c.columns*j],scale)
                +NormalizedDifference(next[k],next[(i+c.columns-1)%c.columns+c.columns*j],scale));
            const double dy=Checked(NormalizedDifference(next[k],next[i+c.columns*((j+1)%c.rows)],scale)
                +NormalizedDifference(next[k],next[i+c.columns*((j+c.rows-1)%c.rows)],scale));
            out[k]=Checked(NormalizedDifference(next[k],old[k],scale)+Product({ax,dx})+Product({ay,dy}));
        }
        return out;
    }
};
struct Component {
    double scale=0,initialRms=0,finalRms=0,initialMean=0,finalMean=0,meanAllowance=0;
    double initialEnergy=0,finalEnergy=0,incrementEnergy=0,dissipation=0,residualWork=0;
    std::vector<double> candidate;
    std::size_t iterations=0;
};
Component Solve(const std::vector<double>& old,const Operator& op,const MacDiffusionConfig& options,
                double target,Work& work,std::size_t& total,bool noOp) {
    Component out; work.charge();
    for(double value:old) out.scale=std::max(out.scale,std::abs(value));
    out.candidate=old;
    if(out.scale==0) return out;
    std::vector<double> rhs(old.size()); work.charge();
    for(std::size_t k=0;k<old.size();++k) rhs[k]=old[k]/out.scale;
    out.initialMean=Product({out.scale,Mean(rhs,work)});
    out.initialRms=Product({out.scale,Rms(rhs,work)});
    std::vector<double> x=rhs,actual(old.size());
    if(!noOp) {
        std::vector<double> residual=op.residual(x,rhs,work);
        work.charge(); for(double& value:residual) value/=op.diagonal;
        std::vector<double> direction=residual;
        work.charge(); for(double& value:direction) value=-value;
        double rr=Dot(residual,residual,work);
        while(true) {
            actual=op.residual(x,rhs,work);
            out.finalRms=Product({out.scale,Rms(actual,work)});
            if(out.finalRms<=target) break;
            if(total==options.maximumIterations || rr==0)
                throw std::runtime_error("MAC diffusion did not achieve actual velocity residual tolerance.");
            const auto ad=op.apply(direction,work,true);
            const double denominator=Dot(direction,ad,work);
            if(denominator<=0) throw std::runtime_error("MAC diffusion lost positive definiteness.");
            const double alpha=Checked(rr/denominator);
            work.charge(); for(std::size_t k=0;k<x.size();++k) {
                x[k]=Checked(x[k]+Checked(alpha*direction[k]));
                residual[k]=Checked(residual[k]+Checked(alpha*ad[k]));
            }
            const double next=Dot(residual,residual,work),beta=Checked(next/rr);
            work.charge(); for(std::size_t k=0;k<x.size();++k)
                direction[k]=Checked(-residual[k]+Checked(beta*direction[k]));
            rr=next; ++total; ++out.iterations;
        }
    }
    if(!noOp) {
        work.charge(); for(std::size_t k=0;k<x.size();++k) out.candidate[k]=Checked(out.scale*x[k]);
    }
    // Final acceptance uses the values after multiplication back into storage.
    work.charge(); for(std::size_t k=0;k<x.size();++k) x[k]=out.candidate[k]/out.scale;
    if(!noOp) actual=op.storedResidual(out.candidate,old,out.scale,work);
    out.finalRms=Product({out.scale,Rms(actual,work)});
    if(out.finalRms>target) throw std::runtime_error("MAC stored diffusion velocity exceeds residual tolerance.");
    out.finalMean=Product({out.scale,Mean(x,work)});
    out.meanAllowance=Product({out.scale,(16*double(x.size())+128)*std::numeric_limits<double>::epsilon()});
    if(std::abs(out.finalMean-out.initialMean)>out.meanAllowance)
        throw std::runtime_error("MAC diffusion mean drift exceeds roundoff allowance.");
    const double volume=op.c.spacingX*op.c.spacingY;
    out.initialEnergy=PhysicalDot(rhs,rhs,.5,options.density,volume,out.scale,work);
    out.finalEnergy=PhysicalDot(x,x,.5,options.density,volume,out.scale,work);
    std::vector<double> difference(x.size()); work.charge();
    for(std::size_t k=0;k<x.size();++k) difference[k]=NormalizedDifference(out.candidate[k],old[k],out.scale);
    out.incrementEnergy=PhysicalDot(difference,difference,.5,options.density,volume,out.scale,work);
    out.residualWork=PhysicalDot(x,actual,1,options.density,volume,out.scale,work);
    if(!noOp) {
        work.charge(); double dissipation=0;
        for(std::size_t j=0;j<op.c.rows;++j) for(std::size_t i=0;i<op.c.columns;++i) {
            const auto k=i+op.c.columns*j;
            const double dx=NormalizedDifference(out.candidate[k],out.candidate[(i+1)%op.c.columns+op.c.columns*j],out.scale);
            const double dy=NormalizedDifference(out.candidate[k],out.candidate[i+op.c.columns*((j+1)%op.c.rows)],out.scale);
            dissipation=Checked(dissipation+Product({options.density,volume,out.scale,out.scale,op.ax,dx,dx})
                +Product({options.density,volume,out.scale,out.scale,op.ay,dy,dy}));
        }
        out.dissipation=dissipation;
    }
    return out;
}
}
void MacDiffusionConfig::Validate() const {
    if(!std::isfinite(kinematicViscosity)||kinematicViscosity<0||!std::isfinite(timeStep)||timeStep<0
        ||!std::isfinite(density)||density<=0||!std::isfinite(absoluteVelocityTolerance)||absoluteVelocityTolerance<0
        ||!std::isfinite(relativeVelocityTolerance)||relativeVelocityTolerance<0
        ||maximumIterations>MaximumIterations||maximumCellVisits>MaximumCellVisits)
        throw std::invalid_argument("MAC diffusion configuration is invalid.");
}
MacDiffusionDiagnostics PeriodicMacGrid::diffuse(const MacDiffusionConfig& options) {
    options.Validate(); const auto n=config_.columns*config_.rows;
    Work work{0,options.maximumCellVisits,n};
    const bool noOp=options.kinematicViscosity==0||options.timeStep==0;
    const double ix=1/config_.spacingX,iy=1/config_.spacingY;
    const double ax=noOp?0:Product({options.kinematicViscosity,options.timeStep,ix,ix});
    const double ay=noOp?0:Product({options.kinematicViscosity,options.timeStep,iy,iy});
    const Operator op{config_,ax,ay,Checked(1+2*Checked(ax+ay))};
    if((ax!=0&&ax/op.diagonal==0)||(ay!=0&&ay/op.diagonal==0))
        throw std::overflow_error("MAC scaled diffusion coefficient underflows float64 range.");
    // Determine the fixed target from the input, never from a decreasing residual.
    double initialRms=0;
    for(const auto* field:{&velocities_.xFaces,&velocities_.yFaces}) {
        work.charge(); double scale=0; for(double value:*field) scale=std::max(scale,std::abs(value));
        std::vector<double> normalized(n); work.charge();
        if(scale!=0) for(std::size_t k=0;k<n;++k) normalized[k]=(*field)[k]/scale;
        initialRms=Checked(std::hypot(initialRms,Product({scale,Rms(normalized,work)})));
    }
    MacDiffusionDiagnostics d;
    d.kinematicViscosity=options.kinematicViscosity; d.timeStep=options.timeStep; d.density=options.density;
    d.initialVelocityRms=initialRms;
    d.targetResidualRms=std::max(options.absoluteVelocityTolerance,Product({options.relativeVelocityTolerance,initialRms}));
    std::size_t iterations=0;
    const double target=d.targetResidualRms/std::sqrt(2.0);
    const auto x=Solve(velocities_.xFaces,op,options,target,work,iterations,noOp);
    const auto y=Solve(velocities_.yFaces,op,options,target,work,iterations,noOp);
    d.iterations=iterations; d.iterationsX=x.iterations; d.iterationsY=y.iterations;
    d.finalResidualRms=Checked(std::hypot(x.finalRms,y.finalRms));
    if(d.finalResidualRms>d.targetResidualRms) throw std::runtime_error("MAC diffusion total residual exceeds tolerance.");
    d.initialMeanX=x.initialMean; d.initialMeanY=y.initialMean; d.finalMeanX=x.finalMean; d.finalMeanY=y.finalMean;
    d.meanRoundoffAllowanceX=x.meanAllowance; d.meanRoundoffAllowanceY=y.meanAllowance;
    d.initialKineticEnergy=Checked(x.initialEnergy+y.initialEnergy);
    d.finalKineticEnergy=Checked(x.finalEnergy+y.finalEnergy);
    d.gradientDissipation=Checked(x.dissipation+y.dissipation);
    d.incrementKineticEnergy=Checked(x.incrementEnergy+y.incrementEnergy);
    d.residualWork=Checked(x.residualWork+y.residualWork);
    const double finalRms=Checked(std::hypot(Rms(x.candidate,work),Rms(y.candidate,work)));
    d.residualEnergyBound=Product({options.density,config_.spacingX*config_.spacingY,double(n),finalRms,d.finalResidualRms});
    d.storageEnergyError=Checked(d.finalKineticEnergy-d.initialKineticEnergy+d.gradientDissipation+d.incrementKineticEnergy-d.residualWork);
    d.roundoffEnergyAllowance=Product({(32*double(n)+256)*std::numeric_limits<double>::epsilon(),
        std::max({d.initialKineticEnergy,d.finalKineticEnergy,d.gradientDissipation,d.incrementKineticEnergy,std::abs(d.residualWork)})});
    if(d.finalKineticEnergy-d.initialKineticEnergy>d.residualEnergyBound+d.roundoffEnergyAllowance
        ||std::abs(d.storageEnergyError)>d.roundoffEnergyAllowance)
        throw std::runtime_error("MAC diffusion energy exceeds residual or roundoff allowance.");
    d.zeroTransportNoOp=noOp; d.cellVisits=work.count;
    MacVelocityState staged{x.candidate,y.candidate};
    velocities_=std::move(staged); diffusion_=d;
    return d;
}
} // namespace PhysicsEngine

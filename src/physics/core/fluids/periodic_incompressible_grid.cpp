#include "physics/core/fluids/periodic_incompressible_grid.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace PhysicsEngine {
namespace {
double Checked(double x) {
    if(!std::isfinite(x)) throw std::overflow_error("Incompressible arithmetic exceeds float64 range.");
    return x;
}
double Product(std::initializer_list<double> factors) {
    for(double x:factors) if(x==0) return 0;
    double m=1; int exponent=0;
    for(double x:factors) { int e=0; m*=std::frexp(x,&e); exponent+=e; }
    const double result=Checked(std::ldexp(m,exponent));
    if(result==0) throw std::overflow_error("Incompressible nonzero product underflows float64 range.");
    return result;
}
struct Sum {
    double sum=0,correction=0;
    void add(double value) {
        const double next=Checked(sum+value);
        correction=Checked(correction+(std::abs(sum)>=std::abs(value)?(sum-next)+value:(value-next)+sum));
        sum=next;
    }
    double value() const { return Checked(sum+correction); }
};
struct Work {
    std::size_t count=0,limit,n,iterations=0,iterationLimit;
    void charge(std::size_t scans=1) {
        if(scans>(limit-count)/n) throw std::runtime_error("Incompressible cell-visit budget exhausted.");
        count+=scans*n;
    }
    void accept(std::size_t visits,std::size_t used) { count+=visits; iterations+=used; }
};
struct Moments { double x=0,y=0,energy=0,scaleX=0,scaleY=0; };
Moments Measure(const MacVelocityState& v,double mass,Work& work) {
    work.charge(); Sum x,y,e; Moments m;
    for(std::size_t k=0;k<work.n;++k) {
        x.add(v.xFaces[k]/double(work.n)); y.add(v.yFaces[k]/double(work.n));
        e.add(Product({.5,mass,v.xFaces[k],v.xFaces[k]}));
        e.add(Product({.5,mass,v.yFaces[k],v.yFaces[k]}));
        m.scaleX=std::max(m.scaleX,std::abs(v.xFaces[k])); m.scaleY=std::max(m.scaleY,std::abs(v.yFaces[k]));
    }
    m.x=x.value(); m.y=y.value(); m.energy=e.value(); return m;
}
double StoredEnergyChange(const MacVelocityState& a,const MacVelocityState& b,double mass,Work& work) {
    work.charge(2); Sum change;
    for(auto pair:{std::pair<const std::vector<double>*,const std::vector<double>*>{&a.xFaces,&b.xFaces},{&a.yFaces,&b.yFaces}})
        for(std::size_t k=0;k<work.n;++k) {
            const double delta=Checked((*pair.second)[k]-(*pair.first)[k]);
            change.add(Product({mass,delta,(*pair.first)[k]})); change.add(Product({.5,mass,delta,delta}));
        }
    return change.value();
}
double Allowance(double scale,std::size_t n) { return Product({(64*double(n)+512)*std::numeric_limits<double>::epsilon(),scale}); }
MacVelocityState Get(const PeriodicMacGrid& mac,Work& work) { work.charge(2); return mac.velocities(); }
void Set(PeriodicMacGrid& mac,const MacVelocityState& v,Work& work) { work.charge(4); mac.setVelocities(v); }
MacProjectionDiagnostics Project(PeriodicMacGrid& mac,double h,double rho,const IncompressibleStepConfig& c,Work& work) {
    MacProjectionConfig p; p.density=rho; p.timeStep=h;
    p.absoluteDivergenceTolerance=c.absoluteDivergenceTolerance; p.relativeDivergenceTolerance=c.relativeDivergenceTolerance;
    p.maximumIterations=work.iterationLimit-work.iterations; p.maximumCellVisits=work.limit-work.count;
    const auto d=mac.project(p); work.accept(d.cellVisits,d.iterations); return d;
}
MacDiffusionDiagnostics Diffuse(PeriodicMacGrid& mac,double h,const PeriodicIncompressibleGridConfig& g,
                               const IncompressibleStepConfig& c,Work& work) {
    MacDiffusionConfig p; p.density=g.density; p.timeStep=h; p.kinematicViscosity=g.kinematicViscosity;
    p.absoluteVelocityTolerance=c.absoluteVelocityTolerance; p.relativeVelocityTolerance=c.relativeVelocityTolerance;
    p.maximumIterations=work.iterationLimit-work.iterations; p.maximumCellVisits=work.limit-work.count;
    const auto d=mac.diffuse(p); work.accept(d.cellVisits,d.iterations); return d;
}
struct Faces { std::vector<double> x,y; };
struct Advector { Faces u,v; double outgoingRate=0; };
Advector Interpolate(const MacVelocityState& q,const PeriodicMacGridConfig& g,Work& work) {
    work.charge(); Advector a{{std::vector<double>(work.n),std::vector<double>(work.n)},
                            {std::vector<double>(work.n),std::vector<double>(work.n)},0};
    for(std::size_t j=0;j<g.rows;++j) for(std::size_t i=0;i<g.columns;++i) {
        const auto k=i+g.columns*j,r=(i+1)%g.columns+g.columns*j,t=i+g.columns*((j+1)%g.rows);
        const auto lt=(i+g.columns-1)%g.columns+g.columns*((j+1)%g.rows);
        const auto rb=(i+1)%g.columns+g.columns*((j+g.rows-1)%g.rows);
        a.u.x[k]=Checked(.5*q.xFaces[k]+.5*q.xFaces[r]); a.u.y[k]=Checked(.5*q.yFaces[lt]+.5*q.yFaces[t]);
        a.v.x[k]=Checked(.5*q.xFaces[rb]+.5*q.xFaces[r]); a.v.y[k]=Checked(.5*q.yFaces[k]+.5*q.yFaces[t]);
    }
    work.charge(2);
    for(const auto* f:{&a.u,&a.v}) for(std::size_t j=0;j<g.rows;++j) for(std::size_t i=0;i<g.columns;++i) {
        const auto k=i+g.columns*j,l=(i+g.columns-1)%g.columns+g.columns*j,b=i+g.columns*((j+g.rows-1)%g.rows);
        const double rate=Checked(Product({std::max(0.,f->x[k]),1/g.spacingX})+Product({std::max(0.,-f->x[l]),1/g.spacingX})
            +Product({std::max(0.,f->y[k]),1/g.spacingY})+Product({std::max(0.,-f->y[b]),1/g.spacingY}));
        a.outgoingRate=std::max(a.outgoingRate,rate);
    }
    return a;
}
std::vector<double> Transport(const std::vector<double>& q,const Faces& w,const PeriodicMacGridConfig& g,
                              double h,double mass,IncompressibleSubstepDiagnostics& d,Work& work) {
    std::vector<double> fx(work.n),fy(work.n),out(work.n); work.charge();
    for(std::size_t j=0;j<g.rows;++j) for(std::size_t i=0;i<g.columns;++i) {
        const auto k=i+g.columns*j,r=(i+1)%g.columns+g.columns*j,t=i+g.columns*((j+1)%g.rows);
        fx[k]=Product({h,w.x[k],q[w.x[k]>=0?k:r],1/g.spacingX});
        fy[k]=Product({h,w.y[k],q[w.y[k]>=0?k:t],1/g.spacingY});
    }
    Sum divergenceWork,donor,increment; work.charge();
    for(std::size_t j=0;j<g.rows;++j) for(std::size_t i=0;i<g.columns;++i) {
        const auto k=i+g.columns*j,r=(i+1)%g.columns+g.columns*j,t=i+g.columns*((j+1)%g.rows);
        const auto l=(i+g.columns-1)%g.columns+g.columns*j,b=i+g.columns*((j+g.rows-1)%g.rows);
        const double delta=Checked(-Checked(fx[k]-fx[l])-Checked(fy[k]-fy[b]));
        out[k]=Checked(q[k]+delta);
        const double actual=Checked(out[k]-q[k]);
        const double div=Checked(Product({Checked(w.x[k]-w.x[l]),1/g.spacingX})+Product({Checked(w.y[k]-w.y[b]),1/g.spacingY}));
        d.maximumDualDivergence=std::max(d.maximumDualDivergence,std::abs(div));
        // Compute represented weights independently; Jensen bound uses this actual row sum.
        const double outgoing=Checked(Product({h,std::max(0.,w.x[k]),1/g.spacingX})+Product({h,std::max(0.,-w.x[l]),1/g.spacingX})
            +Product({h,std::max(0.,w.y[k]),1/g.spacingY})+Product({h,std::max(0.,-w.y[b]),1/g.spacingY}));
        const double incoming=Checked(Product({h,std::max(0.,w.x[l]),1/g.spacingX})+Product({h,std::max(0.,-w.x[k]),1/g.spacingX})
            +Product({h,std::max(0.,w.y[b]),1/g.spacingY})+Product({h,std::max(0.,-w.y[k]),1/g.spacingY}));
        if(outgoing>1) throw std::runtime_error("Incompressible donor diagonal is negative.");
        d.maximumRowSum=std::max(d.maximumRowSum,Checked((1-outgoing)+incoming));
        divergenceWork.add(Product({-.5,mass,h,div,q[k],q[k]}));
        const double dx=Checked(q[r]-q[k]),dy=Checked(q[t]-q[k]);
        donor.add(Product({.5,mass,h,std::abs(w.x[k]),dx,dx,1/g.spacingX}));
        donor.add(Product({.5,mass,h,std::abs(w.y[k]),dy,dy,1/g.spacingY}));
        increment.add(Product({.5,mass,actual,actual}));
    }
    d.dualDivergenceWork=Checked(d.dualDivergenceWork+divergenceWork.value());
    d.donorDissipation=Checked(d.donorDissipation+donor.value());
    d.forwardEulerIncrementEnergy=Checked(d.forwardEulerIncrementEnergy+increment.value()); return out;
}
void AddProjection(IncompressibleStepDiagnostics& d,const MacProjectionDiagnostics& p,Sum& residual) {
    d.projectionCorrectionEnergy=Checked(d.projectionCorrectionEnergy+p.correctionKineticEnergy);
    residual.add(p.divergencePotentialInnerProduct); d.projectionResidualWork=residual.value();
    d.residualEnergyBound=Checked(d.residualEnergyBound+p.residualEnergyBound);
    d.roundoffEnergyAllowance=Checked(d.roundoffEnergyAllowance+p.roundoffEnergyAllowance);
}
}
void PeriodicIncompressibleGridConfig::Validate() const {
    geometry.Validate();
    if(!std::isfinite(density)||density<=0||!std::isfinite(kinematicViscosity)||kinematicViscosity<0)
        throw std::invalid_argument("Incompressible material parameters are invalid.");
    Product({density,geometry.spacingX,geometry.spacingY});
}
void IncompressibleStepConfig::Validate() const {
    if(!std::isfinite(cflSafety)||cflSafety<=0||cflSafety>=1||maximumSubsteps>MaximumSubsteps
        ||maximumIterations>MaximumIterations||maximumCellVisits>MaximumCellVisits)
        throw std::invalid_argument("Incompressible safety or work ceiling is invalid.");
    for(double t:{absoluteDivergenceTolerance,relativeDivergenceTolerance,absoluteVelocityTolerance,relativeVelocityTolerance})
        if(!std::isfinite(t)||t<0) throw std::invalid_argument("Incompressible tolerance is invalid.");
}
PeriodicIncompressibleGrid::PeriodicIncompressibleGrid(const PeriodicIncompressibleGridConfig& c):config_(c),mac_(c.geometry) { config_.Validate(); }
IncompressibleStepDiagnostics PeriodicIncompressibleGrid::step(double dt,const IncompressibleStepConfig& c) {
    c.Validate();
    if(!std::isfinite(dt)||dt<0) throw std::invalid_argument("Incompressible timestep must be finite and nonnegative.");
    const double endpoint=Checked(time_+dt);
    if(dt>0&&endpoint<=time_) throw std::overflow_error("Incompressible clock increment is unrepresentable.");
    const auto& g=config_.geometry; Work work{0,c.maximumCellVisits,g.columns*g.rows,0,c.maximumIterations};
    const double mass=Product({config_.density,g.spacingX,g.spacingY});
    const auto initial=Get(mac_,work); const auto first=Measure(initial,mass,work);
    IncompressibleStepDiagnostics d; d.timeStep=dt; d.initialTime=time_; d.finalTime=endpoint;
    d.initialMeanX=first.x; d.initialMeanY=first.y; d.initialKineticEnergy=first.energy;
    if(dt==0) {
        d.zeroStepNoOp=true; d.finalMeanX=first.x; d.finalMeanY=first.y; d.finalKineticEnergy=first.energy;
        d.cellVisits=work.count; IncompressibleStepDiagnostics returned=d; step_=std::move(d); return returned;
    }
    work.charge(4); PeriodicMacGrid staged=mac_;
    Sum projectionResidual,dualWork,viscousResidual;
    d.initialProjection=Project(staged,dt,config_.density,c,work); AddProjection(d,d.initialProjection,projectionResidual);
    double elapsed=0; Sum ledger;
    ledger.add(d.initialProjection.divergencePotentialInnerProduct); ledger.add(-d.initialProjection.correctionKineticEnergy);
    double scaleX=first.scaleX,scaleY=first.scaleY;
    while(elapsed<dt) {
        if(d.substeps==c.maximumSubsteps) throw std::runtime_error("Incompressible substep budget exhausted.");
        const auto old=Get(staged,work); const auto before=Measure(old,mass,work); const auto w=Interpolate(old,g,work);
        const double remaining=Checked(dt-elapsed);
        // Compare before dividing by a tiny rate: an unused CFL quotient may overflow.
        const double h=w.outgoingRate<=c.cflSafety/remaining?remaining:Checked(c.cflSafety/w.outgoingRate);
        if(h<=0||elapsed+h<=elapsed) throw std::overflow_error("Incompressible CFL increment is unrepresentable.");
        IncompressibleSubstepDiagnostics s; s.timeStep=h; s.outgoingCfl=Product({h,w.outgoingRate});
        if(s.outgoingCfl>c.cflSafety*(1+8*std::numeric_limits<double>::epsilon())) throw std::runtime_error("Incompressible outgoing CFL exceeded.");
        s.initialKineticEnergy=before.energy;
        MacVelocityState next{Transport(old.xFaces,w.u,g,h,mass,s,work),Transport(old.yFaces,w.v,g,h,mass,s,work)};
        const auto after=Measure(next,mass,work); s.advectedKineticEnergy=after.energy;
        s.advectionEnergyChange=StoredEnergyChange(old,next,mass,work);
        Sum error; error.add(s.advectionEnergyChange); error.add(-s.dualDivergenceWork); error.add(s.donorDissipation); error.add(-s.forwardEulerIncrementEnergy);
        s.advectionStorageError=error.value();
        s.advectionEnergyBound=Product({Checked(s.maximumRowSum-1),before.energy});
        s.roundoffEnergyAllowance=Allowance(std::max({before.energy,after.energy,std::abs(s.dualDivergenceWork),s.donorDissipation,s.forwardEulerIncrementEnergy}),work.n);
        s.meanRoundoffAllowanceX=Allowance(std::max(before.scaleX,after.scaleX),work.n);
        s.meanRoundoffAllowanceY=Allowance(std::max(before.scaleY,after.scaleY),work.n);
        if(std::abs(s.advectionStorageError)>s.roundoffEnergyAllowance||s.advectionEnergyChange>s.advectionEnergyBound+s.roundoffEnergyAllowance
            ||std::abs(after.x-before.x)>s.meanRoundoffAllowanceX||std::abs(after.y-before.y)>s.meanRoundoffAllowanceY)
            throw std::runtime_error("Incompressible stored donor energy or momentum audit failed.");
        Set(staged,next,work); s.diffusion=Diffuse(staged,h,config_,c,work); s.projection=Project(staged,h,config_.density,c,work);
        dualWork.add(s.dualDivergenceWork); d.dualDivergenceWork=dualWork.value();
        d.donorDissipation=Checked(d.donorDissipation+s.donorDissipation);
        d.forwardEulerIncrementEnergy=Checked(d.forwardEulerIncrementEnergy+s.forwardEulerIncrementEnergy);
        d.viscousDissipation=Checked(d.viscousDissipation+s.diffusion.gradientDissipation);
        d.viscousIncrementEnergy=Checked(d.viscousIncrementEnergy+s.diffusion.incrementKineticEnergy);
        viscousResidual.add(s.diffusion.residualWork); d.viscousResidualWork=viscousResidual.value();
        d.residualEnergyBound=Checked(d.residualEnergyBound+s.diffusion.residualEnergyBound);
        d.roundoffEnergyAllowance=Checked(d.roundoffEnergyAllowance+s.roundoffEnergyAllowance+s.diffusion.roundoffEnergyAllowance);
        AddProjection(d,s.projection,projectionResidual);
        ledger.add(s.dualDivergenceWork); ledger.add(-s.donorDissipation); ledger.add(s.forwardEulerIncrementEnergy);
        ledger.add(-s.diffusion.gradientDissipation); ledger.add(-s.diffusion.incrementKineticEnergy); ledger.add(s.diffusion.residualWork);
        ledger.add(s.projection.divergencePotentialInnerProduct); ledger.add(-s.projection.correctionKineticEnergy);
        scaleX=std::max({scaleX,before.scaleX,after.scaleX}); scaleY=std::max({scaleY,before.scaleY,after.scaleY});
        d.history.push_back(s); ++d.substeps;
        elapsed=h==remaining?dt:Checked(elapsed+h);
    }
    const auto final=Get(staged,work); const auto last=Measure(final,mass,work);
    d.finalMeanX=last.x; d.finalMeanY=last.y; d.finalKineticEnergy=last.energy;
    d.meanRoundoffAllowanceX=Product({double(3*d.substeps+1),Allowance(std::max(scaleX,last.scaleX),work.n)});
    d.meanRoundoffAllowanceY=Product({double(3*d.substeps+1),Allowance(std::max(scaleY,last.scaleY),work.n)});
    d.storedEnergyChange=StoredEnergyChange(initial,final,mass,work);
    d.storageEnergyError=Checked(d.storedEnergyChange-ledger.value());
    d.roundoffEnergyAllowance=Checked(d.roundoffEnergyAllowance+Allowance(std::max({first.energy,last.energy,std::abs(ledger.value())}),work.n));
    // Independent final stored divergence, charged separately from operator acceptance.
    work.charge(); const auto div=staged.divergence(); work.charge(); double rms=0;
    for(double x:div) rms=Checked(std::hypot(rms,x/std::sqrt(double(work.n))));
    if(rms>d.history.back().projection.targetDivergenceRms||std::abs(d.storageEnergyError)>d.roundoffEnergyAllowance
        ||std::abs(last.x-first.x)>d.meanRoundoffAllowanceX||std::abs(last.y-first.y)>d.meanRoundoffAllowanceY)
        throw std::runtime_error("Incompressible final stored-state audit failed.");
    d.iterations=work.iterations; d.cellVisits=work.count;
    // Allocate all published and return snapshots before touching the owning state.
    IncompressibleStepDiagnostics returned=d;
    mac_=std::move(staged); time_=endpoint; step_=std::move(d); return returned;
}
} // namespace PhysicsEngine

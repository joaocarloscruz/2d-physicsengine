#include "physics/core/fluids/periodic_mac_grid.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace PhysicsEngine {
namespace {
double Checked(double x) {
    if(!std::isfinite(x)) throw std::overflow_error("MAC projection arithmetic exceeds float64 range.");
    return x;
}
struct Work {
    std::size_t count=0,limit,n;
    void charge() {
        if(n>limit-count) throw std::runtime_error("MAC projection cell-visit budget exhausted.");
        count+=n;
    }
};
std::size_t Index(std::size_t i,std::size_t j,std::size_t nx) { return i+nx*j; }
std::vector<double> Divergence(const MacVelocityState& v,const PeriodicMacGridConfig& c) {
    std::vector<double> out(c.columns*c.rows);
    const double ix=1/c.spacingX,iy=1/c.spacingY;
    for(std::size_t j=0;j<c.rows;++j) for(std::size_t i=0;i<c.columns;++i) {
        const auto k=Index(i,j,c.columns);
        out[k]=Checked(Checked(v.xFaces[Index((i+1)%c.columns,j,c.columns)]-v.xFaces[k])*ix
            +Checked(v.yFaces[Index(i,(j+1)%c.rows,c.columns)]-v.yFaces[k])*iy);
    }
    return out;
}
double Dot(const std::vector<double>& a,const std::vector<double>& b,Work& w) {
    w.charge(); double sum=0;
    for(std::size_t k=0;k<a.size();++k) sum=Checked(sum+Checked(a[k]*b[k]));
    return sum;
}
double Mean(const std::vector<double>& a,Work& w) {
    w.charge(); double sum=0;
    for(double x:a) sum=Checked(sum+x/a.size());
    return sum;
}
double RemoveMean(std::vector<double>& a,Work& w) {
    const double mean=Mean(a,w); w.charge();
    for(double& x:a) x=Checked(x-mean);
    return mean;
}
double Rms(const std::vector<double>& a,Work& w) {
    w.charge(); double norm=0; const double scale=std::sqrt(static_cast<double>(a.size()));
    bool nonzero=false;
    for(double x:a) { nonzero=nonzero||x!=0; norm=Checked(std::hypot(norm,x/scale)); }
    if(nonzero&&norm==0) throw std::overflow_error("MAC RMS norm underflows float64 range.");
    return norm;
}
std::vector<double> Apply(const std::vector<double>& p,const PeriodicMacGridConfig& c,Work& w) {
    w.charge(); std::vector<double> out(p.size());
    const double ax=(1/c.spacingX)*(1/c.spacingX),ay=(1/c.spacingY)*(1/c.spacingY);
    for(std::size_t j=0;j<c.rows;++j) for(std::size_t i=0;i<c.columns;++i) {
        const auto k=Index(i,j,c.columns);
        // Distinct forward/backward edges are retained even when nx or ny is 2.
        out[k]=Checked(ax*((p[k]-p[Index((i+1)%c.columns,j,c.columns)])
            +(p[k]-p[Index((i+c.columns-1)%c.columns,j,c.columns)]))
            +ay*((p[k]-p[Index(i,(j+1)%c.rows,c.columns)])
            +(p[k]-p[Index(i,(j+c.rows-1)%c.rows,c.columns)])));
    }
    return out;
}
MacVelocityState Correct(const MacVelocityState& v,const std::vector<double>& p,const PeriodicMacGridConfig& c,Work& w) {
    w.charge(); MacVelocityState out=v;
    for(std::size_t j=0;j<c.rows;++j) for(std::size_t i=0;i<c.columns;++i) {
        const auto k=Index(i,j,c.columns);
        out.xFaces[k]=Checked(v.xFaces[k]-Checked((p[k]-p[Index((i+c.columns-1)%c.columns,j,c.columns)])/c.spacingX));
        out.yFaces[k]=Checked(v.yFaces[k]-Checked((p[k]-p[Index(i,(j+c.rows-1)%c.rows,c.columns)])/c.spacingY));
    }
    return out;
}
}
void PeriodicMacGridConfig::Validate() const {
    if(columns<2||rows<2||columns>MaximumCells||rows>MaximumCells||columns>MaximumCells/rows)
        throw std::invalid_argument("Periodic MAC grid dimensions exceed supported bounds.");
    if(!std::isfinite(spacingX)||spacingX<=0||!std::isfinite(spacingY)||spacingY<=0)
        throw std::invalid_argument("MAC spacings must be positive and finite.");
    const double ix=1/spacingX,iy=1/spacingY;
    for(double value:{ix,iy,ix*ix,iy*iy,spacingX*spacingY,spacingX*columns,spacingY*rows})
        if(!std::isfinite(value)||value<=0) throw std::invalid_argument("MAC derived geometry is unrepresentable.");
    if(!std::isfinite(2*(ix*ix+iy*iy))) throw std::invalid_argument("MAC Laplacian diagonal is unrepresentable.");
}
void MacProjectionConfig::Validate() const {
    if(!std::isfinite(density)||density<=0||!std::isfinite(timeStep)||timeStep<=0
        ||!std::isfinite(density/timeStep)||density/timeStep<=0)
        throw std::invalid_argument("MAC pressure density/timeStep must be positive and representable.");
    if(!std::isfinite(absoluteDivergenceTolerance)||absoluteDivergenceTolerance<0
        ||!std::isfinite(relativeDivergenceTolerance)||relativeDivergenceTolerance<0
        ||maximumIterations>MaximumIterations||maximumCellVisits>MaximumCellVisits)
        throw std::invalid_argument("MAC projection tolerance or budget is invalid.");
}
PeriodicMacGrid::PeriodicMacGrid(const PeriodicMacGridConfig& config):config_(config) {
    config_.Validate(); const auto n=config_.columns*config_.rows;
    velocities_.xFaces.resize(n); velocities_.yFaces.resize(n);
    projection_.potential.resize(n); projection_.pressure.resize(n);
}
void PeriodicMacGrid::setVelocities(const MacVelocityState& v) {
    const auto n=config_.columns*config_.rows;
    if(v.xFaces.size()!=n||v.yFaces.size()!=n) throw std::invalid_argument("MAC face array size mismatch.");
    for(const auto* a:{&v.xFaces,&v.yFaces}) for(double x:*a)
        if(!std::isfinite(x)) throw std::invalid_argument("MAC velocities must be finite.");
    MacVelocityState staged=v; velocities_=std::move(staged);
}
std::vector<double> PeriodicMacGrid::divergence() const { return Divergence(velocities_,config_); }
MacProjectionDiagnostics PeriodicMacGrid::project(const MacProjectionConfig& options) {
    options.Validate(); const auto n=config_.columns*config_.rows;
    const double volume=config_.spacingX*config_.spacingY,energyScale=Checked(options.density*volume);
    if(energyScale<=0) throw std::overflow_error("MAC cell mass underflows float64 range.");
    Work work{0,options.maximumCellVisits,n};
    work.charge(); const auto initial=Divergence(velocities_,config_);
    MacProjectionSnapshot staged; staged.potential.resize(n); staged.pressure.resize(n);
    auto& d=staged.diagnostics;
    d.density=options.density; d.timeStep=options.timeStep;
    d.initialDivergenceRms=Rms(initial,work);
    d.targetDivergenceRms=std::max(options.absoluteDivergenceTolerance,
        Checked(options.relativeDivergenceTolerance*d.initialDivergenceRms));
    std::vector<double> rhs=initial;
    work.charge(); for(double& x:rhs) x=-x;
    d.removedDivergenceMean=-RemoveMean(rhs,work);
    std::vector<double> residual=rhs,direction=rhs;
    double rr=Dot(residual,residual,work);
    MacVelocityState candidate;
    bool achieved=false;
    // Always validate the actual corrected face divergence, not just recursive CG residuals.
    while(true) {
        if(std::sqrt(rr/n)<=d.targetDivergenceRms||d.iterations==options.maximumIterations) {
            RemoveMean(staged.potential,work);
            candidate=Correct(velocities_,staged.potential,config_,work);
            work.charge(); const auto actual=Divergence(candidate,config_);
            d.finalDivergenceRms=Rms(actual,work);
            if(d.finalDivergenceRms<=d.targetDivergenceRms) { achieved=true; break; }
            if(d.iterations==options.maximumIterations) break;
            const auto ap=Apply(staged.potential,config_,work);
            work.charge(); for(std::size_t k=0;k<n;++k) residual[k]=Checked(rhs[k]-ap[k]);
            RemoveMean(residual,work); direction=residual; rr=Dot(residual,residual,work);
        }
        if(rr==0) break; // Requested accuracy may be below representable face precision.
        const auto ad=Apply(direction,config_,work);
        const double denominator=Dot(direction,ad,work);
        if(denominator<=0) throw std::runtime_error("MAC Poisson iteration lost positive definiteness.");
        const double alpha=Checked(rr/denominator);
        work.charge(); for(std::size_t k=0;k<n;++k) {
            staged.potential[k]=Checked(staged.potential[k]+Checked(alpha*direction[k]));
            residual[k]=Checked(residual[k]-Checked(alpha*ad[k]));
        }
        RemoveMean(residual,work);
        const double next=Dot(residual,residual,work),beta=Checked(next/rr);
        work.charge(); for(std::size_t k=0;k<n;++k) direction[k]=Checked(residual[k]+Checked(beta*direction[k]));
        rr=next; ++d.iterations;
    }
    if(!achieved) throw std::runtime_error("MAC projection did not achieve actual divergence tolerance.");
    d.zeroDivergenceNoOp=d.initialDivergenceRms==0;
    d.initialMeanX=Mean(velocities_.xFaces,work); d.initialMeanY=Mean(velocities_.yFaces,work);
    d.finalMeanX=Mean(candidate.xFaces,work); d.finalMeanY=Mean(candidate.yFaces,work);
    d.potentialMean=Mean(staged.potential,work);
    const double pressureScale=options.density/options.timeStep;
    work.charge(); for(std::size_t k=0;k<n;++k) staged.pressure[k]=Checked(pressureScale*staged.potential[k]);
    d.pressureMean=Mean(staged.pressure,work);
    std::vector<double> cx(n),cy(n);
    work.charge(); for(std::size_t k=0;k<n;++k) {
        cx[k]=Checked(velocities_.xFaces[k]-candidate.xFaces[k]);
        cy[k]=Checked(velocities_.yFaces[k]-candidate.yFaces[k]);
    }
    d.initialKineticEnergy=Checked(.5*energyScale*Checked(Dot(velocities_.xFaces,velocities_.xFaces,work)+Dot(velocities_.yFaces,velocities_.yFaces,work)));
    d.finalKineticEnergy=Checked(.5*energyScale*Checked(Dot(candidate.xFaces,candidate.xFaces,work)+Dot(candidate.yFaces,candidate.yFaces,work)));
    d.correctionKineticEnergy=Checked(.5*energyScale*Checked(Dot(cx,cx,work)+Dot(cy,cy,work)));
    d.velocityCorrectionInnerProduct=Checked(energyScale*Checked(Dot(candidate.xFaces,cx,work)+Dot(candidate.yFaces,cy,work)));
    work.charge(); const auto final=Divergence(candidate,config_);
    d.divergencePotentialInnerProduct=Checked(energyScale*Dot(final,staged.potential,work));
    d.residualEnergyBound=Checked(Checked(energyScale*n)*d.finalDivergenceRms*Rms(staged.potential,work));
    d.storageEnergyError=Checked(d.finalKineticEnergy-d.initialKineticEnergy-d.divergencePotentialInnerProduct+d.correctionKineticEnergy);
    // Scale-aware diagnostic guard, including ordinary N-term summation error.
    // This is not a rigorous bound on every intermediate floating-point operation.
    d.roundoffEnergyAllowance=Checked((16*static_cast<double>(n)+128)*std::numeric_limits<double>::epsilon()*
        std::max({d.initialKineticEnergy,d.finalKineticEnergy,d.correctionKineticEnergy}));
    if(d.finalKineticEnergy>d.initialKineticEnergy+d.residualEnergyBound+d.roundoffEnergyAllowance)
        throw std::runtime_error("MAC projection energy exceeds its residual bound.");
    d.cellVisits=work.count;
    velocities_=std::move(candidate); projection_=std::move(staged);
    return projection_.diagnostics;
}
} // namespace PhysicsEngine

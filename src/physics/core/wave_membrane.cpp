#include "physics/core/wave_membrane.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace PhysicsEngine {
namespace {
double Checked(double value) {
    if (!std::isfinite(value)) throw std::runtime_error("Wave membrane arithmetic overflow");
    return value;
}
double Positive(double value) {
    if (!std::isfinite(value) || value<=0)
        throw std::invalid_argument("Wave membrane coefficient is not positively representable");
    return value;
}
struct Coefficients { double x, y, kinetic, strainX, strainY, limit; };
Coefficients Validate(std::size_t width,std::size_t height,double dx,double dy,const WaveMembraneConfig& c) {
    if ((c.boundary!=WaveBoundary::FixedZero && c.boundary!=WaveBoundary::Periodic) ||
        !std::isfinite(c.damping) || c.damping<0 || !std::isfinite(c.cflSafety) || c.cflSafety<=0 || c.cflSafety>=1 ||
        !std::isfinite(c.maxSubstep) || c.maxSubstep<=0 || c.maxCells==0 || c.maxSubsteps==0 || c.maxCellWork==0)
        throw std::invalid_argument("Invalid wave membrane configuration");
    const std::size_t minimum=c.boundary==WaveBoundary::FixedZero ? 3 : 2;
    if (width<minimum || height<minimum) throw std::invalid_argument("Wave membrane grid is too small for its boundary");
    // Validate the product before multiplication or any vector allocation.
    if (width>c.maxCells/height) throw std::length_error("Wave membrane cell budget exceeded");
    Positive(dx); Positive(dy); Positive(c.tension); Positive(c.surfaceDensity);
    const double cSquared=Positive(c.tension/c.surfaceDensity);
    const double wx=Positive(cSquared/dx/dx), wy=Positive(cSquared/dy/dy);
    const double kinetic=Positive(0.5*c.surfaceDensity*dx*dy);
    const double sx=Positive(0.5*c.tension*dy/dx), sy=Positive(0.5*c.tension*dx/dy);
    // omega_max <= 2*sqrt(wx+wy). Verlet requires h*omega_max<2.
    const double frequency=Positive(std::hypot(std::sqrt(wx),std::sqrt(wy)));
    const double cfl=Positive(c.cflSafety/frequency);
    const double limit=std::nextafter(std::min(c.maxSubstep,cfl),0.0);
    Positive(limit);
    return {wx,wy,kinetic,sx,sy,limit};
}
double SquareEnergy(double coefficient,double value) {
    const double scaled=Checked(std::sqrt(coefficient)*value);
    return Checked(scaled*scaled);
}
}
WaveMembrane::WaveMembrane(std::size_t width,std::size_t height,double dx,double dy,const WaveMembraneConfig& c)
    : width_(width),height_(height),dx_(dx),dy_(dy),config_(c) {
    Validate(width,height,dx,dy,c);
    const auto count=width*height;
    if (count>u_.max_size()) throw std::length_error("Wave membrane grid exceeds vector capacity");
    u_.resize(count); v_.resize(count); loads_.resize(count);
}
std::size_t WaveMembrane::index(std::size_t x,std::size_t y) const {
    if (x>=width_ || y>=height_) throw std::out_of_range("Wave membrane cell out of range");
    return y*width_+x;
}
bool WaveMembrane::fixed(std::size_t x,std::size_t y,WaveBoundary boundary) const {
    return boundary==WaveBoundary::FixedZero && (x==0 || y==0 || x==width_-1 || y==height_-1);
}
void WaveMembrane::validateState(const std::vector<double>& values,WaveBoundary boundary) const {
    if (values.size()!=u_.size()) throw std::invalid_argument("Wave membrane state size mismatch");
    for (std::size_t y=0;y<height_;++y) for (std::size_t x=0;x<width_;++x) {
        const double value=values[y*width_+x];
        if (!std::isfinite(value) || (fixed(x,y,boundary) && value!=0))
            throw std::invalid_argument("Wave membrane state must be finite with zero fixed edges");
    }
}
void WaveMembrane::setConfig(const WaveMembraneConfig& c) {
    Validate(width_,height_,dx_,dy_,c);
    validateState(u_,c.boundary); validateState(v_,c.boundary); validateState(loads_,c.boundary);
    config_=c;
}
void WaveMembrane::setState(const std::vector<double>& u,const std::vector<double>& v) {
    validateState(u,config_.boundary); validateState(v,config_.boundary);
    // Copies can throw; complete both before publishing either one.
    auto newU=u, newV=v; u_.swap(newU); v_.swap(newV);
}
void WaveMembrane::setCellState(std::size_t x,std::size_t y,double u,double v) {
    const auto i=index(x,y);
    if (!std::isfinite(u) || !std::isfinite(v) || (fixed(x,y,config_.boundary) && (u!=0 || v!=0)))
        throw std::invalid_argument("Wave membrane cell state must be finite with zero fixed edges");
    u_[i]=u; v_[i]=v;
}
void WaveMembrane::queueAcceleration(std::size_t x,std::size_t y,double acceleration) {
    const auto i=index(x,y);
    if (!std::isfinite(acceleration) || (fixed(x,y,config_.boundary) && acceleration!=0))
        throw std::invalid_argument("Wave membrane acceleration must be finite with zero fixed edges");
    loads_[i]=Checked(loads_[i]+acceleration);
}
void WaveMembrane::clearAccelerations() noexcept { std::fill(loads_.begin(),loads_.end(),0); }
void WaveMembrane::clearAcceleration(std::size_t x,std::size_t y) { loads_[index(x,y)]=0; }
double WaveMembrane::getStableTimeStep() const { return Validate(width_,height_,dx_,dy_,config_).limit; }
WaveMembraneDiagnostics WaveMembrane::diagnostics(const std::vector<double>& u,const std::vector<double>& v) const {
    const auto c=Validate(width_,height_,dx_,dy_,config_);
    WaveMembraneDiagnostics result;
    result.time=time_; result.stableTimeStep=c.limit; result.lastSubstep=lastSubstep_;
    result.lastSubsteps=lastSubsteps_; result.lastCellWork=lastCellWork_;
    for (std::size_t y=0;y<height_;++y) for (std::size_t x=0;x<width_;++x) {
        const auto i=y*width_+x;
        result.kineticEnergy=Checked(result.kineticEnergy+SquareEnergy(c.kinetic,v[i]));
        result.maxAbsDisplacement=std::max(result.maxAbsDisplacement,std::abs(u[i]));
        result.maxAbsVelocity=std::max(result.maxAbsVelocity,std::abs(v[i]));
        if (x+1<width_ || config_.boundary==WaveBoundary::Periodic) {
            const auto j=y*width_+(x+1==width_ ? 0 : x+1);
            result.strainEnergy=Checked(result.strainEnergy+SquareEnergy(c.strainX,Checked(u[j]-u[i])));
        }
        if (y+1<height_ || config_.boundary==WaveBoundary::Periodic) {
            const auto j=(y+1==height_ ? 0 : y+1)*width_+x;
            result.strainEnergy=Checked(result.strainEnergy+SquareEnergy(c.strainY,Checked(u[j]-u[i])));
        }
    }
    result.totalEnergy=Checked(result.kineticEnergy+result.strainEnergy); return result;
}
WaveMembraneDiagnostics WaveMembrane::getDiagnostics() const { return diagnostics(u_,v_); }
void WaveMembrane::step(double dt) {
    if (!std::isfinite(dt) || dt<0) throw std::invalid_argument("Invalid wave membrane timestep");
    if (dt==0) { lastSubstep_=0; lastSubsteps_=lastCellWork_=0; return; }
    const auto c=Validate(width_,height_,dx_,dy_,config_);
    // Comparison and conversion are guarded against count rounding and overflow.
    const double ratio=dt/c.limit;
    if (!std::isfinite(ratio) || ratio>=std::ldexp(1.0,std::numeric_limits<std::size_t>::digits))
        throw std::length_error("Wave membrane substep budget exceeded");
    const double requested=std::max(1.0,std::ceil(ratio));
    if (requested>=std::ldexp(1.0,std::numeric_limits<std::size_t>::digits))
        throw std::length_error("Wave membrane substep count is not representable");
    auto count=static_cast<std::size_t>(requested);
    const auto budget=std::min(config_.maxSubsteps,config_.maxCellWork/u_.size());
    if (count>budget)
        throw std::length_error("Wave membrane substep or cell work budget exceeded");
    double h=dt/count;
    // A quotient rounded onto an integer can hide a fraction above the bound.
    // Add a partition rather than silently accepting that oversized substep.
    if (h>c.limit) {
        if (count>=budget) throw std::length_error("Wave membrane substep or cell work budget exceeded");
        h=dt/++count;
    }
    if (!(0.5*h>0) || h>c.limit) throw std::runtime_error("Wave membrane substep is not safely representable");
    const double nextTime=Checked(time_+dt);
    if (nextTime==time_) throw std::runtime_error("Wave membrane time increment is not representable");
    auto u=u_, v=v_; std::vector<double> acceleration(u.size());
    const double decay=std::exp(-config_.damping*h);
    auto accelerate=[&]() {
        for (std::size_t y=0;y<height_;++y) for (std::size_t x=0;x<width_;++x) {
            const auto i=y*width_+x;
            if (fixed(x,y,config_.boundary)) { acceleration[i]=0; continue; }
            const auto left=y*width_+(x==0 ? width_-1 : x-1), right=y*width_+(x+1==width_ ? 0 : x+1);
            const auto down=(y==0 ? height_-1 : y-1)*width_+x, up=(y+1==height_ ? 0 : y+1)*width_+x;
            const double xx=Checked(Checked(u[left]-u[i])+Checked(u[right]-u[i]));
            const double yy=Checked(Checked(u[down]-u[i])+Checked(u[up]-u[i]));
            acceleration[i]=Checked(Checked(c.x*xx)+Checked(c.y*yy)+loads_[i]);
        }
    };
    for (std::size_t n=0;n<count;++n) {
        accelerate();
        for (std::size_t i=0;i<u.size();++i) {
            v[i]=Checked(decay*v[i]+Checked((0.5*h)*acceleration[i]));
            u[i]=Checked(u[i]+Checked(h*v[i]));
        }
        accelerate();
        for (std::size_t i=0;i<u.size();++i)
            v[i]=Checked(decay*Checked(v[i]+Checked((0.5*h)*acceleration[i])));
    }
    // Reject unrepresentable physical diagnostics before committing state/loads.
    diagnostics(u,v);
    u_.swap(u); v_.swap(v); clearAccelerations(); time_=nextTime;
    lastSubstep_=h; lastSubsteps_=count; lastCellWork_=count*u_.size();
}
}

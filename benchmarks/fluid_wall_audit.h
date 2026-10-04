#pragma once
#include "physics/core/fluids/wcsph_solver.h"
#include "physics/core/fluids/sph_kernels.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace FluidWallAudit {
struct D2 {
    double x=0,y=0;
    D2 operator+(D2 b) const { return {x+b.x,y+b.y}; }
    D2 operator-(D2 b) const { return {x-b.x,y-b.y}; }
    D2 operator*(double s) const { return {x*s,y*s}; }
    double dot(D2 b) const { return x*b.x+y*b.y; }
    double norm() const { return std::hypot(x,y); }
};
inline D2 ToDouble(PhysicsEngine::Vector2 v) { return {v.x,v.y}; }
struct PairOperator { std::size_t a,b; D2 gradient,force; };
struct WallOperator { std::size_t fluid,wall; D2 gradient,force; double pressure; };
struct Operators {
    std::vector<PairOperator> pairs;
    std::vector<WallOperator> walls;
    std::vector<D2> fluidForces,wallForces,gravityForces,localAdjointWallForces,signedExtrapolationWallForces;
};
struct WorkMetrics {
    double pairMechanicalWork=0,pairInternalEnergyRate=0,pairWorkResidual=0;
    double wallFluidWork=0,inferredReactionWork=0,wallRelativeMechanicalWork=0;
    double wallInternalEnergyRate=0,wallWorkResidual=0,gravityWork=0;
    double maximumAbsoluteDensityRate=0,meanDensityRate=0;
    std::vector<double> densityRates;
};
inline double HydrostaticDensityRatio(double depth,double g=9.81,double c=40,double gamma=7) {
    return std::pow(1+(gamma-1)*g*std::max(depth,0.0)/(c*c),1/(gamma-1));
}
inline double HydrostaticPressure(double depth,double rho0=1000,double g=9.81,double c=40,double gamma=7) {
    const double ratio=HydrostaticDensityRatio(depth,g,c,gamma);
    return rho0*c*c/gamma*(std::pow(ratio,gamma)-1);
}
// Diagnostic-only independent reconstruction of the prepared pressure and raw
// continuity operators. No viscosity/density diffusion or integration is used.
inline Operators Build(const std::vector<PhysicsEngine::FluidParticle>& p,
    const std::vector<PhysicsEngine::FluidBoundaryParticle>& b,const PhysicsEngine::WcsphConfig& config) {
    using namespace PhysicsEngine;
    config.Validate();
    if(p.size()>5000 || b.size()>10000) throw std::length_error("Wall diagnostic state exceeds its bounded particle budget.");
    Operators op;
    op.fluidForces.resize(p.size()); op.wallForces.resize(p.size()); op.gravityForces.resize(p.size()); op.localAdjointWallForces.resize(p.size()); op.signedExtrapolationWallForces.resize(p.size());
    for(std::size_t i=0;i<p.size();++i) {
        if(!std::isfinite(p[i].density)||p[i].density<=0||!std::isfinite(p[i].pressure)||
            !std::isfinite(p[i].mass)||p[i].mass<=0||!std::isfinite(p[i].restDensity)||p[i].restDensity<=0)
            throw std::invalid_argument("Wall audit requires valid prepared thermodynamic state.");
        op.gravityForces[i]=ToDouble(config.externalAcceleration)*p[i].mass;
        for(std::size_t j=0;j<i;++j) {
            const float h=static_cast<float>(0.5*(static_cast<double>(p[i].smoothingLength)+p[j].smoothingLength));
            const D2 gradient=ToDouble(SphKernels2D::PressureGradient(p[i].position-p[j].position,h,config.kernelFamily));
            if(gradient.norm()==0) continue;
            const double scale=-static_cast<double>(p[i].mass)*p[j].mass*(p[i].pressure/(static_cast<double>(p[i].density)*p[i].density)
                +p[j].pressure/(static_cast<double>(p[j].density)*p[j].density));
            const D2 force=gradient*scale;
            op.pairs.push_back({i,j,gradient,force});
            op.fluidForces[i]=op.fluidForces[i]+force; op.fluidForces[j]=op.fluidForces[j]-force;
        }
    }
    std::vector<double> numerator(b.size(),0),denominator(b.size(),0),signedNumerator(b.size(),0);
    for(std::size_t j=0;j<b.size();++j) {
        if(!std::isfinite(b[j].volume)||b[j].volume<=0||!std::isfinite(b[j].pressureScale)||b[j].pressureScale<0)
            throw std::invalid_argument("Wall audit requires valid boundary volume and pressure scale.");
        if(b[j].pressureScale==0) continue;
        for(std::size_t i=0;i<p.size();++i) {
            const auto displacement=p[i].position-b[j].position;
            const double radius=ToDouble(displacement).norm();
            if(radius==0 || radius>=p[i].smoothingLength) continue;
            const double weight=SphKernels2D::DensityWeight(displacement,p[i].smoothingLength,config.kernelFamily);
            const double projected=std::max(0.0,(ToDouble(config.externalAcceleration)-ToDouble(b[j].acceleration)).dot(ToDouble(displacement)*(-1/radius)));
            numerator[j]+=(p[i].pressure+p[i].density*radius*projected)*weight;
            signedNumerator[j]+=(p[i].pressure-p[i].density*(ToDouble(config.externalAcceleration)-ToDouble(b[j].acceleration)).dot(ToDouble(displacement)))*weight;
            denominator[j]+=weight;
        }
    }
    for(std::size_t j=0;j<b.size();++j) if(b[j].pressureScale>0) {
        for(std::size_t i=0;i<p.size();++i) {
            const auto displacement=p[i].position-b[j].position;
            const double radius=ToDouble(displacement).norm();
            if(radius==0 || radius>=p[i].smoothingLength) continue;
            const D2 gradient=ToDouble(SphKernels2D::PressureGradient(displacement,p[i].smoothingLength,config.kernelFamily));
            if(gradient.norm()==0) continue;
            double pressure=denominator[j]>0 ? numerator[j]/denominator[j] : p[i].pressure;
            if(config.wallPressureMode==WcsphWallPressureMode::SignedBodyForce) {
                pressure=denominator[j]>0 ? signedNumerator[j]/denominator[j] : p[i].pressure;
                if(config.clampNegativePressure) pressure=std::max(0.0,pressure);
            }
            const double volume=static_cast<double>(p[i].mass)/p[i].density;
            const D2 force=gradient*(-volume*b[j].volume*(p[i].pressure+pressure)*b[j].pressureScale);
            op.walls.push_back({i,j,gradient,force,pressure}); op.wallForces[i]=op.wallForces[i]+force;
            // Counterfactual exact local-fluid pressure adjoint of the current
            // raw mirror rate. Diagnostic only: it removes extrapolated p_b.
            op.localAdjointWallForces[i]=op.localAdjointWallForces[i]+gradient*(-2*volume*b[j].volume*p[i].pressure);
            double signedPressure=denominator[j]>0?signedNumerator[j]/denominator[j]:p[i].pressure;
            if(config.clampNegativePressure) signedPressure=std::max(0.0,signedPressure);
            op.signedExtrapolationWallForces[i]=op.signedExtrapolationWallForces[i]+gradient*(-volume*b[j].volume*(p[i].pressure+signedPressure)*b[j].pressureScale);
        }
    }
    return op;
}
struct PhaseProbe { double normalizedNormalForce,normalAcceleration,pressure; };
// Constant-pressure half-space stencil: rho=1010 and caller mass=rho*dx^2
// make physical volume quadrature identical in fluid and wall samples.
inline PhaseProbe FlatPhaseProbe(PhysicsEngine::SphKernelFamily family,float dx,float ratio,bool staggered) {
    using namespace PhysicsEngine;
    if(!std::isfinite(dx)||dx<=0||!std::isfinite(ratio)||ratio<1||ratio>16)
        throw std::invalid_argument("Invalid bounded flat phase probe dimensions.");
    const float h=dx*ratio;
    FluidParticleProperties properties;properties.restDensity=1000;properties.mass=1010*dx*dx;properties.smoothingLength=h;properties.viscosity=0;
    const int extent=static_cast<int>(std::ceil(ratio))+1;
    std::vector<FluidParticle> particles;
    for(int y=1;y<=extent+1;++y) for(int x=-extent;x<=extent;++x) {
        particles.emplace_back(Vector2{x*dx,y*dx},Vector2{},properties);particles.back().density=1010;
    }
    std::vector<FluidBoundaryParticle> walls;
    for(int y=0;y<extent;++y) for(int x=-extent;x<=extent;++x)
        walls.push_back({{(x+(staggered?0.5f:0))*dx,-y*dx},{},dx*dx,{},1});
    WcsphConfig config;config.kernelFamily=family;config.speedOfSound=20;config.equationOfStateExponent=7;config.externalAcceleration={};config.densityMode=WcsphDensityMode::Continuity;config.densityDiffusion=0;
    WcsphSolver solver(h,config);solver.prepare(particles,walls);
    const auto& point=particles[static_cast<std::size_t>(extent)];
    return {point.force.y/(static_cast<double>(point.pressure)*dx),point.force.y/static_cast<double>(point.mass),point.pressure};
}

inline WorkMetrics MeasureWork(const Operators& op,const std::vector<PhysicsEngine::FluidParticle>& p,
    const std::vector<PhysicsEngine::FluidBoundaryParticle>& b) {
    WorkMetrics m; m.densityRates.resize(p.size()); std::vector<double> wallRates(p.size()),pairRates(p.size());
    for(const auto& pair:op.pairs) {
        const D2 relative=ToDouble(p[pair.a].velocity)-ToDouble(p[pair.b].velocity);
        const double rate=relative.dot(pair.gradient);
        pairRates[pair.a]+=p[pair.b].mass*rate; pairRates[pair.b]+=p[pair.a].mass*rate;
        m.pairMechanicalWork+=pair.force.dot(relative);
    }
    for(const auto& pair:op.walls) {
        const D2 velocity=ToDouble(p[pair.fluid].velocity),wallVelocity=ToDouble(b[pair.wall].velocity);
        // G is radial: projecting onto r before this dot product is redundant.
        wallRates[pair.fluid]+=2.0*p[pair.fluid].density*b[pair.wall].volume*(velocity-wallVelocity).dot(pair.gradient);
        m.wallFluidWork+=pair.force.dot(velocity); m.inferredReactionWork-=pair.force.dot(wallVelocity);
        m.wallRelativeMechanicalWork+=pair.force.dot(velocity-wallVelocity);
    }
    for(std::size_t i=0;i<p.size();++i) {
        const double weight=static_cast<double>(p[i].mass)*p[i].pressure/(static_cast<double>(p[i].density)*p[i].density);
        m.pairInternalEnergyRate+=weight*pairRates[i]; m.wallInternalEnergyRate+=weight*wallRates[i];
        m.densityRates[i]=pairRates[i]+wallRates[i];
        m.maximumAbsoluteDensityRate=std::max(m.maximumAbsoluteDensityRate,std::abs(m.densityRates[i])/p[i].restDensity);
        m.meanDensityRate+=m.densityRates[i]/p[i].restDensity;
        m.gravityWork+=op.gravityForces[i].dot(ToDouble(p[i].velocity));
    }
    if(!p.empty()) m.meanDensityRate/=p.size();
    m.pairWorkResidual=m.pairMechanicalWork+m.pairInternalEnergyRate;
    m.wallWorkResidual=m.wallRelativeMechanicalWork+m.wallInternalEnergyRate;
    return m;
}
}

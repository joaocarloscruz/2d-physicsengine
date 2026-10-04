#pragma once
// Bounded experiment only. This is not a production boundary or SPH API.
#include "physics/core/fluids/sph_kernels.h"
#include <cmath>
#include <stdexcept>
#include <vector>

namespace PlanarReflection {
struct Vec {
    double x=0,y=0;
    Vec operator+(Vec b) const { return {x+b.x,y+b.y}; }
    Vec operator-(Vec b) const { return {x-b.x,y-b.y}; }
    Vec operator*(double a) const { return {x*a,y*a}; }
    double dot(Vec b) const { return x*b.x+y*b.y; }
    double cross(Vec b) const { return x*b.y-y*b.x; }
    double norm() const { return std::hypot(x,y); }
    Vec rotate() const { return {-y,x}; }
};
inline bool Finite(Vec a) { return std::isfinite(a.x)&&std::isfinite(a.y); }
struct Particle { Vec position,velocity; double mass=1,density=1000,pressure=1000; };
struct Plane {
    Vec point,normal{0,1},center,velocity;
    double angularVelocity=0;
    Vec ReflectVector(Vec v) const { return v-normal*(2*v.dot(normal)); }
    double Depth(Vec x) const { return (x-point).dot(normal); }
    Vec Projection(Vec x) const { return x-normal*Depth(x); }
    Vec ReflectPosition(Vec x) const { return x-normal*(2*Depth(x)); }
    Vec VelocityAt(Vec x) const { return velocity+(x-center).rotate()*angularVelocity; }
    // Derivative of the reflected position when the plane itself rotates.
    Vec AngularGhostMap(Vec x) const {
        return normal*(2*normal.dot((Projection(x)-center).rotate()))
            -normal.rotate()*(2*Depth(x));
    }
    Vec GhostVelocity(const Particle& p) const {
        return ReflectVector(p.velocity)+normal*(2*normal.dot(velocity))
            +AngularGhostMap(p.position)*angularVelocity;
    }
};
constexpr double Pi=3.1415926535897932384626433832795;
inline void ValidateFamily(PhysicsEngine::SphKernelFamily family) {
    if (family != PhysicsEngine::SphKernelFamily::Poly6Spiky &&
        family != PhysicsEngine::SphKernelFamily::CubicSpline)
        throw std::invalid_argument("Planar reflection experiment supports legacy and cubic kernels only.");
}
// Independent double formulas preserve physical units and permit clean work tests.
inline double Weight(Vec r,double h,PhysicsEngine::SphKernelFamily family) {
    ValidateFamily(family);
    const double q=r.norm()/h;
    if(q>=1) return 0;
    if(family==PhysicsEngine::SphKernelFamily::Poly6Spiky) {
        const double t=1-q*q; return 4/(Pi*h*h)*t*t*t;
    }
    const double u=2*q,t=2-u;
    return 40/(7*Pi*h*h)*(u<1?1-1.5*u*u+.75*u*u*u:.25*t*t*t);
}
inline Vec Gradient(Vec r,double h,PhysicsEngine::SphKernelFamily family) {
    ValidateFamily(family);
    const double radius=r.norm(),q=radius/h;
    if(radius==0||q>=1) return {};
    double derivative;
    if(family==PhysicsEngine::SphKernelFamily::Poly6Spiky)
        derivative=-30/(Pi*h*h*h)*(1-q)*(1-q);
    else {
        const double u=2*q,t=2-u;
        derivative=40/(7*Pi*h*h*h)*(u<1?-6*u+4.5*u*u:-1.5*t*t);
    }
    return r*(derivative/radius);
}
struct Result {
    std::vector<double> densityRates,summedDensities;
    std::vector<Vec> pairForces,ghostForces,forces;
    Vec wallForce,totalForce;
    double wallTorque=0,wallProjectionTorque=0,wallCouple=0,totalTorque=0;
    double fluidWork=0,wallWork=0,internalEnergyRate=0,workResidual=0;
};
inline Result Evaluate(const std::vector<Particle>& p,const Plane& plane,double h,
    PhysicsEngine::SphKernelFamily family) {
    ValidateFamily(family);
    if(p.size()>1024) throw std::length_error("Planar experiment supports at most 1024 particles.");
    if(!Finite(plane.point)||!Finite(plane.normal)||!Finite(plane.center)||!Finite(plane.velocity)
        ||!std::isfinite(plane.angularVelocity)||std::abs(plane.normal.norm()-1)>1e-12
        ||!std::isfinite(h)||h<=0) throw std::invalid_argument("Invalid planar experiment geometry.");
    for(const auto& a:p)
        if(!Finite(a.position)||!Finite(a.velocity)||!std::isfinite(a.mass)||a.mass<=0
            ||!std::isfinite(a.density)||a.density<=0||!std::isfinite(a.pressure)
            ||!std::isfinite(plane.Depth(a.position))||plane.Depth(a.position)<=0)
            throw std::invalid_argument("Invalid source-owned fluid state.");
    for(const auto& a:p)
        if(!Finite(plane.ReflectPosition(a.position))||!Finite(plane.GhostVelocity(a)))
            throw std::overflow_error("Reflected geometry exceeds double range.");
    Result out;
    out.densityRates.resize(p.size()); out.summedDensities.resize(p.size());
    out.pairForces.resize(p.size()); out.ghostForces.resize(p.size()); out.forces.resize(p.size());
    for(std::size_t i=0;i<p.size();++i) {
        const double dual=(p[i].mass/p[i].density)*(p[i].pressure/p[i].density);
        for(std::size_t j=0;j<p.size();++j) {
            const Vec real=p[i].position-p[j].position;
            const Vec reflected=p[i].position-plane.ReflectPosition(p[j].position);
            out.summedDensities[i]+=p[j].mass*(Weight(real,h,family)+Weight(reflected,h,family));
            if(i!=j) {
                const Vec g=Gradient(real,h,family);
                out.densityRates[i]+=p[j].mass*(p[i].velocity-p[j].velocity).dot(g);
                const Vec f=g*(-dual*p[j].mass);
                out.pairForces[i]=out.pairForces[i]+f;
                out.pairForces[j]=out.pairForces[j]-f;
            }
            const Vec g=Gradient(reflected,h,family),f=g*(-dual*p[j].mass);
            out.densityRates[i]+=p[j].mass*(p[i].velocity-plane.GhostVelocity(p[j])).dot(g);
            out.ghostForces[i]=out.ghostForces[i]+f;
            out.ghostForces[j]=out.ghostForces[j]-plane.ReflectVector(f);
            const Vec reaction=plane.normal*(2*dual*p[j].mass*plane.normal.dot(g));
            const double torque=dual*p[j].mass*plane.AngularGhostMap(p[j].position).dot(g);
            out.wallForce=out.wallForce+reaction;
            out.wallTorque+=torque;
            out.wallProjectionTorque+=(plane.Projection(p[j].position)-plane.center).cross(reaction);
        }
        out.internalEnergyRate+=dual*out.densityRates[i];
    }
    out.wallCouple=out.wallTorque-out.wallProjectionTorque;
    out.totalForce=out.wallForce;
    out.totalTorque=out.wallTorque+plane.center.cross(out.wallForce);
    for(std::size_t i=0;i<p.size();++i) {
        out.forces[i]=out.pairForces[i]+out.ghostForces[i];
        out.totalForce=out.totalForce+out.forces[i];
        out.totalTorque+=p[i].position.cross(out.forces[i]);
        out.fluidWork+=p[i].velocity.dot(out.forces[i]);
    }
    out.wallWork=plane.velocity.dot(out.wallForce)+plane.angularVelocity*out.wallTorque;
    out.workResidual=out.fluidWork+out.wallWork+out.internalEnergyRate;
    for(std::size_t i=0;i<p.size();++i)
        if(!Finite(out.forces[i])||!std::isfinite(out.densityRates[i])||!std::isfinite(out.summedDensities[i]))
            throw std::overflow_error("Planar source arithmetic exceeds double range.");
    if(!Finite(out.totalForce)||!std::isfinite(out.totalTorque)||!std::isfinite(out.workResidual)
        ||!std::isfinite(out.wallCouple)||!std::isfinite(out.wallProjectionTorque)
        ||!std::isfinite(out.fluidWork)||!std::isfinite(out.wallWork)||!std::isfinite(out.internalEnergyRate))
        throw std::overflow_error("Planar experiment arithmetic exceeds double range.");
    return out;
}
} // namespace PlanarReflection

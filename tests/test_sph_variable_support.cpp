#include "catch_amalgamated.hpp"
#include "physics/core/fluids/wcsph_solver.h"
#include "physics/core/fluids/sph_kernels.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

using namespace PhysicsEngine;
namespace {
constexpr double Pi = 3.1415926535897932384626433832795;
struct D2 {
    double x = 0, y = 0;
    D2 operator+(D2 b) const { return {x+b.x,y+b.y}; }
    D2 operator-(D2 b) const { return {x-b.x,y-b.y}; }
    D2 operator*(double s) const { return {x*s,y*s}; }
    double dot(D2 b) const { return x*b.x+y*b.y; }
    double norm() const { return std::hypot(x,y); }
};
double Weight(D2 r, double h, SphKernelFamily family) {
    const double q=r.norm()/h;
    if(q>=1) return 0;
    if(family==SphKernelFamily::WendlandC2)
        return 7/(Pi*h*h)*std::pow(1-q,4)*(1+4*q);
    const double shape=q<.5 ? 1-6*q*q+6*q*q*q : 2*std::pow(1-q,3);
    return 40/(7*Pi*h*h)*shape;
}
D2 Gradient(D2 r, double h, SphKernelFamily family) {
    const double length=r.norm(), q=length/h;
    if(length==0 || q>=1) return {};
    if(family==SphKernelFamily::WendlandC2)
        return r*(-140/(Pi*h*h*h*h)*std::pow(1-q,3));
    const double derivative=q<.5 ? -12*q+18*q*q : -6*(1-q)*(1-q);
    return r*(40/(7*Pi*h*h*h)*derivative/length);
}
double Pressure(double density, double rest, const WcsphConfig& c) {
    double p=rest*double(c.speedOfSound)*c.speedOfSound/c.equationOfStateExponent*
        (std::pow(density/rest,c.equationOfStateExponent)-1);
    return c.clampNegativePressure ? std::max(0.,p) : p;
}
// Integral of p(rho)/rho^2, referenced to rho0. Fixed supplied h is essential.
double SpecificEnergy(double density, double rest, const WcsphConfig& c) {
    const double s=density/rest, gamma=c.equationOfStateExponent;
    if(c.clampNegativePressure && s<=1) return 0;
    return double(c.speedOfSound)*c.speedOfSound/gamma*
        ((std::pow(s,gamma-1)-1)/(gamma-1)+1/s-1);
}
std::vector<D2> Positions(const std::vector<FluidParticle>& p) {
    std::vector<D2> result; for(const auto& a:p)result.push_back({a.position.x,a.position.y});
    return result;
}
std::vector<double> Densities(const std::vector<FluidParticle>& p,const std::vector<D2>& x,SphKernelFamily family) {
    std::vector<double> result(p.size(),0);
    for(std::size_t i=0;i<p.size();++i)for(std::size_t j=0;j<p.size();++j)
        result[i]+=p[j].mass*Weight(x[i]-x[j],p[i].smoothingLength,family);
    return result;
}
double Energy(const std::vector<FluidParticle>& p,const std::vector<D2>& x,const WcsphConfig& c) {
    const auto density=Densities(p,x,c.kernelFamily); double energy=0;
    for(std::size_t i=0;i<p.size();++i)energy+=p[i].mass*SpecificEnergy(density[i],p[i].restDensity,c);
    return energy;
}
struct Oracle {
    std::vector<double> density,rate;
    std::vector<D2> force;
    double energyRate=0,work=0;
};
Oracle Evaluate(const std::vector<FluidParticle>& p,const WcsphConfig& c) {
    const auto x=Positions(p);Oracle result;result.density=Densities(p,x,c.kernelFamily);
    result.rate.assign(p.size(),0);result.force.resize(p.size());
    for(std::size_t i=0;i<p.size();++i)for(std::size_t j=i+1;j<p.size();++j) {
        const auto gi=Gradient(x[i]-x[j],p[i].smoothingLength,c.kernelFamily);
        const auto gj=Gradient(x[i]-x[j],p[j].smoothingLength,c.kernelFamily);
        const double ai=Pressure(result.density[i],p[i].restDensity,c)/std::pow(result.density[i],2);
        const double aj=Pressure(result.density[j],p[j].restDensity,c)/std::pow(result.density[j],2);
        const auto f=(gi*ai+gj*aj)*(-double(p[i].mass)*p[j].mass);
        result.force[i]=result.force[i]+f;result.force[j]=result.force[j]-f;
        const D2 relative{double(p[i].velocity.x)-p[j].velocity.x,double(p[i].velocity.y)-p[j].velocity.y};
        result.rate[i]+=p[j].mass*relative.dot(gi);result.rate[j]+=p[i].mass*relative.dot(gj);
    }
    for(std::size_t i=0;i<p.size();++i) {
        result.energyRate+=p[i].mass*Pressure(result.density[i],p[i].restDensity,c)/
            std::pow(result.density[i],2)*result.rate[i];
        result.work+=result.force[i].dot({p[i].velocity.x,p[i].velocity.y});
    }
    return result;
}
FluidParticle Particle(Vector2 x,Vector2 v,float mass,float h,float rest=1) {
    FluidParticleProperties properties;properties.mass=mass;properties.smoothingLength=h;
    properties.restDensity=rest;properties.viscosity=0;return FluidParticle(x,v,properties);
}
WcsphConfig Config() {
    WcsphConfig c;c.externalAcceleration={};c.kernelFamily=SphKernelFamily::CubicSpline;
    c.speedOfSound=2;c.equationOfStateExponent=3;return c;
}
std::vector<FluidParticle> Triangle() {
    return {Particle({-.1875f,-.125f},{.25f,-.125f},.75f,.5f),
        Particle({.4375f,.125f},{-.125f,.375f},1.25f,1.25f),
        Particle({-.125f,.625f},{.125f,.0625f},1.625f,.875f,8)};
}
} // namespace

TEST_CASE("Matched summation pressure acts through one sided density support",
          "[fluid][variable-support][energy]") {
    auto c=Config();c.kernelFamily=GENERATE(SphKernelFamily::CubicSpline,SphKernelFamily::WendlandC2);
    const float distance=GENERATE(.875f,.9375f);
    const bool reversed=GENERATE(false,true);
    auto p=std::vector<FluidParticle>{Particle({0,0},{.25f,0},.75f,.5f),
        Particle({distance,0},{-.125f,0},1.25f,1.25f)};
    if(reversed)std::swap(p[0],p[1]);
    const auto expected=Evaluate(p,c);WcsphSolver solver(.5f,c);solver.prepare(p);
    REQUIRE(Gradient({-distance,0},.5,c.kernelFamily).norm()==0);
    REQUIRE(Gradient({-distance,0},.875,c.kernelFamily).norm()==0);
    REQUIRE(expected.force[reversed?1:0].x<0);
    REQUIRE(p[0].force.x==Catch::Approx(expected.force[0].x).epsilon(0).margin(6e-6*std::abs(expected.force[0].x)));
    REQUIRE(p[0].force.y==0);
    REQUIRE(p[1].force.x==-p[0].force.x);
    // The comparison diagnostic deliberately keeps its common mean-h gradient.
    REQUIRE(solver.getDiagnostics().maximumCompressionRate==0);
    REQUIRE(expected.rate[reversed?0:1]>0);
    REQUIRE(p[0].density==Catch::Approx(expected.density[0]).epsilon(2e-7));
    REQUIRE(p[1].density==Catch::Approx(expected.density[1]).epsilon(2e-7));
}

TEST_CASE("Heterogeneous matched pressure is the fixed support summation EOS energy gradient",
          "[fluid][variable-support][energy][conservation]") {
    auto c=Config();c.kernelFamily=GENERATE(SphKernelFamily::CubicSpline,SphKernelFamily::WendlandC2);c.clampNegativePressure=GENERATE(false,true);
    auto p=Triangle();const auto x=Positions(p);const auto expected=Evaluate(p,c);
    WcsphSolver solver(.5f,c);solver.prepare(p);
    REQUIRE((c.clampNegativePressure ? p[2].pressure==0 : p[2].pressure<0));
    D2 net;double torque=0,forceScale=0,work=0;
    for(std::size_t i=0;i<p.size();++i) {
        const D2 actual{p[i].force.x,p[i].force.y};forceScale+=actual.norm();
        REQUIRE((actual-expected.force[i]).norm()<6e-6*expected.force[i].norm());
        REQUIRE(p[i].density==Catch::Approx(expected.density[i]).epsilon(3e-7));
        net=net+actual;torque+=x[i].x*actual.y-x[i].y*actual.x;
        work+=actual.dot({p[i].velocity.x,p[i].velocity.y});
        // Differentiate independently recomputed densities and EOS energy by
        // moving one coordinate only; all masses, rest densities and h stay fixed.
        for(int axis=0;axis<2;++axis) {
            auto left=x,right=x;const double delta=1e-5;
            if(axis==0){left[i].x-=delta;right[i].x+=delta;}
            else{left[i].y-=delta;right[i].y+=delta;}
            const double finiteDifference=-(Energy(p,right,c)-Energy(p,left,c))/(2*delta);
            const double component=axis==0?actual.x:actual.y;
            REQUIRE(component==Catch::Approx(finiteDifference).epsilon(0).margin(6e-6*expected.force[i].norm()));
        }
    }
    REQUIRE(net.norm()<2e-7*forceScale);
    REQUIRE(std::abs(torque)<2e-7*forceScale);
    REQUIRE(std::abs(expected.work+expected.energyRate)<2e-14*std::abs(expected.work));
    REQUIRE(std::abs(work+expected.energyRate)<6e-6*std::abs(expected.work));
    double errors[3];
    for(int k=0;k<3;++k) {
        const double delta=std::ldexp(.001,-k);auto left=x,right=x;
        for(std::size_t i=0;i<p.size();++i) {
            const D2 v{p[i].velocity.x,p[i].velocity.y};left[i]=x[i]-v*delta;right[i]=x[i]+v*delta;
        }
        errors[k]=std::abs((Energy(p,right,c)-Energy(p,left,c))/(2*delta)-expected.energyRate);
    }
    REQUIRE(errors[0]/errors[1]>3.8);REQUIRE(errors[0]/errors[1]<4.2);
    REQUIRE(errors[1]/errors[2]>3.8);REQUIRE(errors[1]/errors[2]<4.2);
}

TEST_CASE("Unequal support matched pressure retains momentum through repeated finite steps",
          "[fluid][variable-support][conservation]") {
    auto c=Config();c.kernelFamily=GENERATE(SphKernelFamily::CubicSpline,SphKernelFamily::WendlandC2);c.clampNegativePressure=false;auto p=Triangle();
    const auto moment=[](const std::vector<FluidParticle>& particles) {
        D2 linear;double angular=0;
        for(const auto& a:particles) {
            linear=linear+D2{a.velocity.x,a.velocity.y}*a.mass;
            angular+=a.mass*(double(a.position.x)*a.velocity.y-double(a.position.y)*a.velocity.x);
        }
        return std::vector<double>{linear.x,linear.y,angular};
    };
    const auto initial=moment(p);WcsphSolver solver(.5f,c);
    for(int i=0;i<250;++i)solver.step(p,.0001f);
    const auto final=moment(p);
    for(std::size_t i=0;i<initial.size();++i)
        REQUIRE(final[i]==Catch::Approx(initial[i]).epsilon(0).margin(2e-6));
    for(const auto& a:p) {
        REQUIRE(std::isfinite(a.position.x));REQUIRE(std::isfinite(a.position.y));
        REQUIRE(std::isfinite(a.velocity.x));REQUIRE(std::isfinite(a.velocity.y));
        REQUIRE(std::isfinite(a.density));REQUIRE(a.density>0);
    }
}

TEST_CASE("Existing common gradient pressure paths retain their exact pair arithmetic",
          "[fluid][variable-support][compatibility]") {
    for(auto family:{SphKernelFamily::Poly6Spiky,SphKernelFamily::CubicSpline,SphKernelFamily::WendlandC2})
    for(auto mode:{WcsphDensityMode::Summation,WcsphDensityMode::Continuity}) {
        auto c=Config();c.kernelFamily=family;c.densityMode=mode;c.densityDiffusion=0;
        auto p=std::vector<FluidParticle>{Particle({-.125f,.125f},{.25f,-.125f},.75f,.5f),
            Particle({.25f,.375f},{-.125f,.375f},1.25f,
                family!=SphKernelFamily::Poly6Spiky && mode==WcsphDensityMode::Summation ? .5f:1.25f)};
        p[0].density=2;p[1].density=3;
        WcsphSolver solver(.5f,c);solver.prepare(p);
        const float h=float(.5*(double(p[0].smoothingLength)+p[1].smoothingLength));
        const auto gradient=SphKernels2D::PressureGradient(p[0].position-p[1].position,h,family);
        const double term=p[0].pressure/(double(p[0].density)*p[0].density)+
            p[1].pressure/(double(p[1].density)*p[1].density);
        const double scale=-double(p[0].mass)*p[1].mass*term;
        REQUIRE(p[0].force.x==float(gradient.x*scale));REQUIRE(p[0].force.y==float(gradient.y*scale));
        REQUIRE(p[1].force.x==-p[0].force.x);REQUIRE(p[1].force.y==-p[0].force.y);
        if(mode==WcsphDensityMode::Continuity) {
            const float rate=(p[0].velocity-p[1].velocity).dot(gradient);
            REQUIRE(p[0].densityRate==p[1].mass*rate);REQUIRE(p[1].densityRate==p[0].mass*rate);
            const double work=double(p[0].force.x)*(double(p[0].velocity.x)-p[1].velocity.x)+
                double(p[0].force.y)*(double(p[0].velocity.y)-p[1].velocity.y);
            // Test the common-gradient adjoint before densityRate storage rounding;
            // the published float rates are independently checked above.
            const double exactRate=(double(p[0].velocity.x)-p[1].velocity.x)*gradient.x+
                (double(p[0].velocity.y)-p[1].velocity.y)*gradient.y;
            const double energyRate=double(p[0].mass)*p[1].mass*term*exactRate;
            const double workScale=std::abs(double(p[0].force.x)*(double(p[0].velocity.x)-p[1].velocity.x))+
                std::abs(double(p[0].force.y)*(double(p[0].velocity.y)-p[1].velocity.y));
            REQUIRE(std::abs(work+energyRate)<=2*std::numeric_limits<float>::epsilon()*workScale);
        }
    }
}

#include "catch_amalgamated.hpp"
#include "physics/core/wave_membrane.h"
#include <algorithm>
#include <cmath>
#include <limits>

using namespace PhysicsEngine;
namespace {
constexpr double Pi=3.14159265358979323846;
WaveMembraneConfig Periodic(double substep=0.001) {
    WaveMembraneConfig c; c.boundary=WaveBoundary::Periodic; c.maxSubstep=substep; return c;
}
double DiscreteFrequency(std::size_t nx,std::size_t ny,double dx,double dy,int kx,int ky,bool periodic,double speed=1) {
    const double ax=periodic ? Pi*kx/nx : Pi*kx/(2*(nx-1));
    const double ay=periodic ? Pi*ky/ny : Pi*ky/(2*(ny-1));
    return 2*speed*std::hypot(std::sin(ax)/dx,std::sin(ay)/dy);
}
std::vector<double> Mode(std::size_t nx,std::size_t ny,int kx,int ky,bool periodic) {
    std::vector<double> result(nx*ny);
    for(std::size_t y=0;y<ny;++y) for(std::size_t x=0;x<nx;++x) {
        if(periodic) result[y*nx+x]=std::cos(2*Pi*kx*x/nx+2*Pi*ky*y/ny);
        else if(x!=0 && y!=0 && x!=nx-1 && y!=ny-1)
            result[y*nx+x]=std::sin(Pi*kx*x/(nx-1))*std::sin(Pi*ky*y/(ny-1));
    }
    return result;
}
double ModeError(const WaveMembrane& s,const std::vector<double>& initial,double amplitude,bool velocity=false) {
    const auto& actual=velocity ? s.getVelocities() : s.getDisplacements(); double sum=0;
    for(std::size_t i=0;i<actual.size();++i) sum+=(actual[i]-initial[i]*amplitude)*(actual[i]-initial[i]*amplitude);
    return std::sqrt(sum/actual.size());
}
}
TEST_CASE("Wave membrane constant periodic fields and acceleration are exact", "[wave]") {
    WaveMembrane s(4,3,0.5,2,Periodic()); std::vector<double> u(12,3),v(12,-2);
    s.setState(u,v);
    for(std::size_t y=0;y<3;++y) for(std::size_t x=0;x<4;++x) s.queueAcceleration(x,y,4);
    s.step(0); REQUIRE(s.getQueuedAccelerations()==std::vector<double>(12,4)); REQUIRE(s.getDisplacements()==u);
    s.step(0.5);
    for(auto value:s.getDisplacements()) REQUIRE(value==Catch::Approx(2.5).epsilon(0).margin(3e-13));
    for(auto value:s.getVelocities()) REQUIRE(value==Catch::Approx(0).epsilon(0).margin(3e-13));
    REQUIRE(s.getQueuedAccelerations()==std::vector<double>(12,0)); REQUIRE(s.getDiagnostics().strainEnergy==0);
    REQUIRE(s.getDiagnostics().time==0.5);
}
TEST_CASE("Wave membrane Verlet follows independent discrete eigenmodes", "[wave]") {
    for(bool periodic : {false,true}) {
        auto c=Periodic(0.002); if(!periodic) c.boundary=WaveBoundary::FixedZero;
        c.tension=6; c.surfaceDensity=2;
        constexpr std::size_t nx=11,ny=9; const double dx=0.2,dy=0.35;
        WaveMembrane s(nx,ny,dx,dy,c); const auto initial=Mode(nx,ny,2,1,periodic);
        s.setState(initial,std::vector<double>(nx*ny)); s.step(0.27);
        const auto d=s.getDiagnostics(); const double omega=DiscreteFrequency(nx,ny,dx,dy,2,1,periodic,std::sqrt(3.0));
        const double theta=2*std::asin(omega*d.lastSubstep/2);
        const double angle=theta*d.lastSubsteps;
        REQUIRE(ModeError(s,initial,std::cos(angle))<2e-14);
        const double velocityAmplitude=-omega*std::sqrt(1-std::pow(omega*d.lastSubstep/2,2))*std::sin(angle);
        REQUIRE(ModeError(s,initial,velocityAmplitude,true)<2e-13);
        REQUIRE(d.lastCellWork==nx*ny*d.lastSubsteps); REQUIRE(d.lastSubstep<=d.stableTimeStep);
    }
}
TEST_CASE("Wave membrane temporal order is second against semidiscrete Fourier solution", "[wave]") {
    constexpr std::size_t nx=12,ny=8; const double dx=0.3,dy=0.2,t=0.4;
    const auto initial=Mode(nx,ny,1,1,true); double errors[3]; int k=0;
    const double omega=DiscreteFrequency(nx,ny,dx,dy,1,1,true);
    for(double h : {0.02,0.01,0.005}) {
        WaveMembrane s(nx,ny,dx,dy,Periodic(h)); s.setState(initial,std::vector<double>(nx*ny)); s.step(t);
        errors[k++]=ModeError(s,initial,std::cos(omega*t));
    }
    REQUIRE(errors[0]/errors[1]>3.5); REQUIRE(errors[0]/errors[1]<4.5);
    REQUIRE(errors[1]/errors[2]>3.5); REQUIRE(errors[1]/errors[2]<4.5);
}
TEST_CASE("Wave membrane spatial order is second against continuum anisotropic solution", "[wave]") {
    double errors[3]; int k=0; const double lx=2,ly=3,t=0.37;
    const double continuum=std::hypot(2*Pi/lx,2*Pi/ly);
    for(std::size_t n : {8,16,32}) {
        const std::size_t ny=n/2;
        WaveMembrane s(n,ny,lx/n,ly/ny,Periodic(0.0001));
        const auto initial=Mode(n,ny,1,1,true); s.setState(initial,std::vector<double>(n*ny)); s.step(t);
        errors[k++]=ModeError(s,initial,std::cos(continuum*t));
    }
    REQUIRE(errors[0]/errors[1]>3.7); REQUIRE(errors[0]/errors[1]<4.3);
    REQUIRE(errors[1]/errors[2]>3.7); REQUIRE(errors[1]/errors[2]<4.3);
}
TEST_CASE("Wave membrane physical energy uses each grid interface once", "[wave]") {
    auto c=Periodic(); c.tension=3; c.surfaceDensity=2;
    WaveMembrane s(2,2,2,3,c); s.setState({1,-1,1,-1},{2,2,2,2});
    const auto d=s.getDiagnostics();
    REQUIRE(d.kineticEnergy==Catch::Approx(96).epsilon(0).margin(3e-14));
    // Four periodic x interfaces, including the two wrap interfaces. No y gradient.
    REQUIRE(d.strainEnergy==Catch::Approx(36).epsilon(0).margin(2e-14));
    auto f=c; f.boundary=WaveBoundary::FixedZero;
    WaveMembrane fixed(3,3,2,3,f); fixed.setCellState(1,1,2,3);
    REQUIRE(fixed.getDiagnostics().kineticEnergy==Catch::Approx(54).epsilon(0).margin(3e-14));
    REQUIRE(fixed.getDiagnostics().strainEnergy==Catch::Approx(26).epsilon(0).margin(2e-14));
}
TEST_CASE("Wave membrane undamped energy stays bounded over many periods", "[wave]") {
    auto c=Periodic(1); c.cflSafety=0.8;
    WaveMembrane s(10,8,0.2,0.3,c); const auto initial=Mode(10,8,4,3,true);
    s.setState(initial,std::vector<double>(80)); const double energy=s.getDiagnostics().totalEnergy;
    for(int n=0;n<1000;++n) {
        s.step(s.getStableTimeStep()*0.9);
        REQUIRE(s.getDiagnostics().totalEnergy<=energy*(1+1e-12));
        REQUIRE(s.getDiagnostics().totalEnergy>energy*0.4);
    }
}
TEST_CASE("Wave membrane exact damping velocity and forced split convergence", "[wave]") {
    constexpr double gamma=0.7,t=0.8,initialV=2,load=3;
    double errors[3]; int k=0;
    for(double h : {0.04,0.02,0.01}) {
        auto c=Periodic(h); c.damping=gamma;
        WaveMembrane damped(3,4,1,1,c); damped.setState(std::vector<double>(12),std::vector<double>(12,initialV));
        damped.step(t);
        const double velocity=initialV*std::exp(-2*gamma*t);
        REQUIRE(damped.getVelocities()[0]==Catch::Approx(velocity).epsilon(0).margin(3e-14));
        const double exactU=initialV*(1-std::exp(-2*gamma*t))/(2*gamma);
        REQUIRE(std::abs(damped.getDisplacements()[0]-exactU)<0.0002);
        REQUIRE(damped.getDiagnostics().kineticEnergy<0.12*0.5*12*initialV*initialV);
        WaveMembrane forced(3,4,1,1,c); forced.setState(std::vector<double>(12),std::vector<double>(12,initialV));
        for(std::size_t y=0;y<4;++y) for(std::size_t x=0;x<3;++x) forced.queueAcceleration(x,y,load);
        forced.step(t);
        const double equilibrium=load/(2*gamma);
        const double exactV=equilibrium+(initialV-equilibrium)*std::exp(-2*gamma*t);
        const double exactForcedU=equilibrium*t+(initialV-equilibrium)*(1-std::exp(-2*gamma*t))/(2*gamma);
        errors[k++]=std::hypot(forced.getVelocities()[0]-exactV,forced.getDisplacements()[0]-exactForcedU);
    }
    REQUIRE(errors[0]/errors[1]>3.5); REQUIRE(errors[0]/errors[1]<4.5);
    REQUIRE(errors[1]/errors[2]>3.5); REQUIRE(errors[1]/errors[2]<4.5);
}
TEST_CASE("Wave membrane damped spatial mode follows independent analytical decay", "[wave]") {
    auto c=Periodic(0.00025); c.damping=0.4;
    WaveMembrane s(12,8,0.3,0.2,c); const auto initial=Mode(12,8,1,1,true);
    s.setState(initial,std::vector<double>(96)); const double energy=s.getDiagnostics().totalEnergy;
    const double omega=DiscreteFrequency(12,8,0.3,0.2,1,1,true), omegaD=std::sqrt(omega*omega-c.damping*c.damping),t=1;
    s.step(t);
    const double amplitude=std::exp(-c.damping*t)*(std::cos(omegaD*t)+c.damping/omegaD*std::sin(omegaD*t));
    const double velocity=-omega*omega/omegaD*std::exp(-c.damping*t)*std::sin(omegaD*t);
    REQUIRE(ModeError(s,initial,amplitude)<3e-7); REQUIRE(ModeError(s,initial,velocity,true)<3e-7);
    REQUIRE(s.getDiagnostics().totalEnergy<energy*0.6);
}
TEST_CASE("Wave membrane fixed edges remain zero and reject state and loads", "[wave][validation]") {
    WaveMembrane s(6,5,0.2,0.3); s.setCellState(2,2,1,3); s.queueAcceleration(3,2,2); s.step(0.02);
    for(std::size_t y=0;y<5;++y) for(std::size_t x=0;x<6;++x) if(x==0 || y==0 || x==5 || y==4) {
        const auto i=y*6+x; REQUIRE(s.getDisplacements()[i]==0); REQUIRE(s.getVelocities()[i]==0);
        REQUIRE_THROWS_AS(s.setCellState(x,y,1),std::invalid_argument);
        REQUIRE_THROWS_AS(s.setCellState(x,y,0,1),std::invalid_argument);
        REQUIRE_THROWS_AS(s.queueAcceleration(x,y,1),std::invalid_argument);
    }
    auto invalid=s.getDisplacements(); invalid[0]=1; const auto before=s.getDisplacements();
    REQUIRE_THROWS_AS(s.setState(invalid,s.getVelocities()),std::invalid_argument); REQUIRE(s.getDisplacements()==before);
    auto badVelocity=s.getVelocities(); badVelocity[0]=1; auto changed=before; changed[2*6+2]=9;
    REQUIRE_THROWS_AS(s.setState(changed,badVelocity),std::invalid_argument); REQUIRE(s.getDisplacements()==before);
    auto c=s.getConfig(); c.boundary=WaveBoundary::Periodic; s.setConfig(c); s.queueAcceleration(0,0,1);
    c.boundary=WaveBoundary::FixedZero; REQUIRE_THROWS_AS(s.setConfig(c),std::invalid_argument);
    REQUIRE(s.getConfig().boundary==WaveBoundary::Periodic); s.clearAcceleration(0,0); s.setCellState(0,0,1);
    REQUIRE_THROWS_AS(s.setConfig(c),std::invalid_argument); s.setCellState(0,0,0,1);
    REQUIRE_THROWS_AS(s.setConfig(c),std::invalid_argument); s.setCellState(0,0,0); s.setConfig(c);
}
TEST_CASE("Wave membrane validates dimensions physical coefficients and configuration", "[wave][validation]") {
    auto c=Periodic(); const double inf=std::numeric_limits<double>::infinity(),nan=std::numeric_limits<double>::quiet_NaN();
    REQUIRE_THROWS_AS(WaveMembrane(0,4,1,1,c),std::invalid_argument);
    REQUIRE_THROWS_AS(WaveMembrane(2,2,1,1),std::invalid_argument);
    REQUIRE_THROWS_AS(WaveMembrane(std::numeric_limits<std::size_t>::max(),2,1,1,c),std::length_error);
    for(double bad : {0.0,-1.0,inf,nan}) {
        REQUIRE_THROWS_AS(WaveMembrane(4,4,bad,1,c),std::invalid_argument);
        REQUIRE_THROWS_AS(WaveMembrane(4,4,1,bad,c),std::invalid_argument);
        auto invalid=c; invalid.tension=bad; REQUIRE_THROWS_AS(WaveMembrane(4,4,1,1,invalid),std::invalid_argument);
        invalid=c; invalid.surfaceDensity=bad; REQUIRE_THROWS_AS(WaveMembrane(4,4,1,1,invalid),std::invalid_argument);
        invalid=c; invalid.maxSubstep=bad; REQUIRE_THROWS_AS(WaveMembrane(4,4,1,1,invalid),std::invalid_argument);
    }
    for(double bad : {-1.0,inf,nan}) { auto invalid=c; invalid.damping=bad; REQUIRE_THROWS_AS(WaveMembrane(4,4,1,1,invalid),std::invalid_argument); }
    for(double bad : {0.0,1.0,-1.0,inf,nan}) { auto invalid=c; invalid.cflSafety=bad; REQUIRE_THROWS_AS(WaveMembrane(4,4,1,1,invalid),std::invalid_argument); }
    auto invalid=c; invalid.boundary=static_cast<WaveBoundary>(17); REQUIRE_THROWS_AS(WaveMembrane(4,4,1,1,invalid),std::invalid_argument);
    invalid=c; invalid.maxCells=0; REQUIRE_THROWS_AS(WaveMembrane(4,4,1,1,invalid),std::invalid_argument);
    invalid=c; invalid.maxSubsteps=0; REQUIRE_THROWS_AS(WaveMembrane(4,4,1,1,invalid),std::invalid_argument);
    invalid=c; invalid.maxCellWork=0; REQUIRE_THROWS_AS(WaveMembrane(4,4,1,1,invalid),std::invalid_argument);
    REQUIRE_THROWS_AS(WaveMembrane(4,4,1e-200,1,c),std::invalid_argument);
    REQUIRE_THROWS_AS(WaveMembrane(4,4,1e200,1,c),std::invalid_argument);
    WaveMembrane s(4,4,1,1,c); invalid=c; invalid.maxCells=15; REQUIRE_THROWS_AS(s.setConfig(invalid),std::length_error);
    REQUIRE(s.getConfig().maxCells==c.maxCells);
    REQUIRE_THROWS_AS(s.setCellState(4,0,0),std::out_of_range); REQUIRE_THROWS_AS(s.queueAcceleration(0,4,0),std::out_of_range);
    REQUIRE_THROWS_AS(s.setCellState(0,0,nan),std::invalid_argument); REQUIRE_THROWS_AS(s.queueAcceleration(0,0,inf),std::invalid_argument);
    REQUIRE_THROWS_AS(s.setState({1},{0}),std::invalid_argument);
}
TEST_CASE("Wave membrane budgets and failed steps retain state loads and diagnostics", "[wave][validation]") {
    auto exactBudget=Periodic(); exactBudget.maxCells=4; exactBudget.maxSubsteps=1; exactBudget.maxCellWork=4;
    WaveMembrane exact(2,2,1,1,exactBudget); exact.step(0.0005);
    REQUIRE(exact.getDiagnostics().lastSubsteps==1); REQUIRE(exact.getDiagnostics().lastCellWork==4);
    REQUIRE_THROWS_AS(WaveMembrane(2,3,1,1,exactBudget),std::length_error);
    auto c=Periodic(0.01); c.maxSubsteps=3; c.maxCellWork=48;
    WaveMembrane s(4,4,1,1,c); s.queueAcceleration(1,1,2); s.step(0);
    const auto beforeU=s.getDisplacements(),beforeV=s.getVelocities(),beforeLoads=s.getQueuedAccelerations();
    for(double bad : {-1.0,std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::infinity(),1e300,0.04}) {
        REQUIRE_THROWS(s.step(bad)); REQUIRE(s.getDisplacements()==beforeU); REQUIRE(s.getVelocities()==beforeV);
        REQUIRE(s.getQueuedAccelerations()==beforeLoads); REQUIRE(s.getDiagnostics().lastSubsteps==0); REQUIRE(s.getDiagnostics().time==0);
    }
    c.maxSubsteps=100; c.maxCellWork=31; s.setConfig(c); REQUIRE_THROWS_AS(s.step(0.02),std::length_error);
    REQUIRE(s.getQueuedAccelerations()==beforeLoads);
    c.maxCellWork=1600; s.setConfig(c); s.step(0.02); REQUIRE(s.getQueuedAccelerations()==std::vector<double>(16,0));
    const auto d=s.getDiagnostics(); s.queueAcceleration(0,0,1e308); s.queueAcceleration(0,0,1e308/2);
    const auto overflowLoad=s.getQueuedAccelerations(); REQUIRE_THROWS(s.queueAcceleration(0,0,1e308)); REQUIRE(s.getQueuedAccelerations()==overflowLoad);
    const auto u=s.getDisplacements(),v=s.getVelocities(); REQUIRE_THROWS(s.step(0.01));
    REQUIRE(s.getDisplacements()==u); REQUIRE(s.getVelocities()==v); REQUIRE(s.getQueuedAccelerations()==overflowLoad);
    REQUIRE(s.getDiagnostics().lastSubsteps==d.lastSubsteps); REQUIRE(s.getDiagnostics().time==d.time);
    s.clearAccelerations(); s.step(0); REQUIRE(s.getDiagnostics().lastCellWork==0); REQUIRE(s.getDiagnostics().time==d.time);
    s.queueAcceleration(0,0,1); REQUIRE_THROWS_AS(s.step(1e-30),std::runtime_error);
    REQUIRE(s.getQueuedAccelerations()[0]==1); REQUIRE(s.getDiagnostics().time==d.time);
    WaveMembrane tiny(2,2,1,1,Periodic()); tiny.queueAcceleration(0,0,1);
    REQUIRE_THROWS_AS(tiny.step(std::numeric_limits<double>::denorm_min()),std::runtime_error);
    REQUIRE(tiny.getQueuedAccelerations()[0]==1); REQUIRE(tiny.getDiagnostics().time==0);
}
TEST_CASE("Wave membrane CFL is strict on anisotropic grids", "[wave]") {
    auto c=Periodic(10); c.tension=8; c.surfaceDensity=2; c.cflSafety=0.95;
    WaveMembrane s(3,4,0.2,0.5,c); const double h=s.getStableTimeStep();
    REQUIRE(4*h*h*(1/0.04+1/0.25)<1);
    REQUIRE(h==Catch::Approx(0.95/(2*std::hypot(5.0,2.0))).epsilon(0).margin(2e-17));
    s.step(h*3.2); const auto d=s.getDiagnostics(); REQUIRE(d.lastSubsteps==4); REQUIRE(d.lastSubstep<=h);
}
TEST_CASE("Wave membrane rejects nonfinite stencil state and physical energies transactionally", "[wave][validation]") {
    WaveMembrane s(2,2,1,1,Periodic()); s.setState({1e308,-1e308,0,0},std::vector<double>(4));
    s.queueAcceleration(1,1,2); const auto before=s.getDisplacements(),loads=s.getQueuedAccelerations();
    REQUIRE_THROWS_AS(s.getDiagnostics(),std::runtime_error); REQUIRE_THROWS_AS(s.step(0.001),std::runtime_error);
    REQUIRE(s.getDisplacements()==before); REQUIRE(s.getVelocities()==std::vector<double>(4)); REQUIRE(s.getQueuedAccelerations()==loads);
    WaveMembrane energy(2,2,1,1,Periodic()); energy.setState(std::vector<double>(4),std::vector<double>(4,1e200));
    const auto v=energy.getVelocities(); energy.queueAcceleration(0,0,1);
    REQUIRE_THROWS_AS(energy.step(0.001),std::runtime_error); REQUIRE(energy.getVelocities()==v);
    REQUIRE(energy.getDisplacements()==std::vector<double>(4)); REQUIRE(energy.getQueuedAccelerations()[0]==1);
}

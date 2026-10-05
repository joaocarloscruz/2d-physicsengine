#include "catch_amalgamated.hpp"
#include "physics/core/fluids/periodic_incompressible_grid.h"
#include <algorithm>
#include <cmath>
#include <complex>
#include <limits>
using namespace PhysicsEngine;
namespace {
constexpr double pi=3.14159265358979323846;
PeriodicIncompressibleGridConfig Config(std::size_t nx=8,std::size_t ny=6) {
    PeriodicIncompressibleGridConfig c; c.geometry={nx,ny,2*pi/double(nx),2*pi/double(ny)}; return c;
}
MacVelocityState TaylorGreen(const PeriodicIncompressibleGridConfig& c,double amplitude=1) {
    const auto& g=c.geometry; MacVelocityState q; q.xFaces.resize(g.columns*g.rows); q.yFaces.resize(q.xFaces.size());
    for(std::size_t j=0;j<g.rows;++j) for(std::size_t i=0;i<g.columns;++i) {
        q.xFaces[i+g.columns*j]=amplitude*std::sin(i*g.spacingX)*std::cos((j+.5)*g.spacingY);
        q.yFaces[i+g.columns*j]=-amplitude*std::cos((i+.5)*g.spacingX)*std::sin(j*g.spacingY);
    }
    return q;
}
void Equal(const MacVelocityState& a,const MacVelocityState& b) { REQUIRE(a.xFaces==b.xFaces); REQUIRE(a.yFaces==b.yFaces); }
double Error(const MacVelocityState& a,const MacVelocityState& b) {
    double e=0; for(std::size_t k=0;k<a.xFaces.size();++k) e+=std::pow(a.xFaces[k]-b.xFaces[k],2)+std::pow(a.yFaces[k]-b.yFaces[k],2);
    return std::sqrt(e/a.xFaces.size());
}
// Independent dense linear algebra: gauge replaces the last Poisson equation.
std::vector<double> DenseSolve(std::vector<std::vector<double>> a,std::vector<double> b) {
    const auto n=b.size();
    for(std::size_t i=0;i<n;++i) {
        std::size_t pivot=i; for(std::size_t j=i+1;j<n;++j) if(std::abs(a[j][i])>std::abs(a[pivot][i])) pivot=j;
        std::swap(a[i],a[pivot]); std::swap(b[i],b[pivot]); REQUIRE(std::abs(a[i][i])>1e-14);
        const double diagonal=a[i][i]; for(std::size_t k=i;k<n;++k) a[i][k]/=diagonal; b[i]/=diagonal;
        for(std::size_t j=0;j<n;++j) if(j!=i) { const double factor=a[j][i]; for(std::size_t k=i;k<n;++k) a[j][k]-=factor*a[i][k]; b[j]-=factor*b[i]; }
    }
    return b;
}
std::size_t Id(int i,int j,const PeriodicMacGridConfig& g) { return std::size_t((i+int(g.columns))%int(g.columns))+g.columns*std::size_t((j+int(g.rows))%int(g.rows)); }
std::vector<std::vector<double>> DenseOperator(const PeriodicMacGridConfig& g,double identity,double scale) {
    const auto n=g.columns*g.rows; std::vector<std::vector<double>> a(n,std::vector<double>(n));
    for(int j=0;j<int(g.rows);++j) for(int i=0;i<int(g.columns);++i) {
        const auto k=Id(i,j,g); a[k][k]=identity;
        for(auto offset:{std::pair<int,int>{1,0},{-1,0},{0,1},{0,-1}}) {
            const double weight=scale/std::pow(offset.first?g.spacingX:g.spacingY,2);
            a[k][k]+=weight; a[k][Id(i+offset.first,j+offset.second,g)]-=weight;
        }
    }
    return a;
}
std::vector<double> DenseProject(MacVelocityState& q,const PeriodicMacGridConfig& g) {
    const auto n=q.xFaces.size(); std::vector<double> rhs(n);
    for(int j=0;j<int(g.rows);++j) for(int i=0;i<int(g.columns);++i) {
        const auto k=Id(i,j,g); rhs[k]=-(q.xFaces[Id(i+1,j,g)]-q.xFaces[k])/g.spacingX-(q.yFaces[Id(i,j+1,g)]-q.yFaces[k])/g.spacingY;
    }
    auto a=DenseOperator(g,0,1); a.back().assign(n,1); rhs.back()=0; auto potential=DenseSolve(a,rhs);
    for(int j=0;j<int(g.rows);++j) for(int i=0;i<int(g.columns);++i) {
        const auto k=Id(i,j,g); q.xFaces[k]-=(potential[k]-potential[Id(i-1,j,g)])/g.spacingX; q.yFaces[k]-=(potential[k]-potential[Id(i,j-1,g)])/g.spacingY;
    }
    return potential;
}
MacVelocityState IndependentFlux(const MacVelocityState& q,const PeriodicMacGridConfig& g,double h) {
    auto next=q;
    for(int component=0;component<2;++component) {
        const auto& field=component?q.yFaces:q.xFaces; auto& out=component?next.yFaces:next.xFaces;
        for(int j=0;j<int(g.rows);++j) for(int i=0;i<int(g.columns);++i) {
            double change=0;
            // Visit both oriented faces of this dual cell (separate edges on two-cell axes).
            for(int axis=0;axis<2;++axis) for(int side:{-1,1}) {
                const int x=i-(axis==0&&side==-1),y=j-(axis==1&&side==-1);
                const auto lo=Id(x,y,g),hi=Id(x+(axis==0),y+(axis==1),g);
                double speed;
                if(component==0&&axis==0) speed=(q.xFaces[lo]+q.xFaces[hi])/2;
                else if(component==0) speed=(q.yFaces[Id(x-1,y+1,g)]+q.yFaces[Id(x,y+1,g)])/2;
                else if(axis==0) speed=(q.xFaces[Id(x+1,y-1,g)]+q.xFaces[Id(x+1,y,g)])/2;
                else speed=(q.yFaces[lo]+q.yFaces[hi])/2;
                change-=side*h*speed*field[speed>=0?lo:hi]/(axis==0?g.spacingX:g.spacingY);
            }
            out[Id(i,j,g)]+=change;
        }
    }
    return next;
}
}
TEST_CASE("Incompressible uniform states and copied histories", "[incompressible][transaction]") {
    auto c=Config(2,3); c.kinematicViscosity=.3; PeriodicIncompressibleGrid f(c); auto q=f.velocities();
    q.xFaces.assign(6,.7); q.yFaces.assign(6,-.2); f.setVelocities(q);
    const auto d=f.step(.3); Equal(q,f.velocities()); REQUIRE(f.time()==.3); REQUIRE(d.iterations==0);
    REQUIRE(d.initialKineticEnergy==d.finalKineticEnergy); REQUIRE(d.storageEnergyError==0); REQUIRE(d.donorDissipation==0);
    auto copy=f.lastStep(); copy.history[0].timeStep=99; REQUIRE(f.lastStep().history[0].timeStep!=99);
    auto p=f.lastProjection(); p.pressure[0]=7; REQUIRE(f.lastProjection().pressure[0]!=7);
    const auto before=f.lastDiffusion(); const auto projection=f.lastProjection();
    auto z=f.step(0); REQUIRE(z.zeroStepNoOp); REQUIRE(z.substeps==0); REQUIRE(z.cellVisits==18); REQUIRE(f.time()==.3);
    REQUIRE(f.lastProjection().pressure==projection.pressure); REQUIRE(f.lastDiffusion().cellVisits==before.cellVisits); Equal(q,f.velocities());
    f.setVelocities(q); REQUIRE(f.time()==.3); REQUIRE(f.lastStep().zeroStepNoOp);
}
TEST_CASE("Incompressible shear matches independent discrete donor and BE Fourier symbols", "[incompressible][oracle]") {
    for(double mean:{-.7,0.,.7}) {
        auto c=Config(12,2); c.kinematicViscosity=.13; PeriodicIncompressibleGrid f(c); auto q=f.velocities();
        for(std::size_t j=0;j<2;++j) for(std::size_t i=0;i<12;++i) { q.xFaces[i+12*j]=mean; q.yFaces[i+12*j]=std::sin((i+.5)*c.geometry.spacingX); }
        f.setVelocities(q); const auto d=f.step(2.1); std::complex<double> factor=1.;
        const double theta=c.geometry.spacingX,lambda=4*std::pow(std::sin(theta/2)/c.geometry.spacingX,2);
        for(const auto& s:d.history) {
            factor*=(1.-std::abs(mean)*s.timeStep/c.geometry.spacingX*(1.-std::exp(std::complex<double>(0,mean>=0?-theta:theta))))/(1.+c.kinematicViscosity*s.timeStep*lambda);
            REQUIRE(s.outgoingCfl<=.8+1e-15); REQUIRE(s.maximumDualDivergence<1e-12);
            REQUIRE(std::abs(s.advectionStorageError)<=s.roundoffEnergyAllowance);
        }
        auto expected=q; for(std::size_t j=0;j<2;++j) for(std::size_t i=0;i<12;++i) expected.yFaces[i+12*j]=std::imag(factor*std::exp(std::complex<double>(0,(i+.5)*theta)));
        REQUIRE(Error(f.velocities(),expected)<2e-10); REQUIRE(std::abs(d.finalMeanX-d.initialMeanX)<=d.meanRoundoffAllowanceX);
        REQUIRE(std::abs(d.storageEnergyError)<=d.roundoffEnergyAllowance);
        REQUIRE(d.donorDissipation>=0); REQUIRE(d.viscousDissipation>=0); REQUIRE(d.projectionResidualWork==Catch::Approx(0).margin(1e-16));
    }
}
TEST_CASE("Incompressible late failures preserve clock fields and every snapshot", "[incompressible][transaction]") {
    auto c=Config(); c.kinematicViscosity=.2; PeriodicIncompressibleGrid f(c); f.setVelocities(TaylorGreen(c)); f.step(.02);
    const auto q=f.velocities(); const auto d=f.lastStep(); const auto p=f.lastProjection(); const auto v=f.lastDiffusion(); const auto t=f.time();
    PeriodicIncompressibleGrid reference(c); reference.setVelocities(q); const auto successful=reference.step(.02);
    IncompressibleStepConfig o;
    for(int mode=0;mode<5;++mode) {
        o={}; if(mode==0) o.maximumCellVisits=1; if(mode==1) o.maximumCellVisits=successful.cellVisits-c.geometry.columns*c.geometry.rows;
        if(mode==2) o.maximumIterations=0; if(mode==3) o.maximumSubsteps=0;
        if(mode==4) { o.maximumSubsteps=1; o.cflSafety=.001; }
        REQUIRE_THROWS_AS(f.step(.02,o),std::runtime_error); Equal(q,f.velocities()); REQUIRE(f.time()==t);
        REQUIRE(f.lastStep().cellVisits==d.cellVisits); REQUIRE(f.lastStep().history.size()==d.history.size());
        REQUIRE(f.lastProjection().potential==p.potential); REQUIRE(f.lastProjection().pressure==p.pressure); REQUIRE(f.lastDiffusion().cellVisits==v.cellVisits);
    }
    o={}; o.maximumIterations=successful.iterations; o.maximumCellVisits=successful.cellVisits;
    auto accepted=f.step(.02,o); REQUIRE(accepted.iterations<=o.maximumIterations); REQUIRE(accepted.cellVisits==o.maximumCellVisits); Equal(f.velocities(),reference.velocities());
    PeriodicIncompressibleGrid bounded(c); bounded.setVelocities(q); o={}; o.maximumIterations=successful.iterations-1;
    REQUIRE_THROWS_AS(bounded.step(.02,o),std::runtime_error); Equal(q,bounded.velocities()); REQUIRE(bounded.time()==0);
}
TEST_CASE("Incompressible invalid controls and unsupported scales reject transactionally", "[incompressible][range]") {
    for(double bad:{-1.,std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN()}) {
        PeriodicIncompressibleGrid f; REQUIRE_THROWS_AS(f.step(bad),std::invalid_argument);
        auto c=Config(); c.kinematicViscosity=bad; REQUIRE_THROWS_AS(PeriodicIncompressibleGrid(c),std::invalid_argument);
        auto q=f.velocities(); q.xFaces[0]=bad;
        if(!std::isfinite(bad)) REQUIRE_THROWS_AS(f.setVelocities(q),std::invalid_argument);
    }
    PeriodicIncompressibleGrid f(Config()); IncompressibleStepConfig o; o.cflSafety=1; REQUIRE_THROWS_AS(f.step(.1,o),std::invalid_argument);
    o={}; o.maximumCellVisits=IncompressibleStepConfig::MaximumCellVisits+1; REQUIRE_THROWS_AS(f.step(.1,o),std::invalid_argument);
    o={}; o.maximumSubsteps=IncompressibleStepConfig::MaximumSubsteps+1; REQUIRE_THROWS_AS(f.step(.1,o),std::invalid_argument);
    o={}; o.maximumIterations=IncompressibleStepConfig::MaximumIterations+1; REQUIRE_THROWS_AS(f.step(.1,o),std::invalid_argument);
    auto q=f.velocities(); q.xFaces.assign(q.xFaces.size(),1e200); f.setVelocities(q);
    REQUIRE_THROWS_AS(f.step(.1),std::overflow_error); Equal(q,f.velocities()); REQUIRE(f.time()==0);
    q.xFaces.assign(q.xFaces.size(),1e-200); f.setVelocities(q); REQUIRE_THROWS_AS(f.step(.1),std::overflow_error); Equal(q,f.velocities());
}
TEST_CASE("Incompressible anisotropic and two-cell fluxes match independent dense coupled oracle", "[incompressible][oracle]") {
    for(auto dimensions:{std::pair<std::size_t,std::size_t>{2,3},{3,2},{4,3}}) {
        auto c=Config(dimensions.first,dimensions.second); c.geometry.spacingX=.7; c.geometry.spacingY=1.3; c.density=1.7; c.kinematicViscosity=.23;
        PeriodicIncompressibleGrid f(c); auto q=f.velocities();
        for(std::size_t k=0;k<q.xFaces.size();++k) { q.xFaces[k]=.4+std::sin(1.3*k); q.yFaces[k]=-.2+std::cos(.7*k); }
        f.setVelocities(q); auto expected=q; DenseProject(expected,c.geometry);
        expected=IndependentFlux(expected,c.geometry,.03);
        const auto diffusion=DenseOperator(c.geometry,1,.03*c.kinematicViscosity);
        expected.xFaces=DenseSolve(diffusion,expected.xFaces); expected.yFaces=DenseSolve(diffusion,expected.yFaces);
        const auto potential=DenseProject(expected,c.geometry);
        const auto d=f.step(.03); REQUIRE(d.substeps==1); REQUIRE(d.initialProjection.initialDivergenceRms>0);
        REQUIRE(Error(f.velocities(),expected)<2e-10);
        const auto pressure=f.lastProjection().pressure;
        for(std::size_t k=0;k<pressure.size();++k) REQUIRE(pressure[k]==Catch::Approx(c.density*potential[k]/.03).margin(3e-8));
        std::size_t native=d.initialProjection.cellVisits; for(const auto& s:d.history) native+=s.diffusion.cellVisits+s.projection.cellVisits;
        REQUIRE(d.cellVisits==native+(14+17*d.substeps)*q.xFaces.size());
        REQUIRE(std::abs(d.storageEnergyError)<=d.roundoffEnergyAllowance);
        PeriodicIncompressibleGrid replay(c); replay.setVelocities(q); const auto again=replay.step(.03); Equal(replay.velocities(),f.velocities());
        REQUIRE(again.cellVisits==d.cellVisits); REQUIRE(replay.lastProjection().pressure==pressure);
    }
}
TEST_CASE("Incompressible shear temporal and spatial errors are independently refined", "[incompressible][convergence]") {
    auto run=[](std::size_t nx,int steps) {
        auto c=Config(nx,2); c.kinematicViscosity=.2; PeriodicIncompressibleGrid f(c); auto q=f.velocities();
        for(std::size_t j=0;j<2;++j) for(std::size_t i=0;i<nx;++i) { q.xFaces[i+nx*j]=.6; q.yFaces[i+nx*j]=std::sin((i+.5)*c.geometry.spacingX); }
        f.setVelocities(q); const double time=.2;
        for(int k=0;k<steps;++k) f.step(time/steps);
        return std::make_pair(f.velocities(),c);
    };
    double previous=1;
    for(int steps:{4,8,16,32}) {
        const auto result=run(24,steps); const double dx=result.second.geometry.spacingX;
        const std::complex<double> generator=-.6/dx*(1.-std::exp(std::complex<double>(0,-dx)))-.2*4*std::pow(std::sin(dx/2)/dx,2);
        auto exact=result.first;
        for(std::size_t j=0;j<2;++j) for(std::size_t i=0;i<24;++i) exact.yFaces[i+24*j]=std::imag(std::exp(.2*generator+std::complex<double>(0,(i+.5)*dx)));
        const double e=Error(result.first,exact); INFO("fixed spatial semidiscrete error "<<e); REQUIRE(e<previous*.65); previous=e;
    }
    previous=1;
    for(std::size_t nx:{12,24,48}) {
        const auto result=run(nx,200); const double dx=result.second.geometry.spacingX; auto exact=result.first;
        for(std::size_t j=0;j<2;++j) for(std::size_t i=0;i<nx;++i) exact.yFaces[i+nx*j]=std::exp(-.2*.2)*std::sin((i+.5)*dx-.6*.2);
        const double e=Error(result.first,exact); INFO("continuum spatial error "<<e); REQUIRE(e<previous*.7); previous=e;
    }
}
TEST_CASE("Incompressible Taylor-Green point velocity pressure and energy converge spatially", "[incompressible][convergence]") {
    double previousVelocity=1,previousPressure=1,previousEnergy=1;
    for(std::size_t nx:{12,24,48}) {
        auto c=Config(nx,nx); c.density=1.3; c.kinematicViscosity=.1; PeriodicIncompressibleGrid f(c); f.setVelocities(TaylorGreen(c));
        IncompressibleStepConfig o; o.absoluteDivergenceTolerance=1e-12; o.relativeDivergenceTolerance=0;
        for(int k=0;k<20;++k) f.step(.001,o);
        const double t=.02; const double velocityError=Error(f.velocities(),TaylorGreen(c,std::exp(-2*c.kinematicViscosity*t)));
        double pressureError=0; const auto p=f.lastProjection().pressure;
        for(std::size_t j=0;j<nx;++j) for(std::size_t i=0;i<nx;++i) {
            const double exact=c.density/4*std::exp(-4*c.kinematicViscosity*t)*(std::cos(2*(i+.5)*c.geometry.spacingX)+std::cos(2*(j+.5)*c.geometry.spacingY));
            pressureError+=std::pow(p[i+nx*j]-exact,2);
        }
        pressureError=std::sqrt(pressureError/(nx*nx));
        const double energyError=std::abs(f.lastStep().finalKineticEnergy-c.density*pi*pi*std::exp(-4*c.kinematicViscosity*t));
        INFO("N="<<nx<<" velocity="<<velocityError<<" pressure="<<pressureError<<" energy="<<energyError);
        REQUIRE(velocityError<previousVelocity*.75); REQUIRE(pressureError<previousPressure*.75); REQUIRE(energyError<previousEnergy*.75);
        previousVelocity=velocityError; previousPressure=pressureError; previousEnergy=energyError;
    }
    // Deliberately coarse inviscid donor transport loses energy: report and bound it.
    auto c=Config(4,4); PeriodicIncompressibleGrid coarse(c); coarse.setVelocities(TaylorGreen(c)); const auto d=coarse.step(.4);
    REQUIRE(d.donorDissipation>0); REQUIRE(d.viscousDissipation==0); REQUIRE(d.finalKineticEnergy<d.initialKineticEnergy);
}
TEST_CASE("Incompressible Taylor-Green time refinement at fixed grid and converged pressure tolerance", "[incompressible][convergence]") {
    auto c=Config(12,12); c.kinematicViscosity=.1;
    auto run=[&](int steps) {
        PeriodicIncompressibleGrid f(c); f.setVelocities(TaylorGreen(c)); IncompressibleStepConfig o;
        o.absoluteDivergenceTolerance=1e-13; o.relativeDivergenceTolerance=0;
        for(int k=0;k<steps;++k) f.step(.04/steps,o);
        return std::make_pair(f.velocities(),f.lastProjection().pressure);
    };
    const auto reference=run(256); double previousVelocity=1,previousPressure=1;
    for(int steps:{4,8,16,32}) {
        const auto actual=run(steps); const double velocityError=Error(actual.first,reference.first); double pressureError=0;
        for(std::size_t k=0;k<actual.second.size();++k) pressureError+=std::pow(actual.second[k]-reference.second[k],2);
        pressureError=std::sqrt(pressureError/actual.second.size());
        INFO("Fixed-grid numerical time reference: velocity="<<velocityError<<" pressure="<<pressureError);
        REQUIRE(velocityError<previousVelocity*.65); REQUIRE(pressureError<previousPressure*.65);
        previousVelocity=velocityError; previousPressure=pressureError;
    }
}
TEST_CASE("Incompressible finite projection residual drives measured dual divergence work", "[incompressible][residual]") {
    auto c=Config(4,3); c.geometry.spacingX=.7; c.geometry.spacingY=1.3; PeriodicIncompressibleGrid f(c); auto q=f.velocities();
    for(std::size_t k=0;k<q.xFaces.size();++k) { q.xFaces[k]=.7+.2*std::sin(k); q.yFaces[k]=.3*std::cos(.7*k); }
    f.setVelocities(q); IncompressibleStepConfig o; o.absoluteDivergenceTolerance=1; o.relativeDivergenceTolerance=0;
    const auto d=f.step(.02,o); REQUIRE(d.initialProjection.finalDivergenceRms>1e-3);
    REQUIRE(d.history[0].maximumDualDivergence>1e-3); REQUIRE(d.history[0].maximumRowSum>1);
    REQUIRE(std::abs(d.dualDivergenceWork)>1e-6); REQUIRE(std::abs(d.storageEnergyError)<=d.roundoffEnergyAllowance);
    REQUIRE(d.history[0].advectionEnergyChange<=d.history[0].advectionEnergyBound+d.history[0].roundoffEnergyAllowance);
}
TEST_CASE("Incompressible scaling preserves measured energies and clock range is explicit", "[incompressible][range]") {
    double referenceEnergy=0;
    for(double rho:{1e-6,1.,1e6}) {
        auto c=Config(); c.density=rho; c.kinematicViscosity=.2; PeriodicIncompressibleGrid f(c); f.setVelocities(TaylorGreen(c));
        const auto d=f.step(.02); const double normalized=d.finalKineticEnergy/rho;
        if(referenceEnergy==0) referenceEnergy=normalized; else REQUIRE(normalized==Catch::Approx(referenceEnergy).epsilon(1e-12));
        REQUIRE(std::abs(d.storageEnergyError)<=d.roundoffEnergyAllowance);
    }
    auto c=Config(2,2); PeriodicIncompressibleGrid clock(c); clock.step(1e308); const auto previous=clock.lastStep();
    REQUIRE_THROWS_AS(clock.step(1),std::overflow_error); REQUIRE(clock.time()==1e308); REQUIRE(clock.lastStep().cellVisits==previous.cellVisits);
    auto q=clock.velocities(); q.xFaces.assign(4,1e12); q.yFaces.assign(4,-1e12);
    q.xFaces[0]=std::nextafter(1e12,std::numeric_limits<double>::infinity());
    PeriodicIncompressibleGrid difficult(c); difficult.setVelocities(q); IncompressibleStepConfig o;
    o.absoluteDivergenceTolerance=1e-6; o.relativeDivergenceTolerance=0; o.maximumSubsteps=1;
    REQUIRE_THROWS(difficult.step(.01,o)); Equal(q,difficult.velocities()); REQUIRE(difficult.time()==0);
}

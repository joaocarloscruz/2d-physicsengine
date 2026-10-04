#include "catch_amalgamated.hpp"
#include "physics/core/fluids/periodic_mac_grid.h"
#include <algorithm>
#include <cmath>
#include <limits>
using namespace PhysicsEngine;
namespace {
constexpr double Pi=3.1415926535897932384626433832795;
MacVelocityState Mode(const PeriodicMacGridConfig& c,int mx,int my,double u,double v,double phase=.31) {
    MacVelocityState out; out.xFaces.resize(c.columns*c.rows); out.yFaces.resize(c.columns*c.rows);
    for(std::size_t j=0;j<c.rows;++j) for(std::size_t i=0;i<c.columns;++i) {
        const auto k=i+c.columns*j;
        out.xFaces[k]=u*std::sin(2*Pi*(mx*static_cast<double>(i)/c.columns+my*(j+.5)/c.rows)+phase);
        out.yFaces[k]=v*std::sin(2*Pi*(mx*(i+.5)/c.columns+my*static_cast<double>(j)/c.rows)+phase);
    }
    return out;
}
double Difference(const MacVelocityState& a,const MacVelocityState& b) {
    double out=0; for(std::size_t k=0;k<a.xFaces.size();++k)
        out=std::max({out,std::abs(a.xFaces[k]-b.xFaces[k]),std::abs(a.yFaces[k]-b.yFaces[k])});
    return out;
}
MacVelocityState Mixed(const PeriodicMacGridConfig& c) {
    auto a=Mode(c,1,2,.7,-.3),b=Mode(c,3,1,-.2,.8);
    for(std::size_t k=0;k<a.xFaces.size();++k) { a.xFaces[k]+=b.xFaces[k]+.17; a.yFaces[k]+=b.yFaces[k]-.28; }
    return a;
}
void SameSnapshot(const MacProjectionSnapshot& a,const MacProjectionSnapshot& b) {
    REQUIRE(a.potential==b.potential); REQUIRE(a.pressure==b.pressure);
    REQUIRE(a.diagnostics.iterations==b.diagnostics.iterations);
    REQUIRE(a.diagnostics.cellVisits==b.diagnostics.cellVisits);
    REQUIRE(a.diagnostics.finalDivergenceRms==b.diagnostics.finalDivergenceRms);
    REQUIRE(a.diagnostics.finalKineticEnergy==b.diagnostics.finalKineticEnergy);
    REQUIRE(a.diagnostics.density==b.diagnostics.density); REQUIRE(a.diagnostics.timeStep==b.diagnostics.timeStep);
}
}
TEST_CASE("Periodic MAC longitudinal modes use independently derived anisotropic frequency", "[mac][oracle]") {
    const auto shape=GENERATE(std::pair<std::size_t,std::size_t>{12,10},std::pair<std::size_t,std::size_t>{2,5},std::pair<std::size_t,std::size_t>{5,2},std::pair<std::size_t,std::size_t>{2,2});
    PeriodicMacGridConfig c; c.columns=shape.first; c.rows=shape.second; c.spacingX=.23; c.spacingY=.41;
    const int mx=1,my=1; const double ax=2*std::sin(Pi*mx/c.columns)/c.spacingX,ay=2*std::sin(Pi*my/c.rows)/c.spacingY;
    const double eigenvalue=ax*ax+ay*ay;
    PeriodicMacGrid grid(c); auto v=Mode(c,mx,my,ax,ay); grid.setVelocities(v);
    const auto div=grid.divergence();
    for(std::size_t j=0;j<c.rows;++j) for(std::size_t i=0;i<c.columns;++i) {
        const auto k=i+c.columns*j; const double phase=2*Pi*(mx*(i+.5)/c.columns+my*(j+.5)/c.rows)+.31;
        REQUIRE(div[k]==Catch::Approx(eigenvalue*std::cos(phase)).margin(2e-12));
    }
    MacProjectionConfig options; options.density=3; options.timeStep=.2;
    const auto d=grid.project(options); const auto result=grid.lastProjection(); const auto actual=grid.velocities();
    REQUIRE(d.iterations<=2); REQUIRE(d.finalDivergenceRms<=d.targetDivergenceRms);
    for(std::size_t j=0;j<c.rows;++j) for(std::size_t i=0;i<c.columns;++i) {
        const auto k=i+c.columns*j; const double phase=2*Pi*(mx*(i+.5)/c.columns+my*(j+.5)/c.rows)+.31;
        REQUIRE(result.potential[k]==Catch::Approx(-std::cos(phase)).margin(2e-12));
        REQUIRE(result.pressure[k]==Catch::Approx(-15*std::cos(phase)).margin(3e-11));
        REQUIRE(std::abs(actual.xFaces[k])<2e-12); REQUIRE(std::abs(actual.yFaces[k])<2e-12);
    }
}
TEST_CASE("Periodic MAC transverse Fourier modes and mean flow are preserved", "[mac][oracle]") {
    PeriodicMacGridConfig c; c.columns=13; c.rows=11; c.spacingX=.2; c.spacingY=.37;
    const double ax=2*std::sin(2*Pi/c.columns)/c.spacingX,ay=2*std::sin(3*Pi/c.rows)/c.spacingY;
    auto v=Mode(c,2,3,ay,-ax);
    for(std::size_t k=0;k<v.xFaces.size();++k) { v.xFaces[k]+=.71; v.yFaces[k]-=.27; }
    PeriodicMacGrid grid(c); grid.setVelocities(v); const auto d=grid.project();
    REQUIRE(Difference(grid.velocities(),v)<1e-12);
    REQUIRE(d.finalMeanX==Catch::Approx(.71).margin(1e-13)); REQUIRE(d.finalMeanY==Catch::Approx(-.27).margin(1e-13));
    REQUIRE(d.finalDivergenceRms<1e-12);
}
TEST_CASE("Periodic MAC projection energy and orthogonality obey measured residual bounds", "[mac][conservation]") {
    PeriodicMacGridConfig c; c.columns=17; c.rows=12; c.spacingX=.17; c.spacingY=.31;
    PeriodicMacGrid grid(c); grid.setVelocities(Mixed(c));
    const auto tolerance=GENERATE(1e-11,.03);
    MacProjectionConfig options; options.density=7; options.timeStep=.031; options.absoluteDivergenceTolerance=tolerance; options.relativeDivergenceTolerance=0;
    const auto d=grid.project(options); const auto snap=grid.lastProjection(); const auto div=grid.divergence();
    double measured=0,inner=0; for(std::size_t k=0;k<div.size();++k) { measured+=div[k]*div[k]; inner+=div[k]*snap.potential[k]; }
    REQUIRE(d.finalDivergenceRms==Catch::Approx(std::sqrt(measured/div.size())).margin(1e-15));
    REQUIRE(d.divergencePotentialInnerProduct==Catch::Approx(7*c.spacingX*c.spacingY*inner).margin(1e-13));
    REQUIRE(d.finalDivergenceRms<=tolerance);
    REQUIRE(std::abs(d.velocityCorrectionInnerProduct)<=d.residualEnergyBound+d.roundoffEnergyAllowance);
    REQUIRE(std::abs(d.velocityCorrectionInnerProduct+d.divergencePotentialInnerProduct)<d.roundoffEnergyAllowance);
    REQUIRE(std::abs(d.storageEnergyError)<d.roundoffEnergyAllowance);
    REQUIRE(d.finalKineticEnergy<=d.initialKineticEnergy+d.residualEnergyBound+d.roundoffEnergyAllowance);
    REQUIRE(std::abs(d.finalMeanX-d.initialMeanX)<1e-14); REQUIRE(std::abs(d.finalMeanY-d.initialMeanY)<1e-14);
    REQUIRE(std::abs(d.potentialMean)<1e-14); REQUIRE(std::abs(d.pressureMean)<1e-12);
}
TEST_CASE("MAC physical mode divergence and pressure converge at second spatial order", "[mac][refinement]") {
    auto errors=[](std::size_t nx) {
        PeriodicMacGridConfig c; c.columns=nx; c.rows=6; c.spacingX=2./nx; c.spacingY=.7;
        const double k=Pi,frequency=2*std::sin(Pi/nx)/c.spacingX;
        PeriodicMacGrid grid(c); grid.setVelocities(Mode(c,1,0,k,0));
        const auto div=grid.divergence(); double divergenceError=0;
        for(std::size_t i=0;i<nx;++i) divergenceError=std::max(divergenceError,std::abs(div[i]-k*k*std::cos(2*Pi*(i+.5)/nx+.31)));
        grid.project(); const auto p=grid.lastProjection(); double potentialError=0;
        for(std::size_t i=0;i<nx;++i) {
            const double phase=2*Pi*(i+.5)/nx+.31;
            REQUIRE(p.potential[i]==Catch::Approx(-k/frequency*std::cos(phase)).margin(1e-12));
            potentialError=std::max(potentialError,std::abs(p.potential[i]+std::cos(phase)));
        }
        return std::pair<double,double>{divergenceError,potentialError};
    };
    const auto coarse=errors(16),fine=errors(32),finest=errors(64);
    REQUIRE(coarse.first/fine.first>3.9); REQUIRE(fine.first/finest.first>3.9);
    REQUIRE(coarse.second/fine.second>3.9); REQUIRE(fine.second/finest.second>3.9);
}
TEST_CASE("Exact zero MAC divergence preserves velocities and clears previous pressure", "[mac][transaction]") {
    PeriodicMacGridConfig c; c.columns=8; c.rows=6;
    PeriodicMacGrid grid(c); grid.setVelocities(Mixed(c)); grid.project();
    REQUIRE(grid.lastProjection().diagnostics.iterations>0);
    MacVelocityState constant; constant.xFaces.assign(48,.7); constant.yFaces.assign(48,-.2); grid.setVelocities(constant);
    MacProjectionConfig options; options.maximumIterations=0; options.timeStep=.37; options.density=2;
    const auto d=grid.project(options); REQUIRE(d.zeroDivergenceNoOp); REQUIRE(d.iterations==0); REQUIRE(d.cellVisits>0);
    REQUIRE(d.timeStep==.37); REQUIRE(d.density==2);
    REQUIRE(grid.velocities().xFaces==constant.xFaces); REQUIRE(grid.velocities().yFaces==constant.yFaces);
    for(double p:grid.lastProjection().pressure) REQUIRE(p==0);
    for(double p:grid.lastProjection().potential) REQUIRE(p==0);
    options.timeStep=0; const auto before=grid.lastProjection(); REQUIRE_THROWS_AS(grid.project(options),std::invalid_argument); SameSnapshot(grid.lastProjection(),before);
}
TEST_CASE("MAC failed projection and failed setter publish no partial state", "[mac][transaction]") {
    PeriodicMacGridConfig c; c.columns=11; c.rows=9;
    PeriodicMacGrid grid(c); grid.setVelocities(Mixed(c)); grid.project(); grid.setVelocities(Mixed(c));
    const auto v=grid.velocities(); const auto before=grid.lastProjection();
    MacProjectionConfig config; config.maximumIterations=0;
    REQUIRE_THROWS_AS(grid.project(config),std::runtime_error); REQUIRE(Difference(v,grid.velocities())==0); SameSnapshot(before,grid.lastProjection());
    config.maximumIterations=1; REQUIRE_THROWS_AS(grid.project(config),std::runtime_error); REQUIRE(Difference(v,grid.velocities())==0); SameSnapshot(before,grid.lastProjection());
    config.maximumIterations=1000; config.maximumCellVisits=4;
    REQUIRE_THROWS_AS(grid.project(config),std::runtime_error); REQUIRE(Difference(v,grid.velocities())==0); SameSnapshot(before,grid.lastProjection());
    auto bad=v; bad.xFaces[0]=std::numeric_limits<double>::infinity(); REQUIRE_THROWS_AS(grid.setVelocities(bad),std::invalid_argument);
    REQUIRE(Difference(v,grid.velocities())==0); SameSnapshot(before,grid.lastProjection());
    bad=v; bad.xFaces.pop_back(); REQUIRE_THROWS_AS(grid.setVelocities(bad),std::invalid_argument);
    bad=v; bad.xFaces[0]=std::numeric_limits<double>::max(); grid.setVelocities(bad); const auto huge=grid.velocities();
    REQUIRE_THROWS_AS(grid.project(),std::overflow_error); REQUIRE(grid.velocities().xFaces==huge.xFaces); SameSnapshot(before,grid.lastProjection());
}
TEST_CASE("MAC dimensions coefficients counts pressure scale and resource ceilings are validated", "[mac][validation]") {
    PeriodicMacGridConfig c;
    c.columns=1; REQUIRE_THROWS_AS(PeriodicMacGrid(c),std::invalid_argument);
    c.columns=std::numeric_limits<std::size_t>::max(); c.rows=std::numeric_limits<std::size_t>::max(); REQUIRE_THROWS_AS(PeriodicMacGrid(c),std::invalid_argument);
    c={}; c.spacingX=1e-200; REQUIRE_THROWS_AS(PeriodicMacGrid(c),std::invalid_argument);
    c={}; c.spacingX=1e200; REQUIRE_THROWS_AS(PeriodicMacGrid(c),std::invalid_argument);
    c={}; c.spacingY=std::numeric_limits<double>::quiet_NaN(); REQUIRE_THROWS_AS(PeriodicMacGrid(c),std::invalid_argument);
    PeriodicMacGrid grid; MacProjectionConfig options;
    options.density=-1; REQUIRE_THROWS_AS(grid.project(options),std::invalid_argument);
    options={}; options.density=1e300; options.timeStep=1e-300; REQUIRE_THROWS_AS(grid.project(options),std::invalid_argument);
    options={}; options.maximumCellVisits=MacProjectionConfig::MaximumCellVisits+1; REQUIRE_THROWS_AS(grid.project(options),std::invalid_argument);
    options={}; options.absoluteDivergenceTolerance=-1; REQUIRE_THROWS_AS(grid.project(options),std::invalid_argument);
    c={}; c.spacingX=1e-100; c.spacingY=1e-100; PeriodicMacGrid smallCells(c);
    options={}; options.density=1e-200; REQUIRE_THROWS_AS(smallCells.project(options),std::overflow_error);
    auto snapshot=grid.velocities(); snapshot.xFaces[0]=2; REQUIRE(grid.velocities().xFaces[0]==0);
}
TEST_CASE("MAC summation by parts and dyadic streamfunction null mode are independent checks", "[mac][oracle]") {
    PeriodicMacGridConfig c; c.columns=7; c.rows=5; c.spacingX=.25; c.spacingY=.5;
    PeriodicMacGrid grid(c); const auto v=Mixed(c); grid.setVelocities(v); const auto div=grid.divergence();
    std::vector<double> scalar(35);
    for(std::size_t k=0;k<scalar.size();++k) scalar[k]=std::sin(.31*k)+.1*k;
    double divergencePairing=0,gradientPairing=0;
    for(std::size_t j=0;j<c.rows;++j) for(std::size_t i=0;i<c.columns;++i) {
        const auto k=i+c.columns*j;
        divergencePairing+=scalar[k]*div[k];
        gradientPairing+=v.xFaces[k]*(scalar[k]-scalar[(i+c.columns-1)%c.columns+c.columns*j])/c.spacingX
            +v.yFaces[k]*(scalar[k]-scalar[i+c.columns*((j+c.rows-1)%c.rows)])/c.spacingY;
    }
    REQUIRE(std::abs((divergencePairing+gradientPairing)*c.spacingX*c.spacingY)<1e-13);
    MacVelocityState curl; curl.xFaces.resize(35); curl.yFaces.resize(35);
    auto psi=[](std::size_t i,std::size_t j) { return static_cast<double>((3*i+7*j+i*j)%11); };
    for(std::size_t j=0;j<c.rows;++j) for(std::size_t i=0;i<c.columns;++i) {
        const auto k=i+c.columns*j;
        curl.xFaces[k]=(psi(i,(j+1)%c.rows)-psi(i,j))/c.spacingY;
        curl.yFaces[k]=-(psi((i+1)%c.columns,j)-psi(i,j))/c.spacingX;
    }
    grid.setVelocities(curl); for(double x:grid.divergence()) REQUIRE(x==0);
    const auto d=grid.project(); REQUIRE(d.zeroDivergenceNoOp); REQUIRE(Difference(curl,grid.velocities())==0);
}
TEST_CASE("MAC rejects unattainable stored velocity accuracy and pressure overflow transactionally", "[mac][transaction]") {
    PeriodicMacGridConfig c; c.columns=11; c.rows=9; c.spacingX=.17; c.spacingY=.31;
    PeriodicMacGrid grid(c); auto v=Mixed(c);
    for(std::size_t k=0;k<v.xFaces.size();++k) { v.xFaces[k]+=1e12; v.yFaces[k]-=1e12; }
    grid.setVelocities(v); const auto before=grid.lastProjection();
    MacProjectionConfig options; options.maximumIterations=20; options.absoluteDivergenceTolerance=1e-20; options.relativeDivergenceTolerance=0;
    REQUIRE_THROWS_AS(grid.project(options),std::runtime_error); REQUIRE(Difference(v,grid.velocities())==0); SameSnapshot(before,grid.lastProjection());
    c.columns=4; c.rows=4; c.spacingX=1; c.spacingY=1; PeriodicMacGrid pressureGrid(c);
    pressureGrid.setVelocities(Mode(c,1,0,10,0)); const auto pressureBefore=pressureGrid.velocities();
    options={}; options.density=1e308;
    REQUIRE_THROWS_AS(pressureGrid.project(options),std::overflow_error);
    REQUIRE(Difference(pressureBefore,pressureGrid.velocities())==0);
    REQUIRE(pressureGrid.lastProjection().diagnostics.cellVisits==0);
    auto tiny=Mode(c,1,0,1e-200,0); pressureGrid.setVelocities(tiny);
    options={}; options.absoluteDivergenceTolerance=0; options.relativeDivergenceTolerance=0;
    REQUIRE_THROWS_AS(pressureGrid.project(options),std::runtime_error);
    REQUIRE(Difference(tiny,pressureGrid.velocities())==0);
}
TEST_CASE("MAC energy roundoff guard has no absolute physical unit floor", "[mac][conservation]") {
    PeriodicMacGridConfig c; c.columns=11; c.rows=9; PeriodicMacGrid small(c),large(c);
    const auto v=Mixed(c); small.setVelocities(v); large.setVelocities(v);
    MacProjectionConfig low,high; low.density=1e-8; high.density=1e8;
    const auto a=small.project(low),b=large.project(high);
    REQUIRE(a.roundoffEnergyAllowance/a.initialKineticEnergy==Catch::Approx(b.roundoffEnergyAllowance/b.initialKineticEnergy).epsilon(1e-14));
    REQUIRE(b.roundoffEnergyAllowance/a.roundoffEnergyAllowance==Catch::Approx(1e16).epsilon(1e-14));
    PeriodicMacGrid zero(c); REQUIRE(zero.project().roundoffEnergyAllowance==0);
}

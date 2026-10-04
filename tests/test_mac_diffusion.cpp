#include "catch_amalgamated.hpp"
#include "physics/core/fluids/periodic_mac_grid.h"
#include <algorithm>
#include <cmath>
#include <limits>
using namespace PhysicsEngine;
namespace {
constexpr double Pi=3.1415926535897932384626433832795;
MacVelocityState Mode(const PeriodicMacGridConfig& c,int mx,int my,double u,double v,double phase=.31) {
    MacVelocityState state; state.xFaces.resize(c.columns*c.rows); state.yFaces.resize(c.columns*c.rows);
    for(std::size_t j=0;j<c.rows;++j) for(std::size_t i=0;i<c.columns;++i) {
        const auto k=i+c.columns*j;
        state.xFaces[k]=u*std::sin(2*Pi*(mx*double(i)/c.columns+my*(j+.5)/c.rows)+phase);
        state.yFaces[k]=v*std::sin(2*Pi*(mx*(i+.5)/c.columns+my*double(j)/c.rows)+phase);
    }
    return state;
}
double Lambda(const PeriodicMacGridConfig& c,int mx,int my) {
    return 4*std::pow(std::sin(Pi*mx/c.columns)/c.spacingX,2)+4*std::pow(std::sin(Pi*my/c.rows)/c.spacingY,2);
}
void EqualVelocity(const MacVelocityState& a,const MacVelocityState& b) {
    REQUIRE(a.xFaces==b.xFaces); REQUIRE(a.yFaces==b.yFaces);
}
void EqualDiagnostics(const MacDiffusionDiagnostics& a,const MacDiffusionDiagnostics& b) {
    REQUIRE(a.cellVisits==b.cellVisits); REQUIRE(a.iterations==b.iterations);
    REQUIRE(a.finalKineticEnergy==b.finalKineticEnergy); REQUIRE(a.storageEnergyError==b.storageEnergyError);
    REQUIRE(a.timeStep==b.timeStep); REQUIRE(a.kinematicViscosity==b.kinematicViscosity);
    REQUIRE(a.finalResidualRms==b.finalResidualRms); REQUIRE(a.finalMeanX==b.finalMeanX);
}
MacVelocityState Mixed(const PeriodicMacGridConfig& c) {
    auto a=Mode(c,1,2,.7,-.4),b=Mode(c,3,1,-.2,.5);
    for(std::size_t k=0;k<a.xFaces.size();++k) { a.xFaces[k]+=b.xFaces[k]+.17; a.yFaces[k]+=b.yFaces[k]-.23; }
    return a;
}
double MaxDifference(const MacVelocityState& a,const MacVelocityState& b) {
    double error=0; for(std::size_t k=0;k<a.xFaces.size();++k)
        error=std::max({error,std::abs(a.xFaces[k]-b.xFaces[k]),std::abs(a.yFaces[k]-b.yFaces[k])});
    return error;
}
}
TEST_CASE("MAC diffusion discrete Fourier amplification includes anisotropic two-cell axes", "[mac-diffusion][oracle]") {
    for(auto dimensions : {std::pair<std::size_t,std::size_t>{13,11},{2,5},{5,2},{2,2}}) {
        PeriodicMacGridConfig c; c.columns=dimensions.first; c.rows=dimensions.second; c.spacingX=.23; c.spacingY=.41;
        PeriodicMacGrid grid(c); const auto old=Mode(c,1,1,.7,-.4); grid.setVelocities(old);
        MacDiffusionConfig options; options.kinematicViscosity=.17; options.timeStep=.3;
        options.absoluteVelocityTolerance=1e-13; options.relativeVelocityTolerance=0;
        const double amplification=1/(1+options.kinematicViscosity*options.timeStep*Lambda(c,1,1));
        const auto d=grid.diffuse(options); const auto actual=grid.velocities();
        REQUIRE(d.iterations<=4); REQUIRE(d.finalResidualRms<=d.targetResidualRms);
        for(std::size_t k=0;k<actual.xFaces.size();++k) {
            REQUIRE(actual.xFaces[k]==Catch::Approx(old.xFaces[k]*amplification).epsilon(0).margin(3e-14));
            REQUIRE(actual.yFaces[k]==Catch::Approx(old.yFaces[k]*amplification).epsilon(0).margin(3e-14));
        }
        REQUIRE(d.gradientDissipation>0); REQUIRE(d.finalKineticEnergy<d.initialKineticEnergy);
    }
}
TEST_CASE("MAC diffusion mixed modes dissipate energy and preserve means", "[mac-diffusion][conservation]") {
    PeriodicMacGridConfig c; c.columns=17; c.rows=12; c.spacingX=.17; c.spacingY=.31;
    const auto old=Mixed(c); PeriodicMacGrid grid(c); grid.setVelocities(old);
    MacDiffusionConfig options; options.kinematicViscosity=.2; options.timeStep=.031; options.density=7;
    options.absoluteVelocityTolerance=GENERATE(1e-12,.02); options.relativeVelocityTolerance=0;
    const auto d=grid.diffuse(options); const auto actual=grid.velocities();
    double eOld=0,eNew=0,increment=0,gradient=0,work=0,residual2=0;
    for(const auto pair : {std::pair<const std::vector<double>*,const std::vector<double>*>{&old.xFaces,&actual.xFaces},
                          {&old.yFaces,&actual.yFaces}}) {
        const auto& initial=*pair.first; const auto& next=*pair.second;
        for(std::size_t j=0;j<c.rows;++j) for(std::size_t i=0;i<c.columns;++i) {
            const auto k=i+c.columns*j;
            // Independently form the unscaled physical equation stencil.
            const double lap=(next[(i+1)%c.columns+c.columns*j]-2*next[k]+next[(i+c.columns-1)%c.columns+c.columns*j])/(c.spacingX*c.spacingX)
                +(next[i+c.columns*((j+1)%c.rows)]-2*next[k]+next[i+c.columns*((j+c.rows-1)%c.rows)])/(c.spacingY*c.spacingY);
            const double residual=next[k]-options.kinematicViscosity*options.timeStep*lap-initial[k];
            const double dx=(next[(i+1)%c.columns+c.columns*j]-next[k])/c.spacingX;
            const double dy=(next[i+c.columns*((j+1)%c.rows)]-next[k])/c.spacingY;
            eOld+=initial[k]*initial[k]; eNew+=next[k]*next[k]; increment+=std::pow(next[k]-initial[k],2);
            gradient+=dx*dx+dy*dy; work+=next[k]*residual; residual2+=residual*residual;
        }
    }
    const double mass=options.density*c.spacingX*c.spacingY;
    REQUIRE(d.initialKineticEnergy==Catch::Approx(.5*mass*eOld).epsilon(0).margin(2e-13));
    REQUIRE(d.finalKineticEnergy==Catch::Approx(.5*mass*eNew).epsilon(0).margin(2e-13));
    REQUIRE(d.incrementKineticEnergy==Catch::Approx(.5*mass*increment).epsilon(0).margin(1e-13));
    REQUIRE(d.gradientDissipation==Catch::Approx(mass*options.kinematicViscosity*options.timeStep*gradient).epsilon(0).margin(2e-13));
    REQUIRE(d.residualWork==Catch::Approx(mass*work).epsilon(0).margin(2e-13));
    REQUIRE(d.finalResidualRms==Catch::Approx(std::sqrt(residual2/(c.columns*c.rows))).epsilon(0).margin(2e-16));
    REQUIRE(std::abs(d.storageEnergyError)<=d.roundoffEnergyAllowance);
    REQUIRE(std::abs(d.residualWork)<=d.residualEnergyBound+d.roundoffEnergyAllowance);
    REQUIRE(d.finalKineticEnergy<=d.initialKineticEnergy+d.residualEnergyBound+d.roundoffEnergyAllowance);
    REQUIRE(std::abs(d.finalMeanX-d.initialMeanX)<=d.meanRoundoffAllowanceX);
    REQUIRE(std::abs(d.finalMeanY-d.initialMeanY)<=d.meanRoundoffAllowanceY);
    if(options.absoluteVelocityTolerance<1e-10) {
        auto expected=old; const auto a=Mode(c,1,2,.7,-.4),b=Mode(c,3,1,-.2,.5);
        const double fa=1/(1+.2*.031*Lambda(c,1,2)),fb=1/(1+.2*.031*Lambda(c,3,1));
        for(std::size_t k=0;k<expected.xFaces.size();++k) {
            expected.xFaces[k]=fa*a.xFaces[k]+fb*b.xFaces[k]+.17;
            expected.yFaces[k]=fa*a.yFaces[k]+fb*b.yFaces[k]-.23;
        }
        REQUIRE(MaxDifference(actual,expected)<1e-13);
    }
}
TEST_CASE("MAC shear diffusion converges to the continuous exponential", "[mac-diffusion][refinement]") {
    auto error=[](std::size_t ny,int steps,double duration) {
        PeriodicMacGridConfig c; c.columns=2; c.rows=ny; c.spacingX=.3; c.spacingY=2*Pi/ny;
        const auto initial=Mode(c,0,1,1,0); PeriodicMacGrid grid(c); grid.setVelocities(initial);
        MacDiffusionConfig options; options.kinematicViscosity=.2; options.timeStep=duration/steps;
        options.absoluteVelocityTolerance=1e-13; options.relativeVelocityTolerance=0;
        for(int step=0;step<steps;++step) grid.diffuse(options);
        const auto actual=grid.velocities(); const double exact=std::exp(-.2*duration);
        double error=0; for(std::size_t k=0;k<actual.xFaces.size();++k) error=std::max(error,std::abs(actual.xFaces[k]-exact*initial.xFaces[k]));
        for(double div:grid.divergence()) REQUIRE(div==0);
        return error;
    };
    // Separate the two truncation effects: fine spatial grid for temporal
    // refinement; dt proportional to dx^2 for joint spatial/time refinement.
    const double coarse=error(256,4,1),fine=error(256,8,1),finest=error(256,16,1);
    REQUIRE(coarse/fine>1.9); REQUIRE(fine/finest>1.9);
    const double fixedTimeCoarse=error(16,100,.01),fixedTimeFine=error(32,100,.01),fixedTimeFinest=error(64,100,.01);
    REQUIRE(fixedTimeCoarse/fixedTimeFine>3.9); REQUIRE(fixedTimeFine/fixedTimeFinest>3.9);
    const double spatialCoarse=error(16,16,1),spatialFine=error(32,64,1),spatialFinest=error(64,256,1);
    REQUIRE(spatialCoarse/spatialFine>3.8); REQUIRE(spatialFine/spatialFinest>3.8);
}
TEST_CASE("MAC diffusion reduces divergence modes and composes with projection", "[mac-diffusion][composition]") {
    PeriodicMacGridConfig c; c.columns=13; c.rows=11; c.spacingX=.2; c.spacingY=.37;
    PeriodicMacGrid first(c),second(c); auto old=Mixed(c); first.setVelocities(old); second.setVelocities(old);
    MacDiffusionConfig options; options.kinematicViscosity=.3; options.timeStep=.1; options.absoluteVelocityTolerance=1e-12; options.relativeVelocityTolerance=0;
    MacProjectionConfig projection; projection.absoluteDivergenceTolerance=1e-11; projection.relativeDivergenceTolerance=0;
    first.project(projection); const auto previous=first.lastProjection(); first.diffuse(options);
    REQUIRE(first.lastProjection().potential==previous.potential); REQUIRE(first.lastProjection().pressure==previous.pressure);
    const auto diffusion=second.diffuse(options); second.project(projection);
    EqualDiagnostics(diffusion,second.lastDiffusion());
    REQUIRE(MaxDifference(first.velocities(),second.velocities())<1e-12);
    for(double div:first.divergence()) REQUIRE(std::abs(div)<2e-11);
    PeriodicMacGrid longitudinal(c); longitudinal.setVelocities(Mode(c,1,2,.7,-.3));
    const auto before=longitudinal.divergence(); longitudinal.diffuse(options); const auto after=longitudinal.divergence();
    const double factor=1/(1+.3*.1*Lambda(c,1,2));
    for(std::size_t k=0;k<before.size();++k) REQUIRE(after[k]==Catch::Approx(before[k]*factor).epsilon(0).margin(1e-12));
}
TEST_CASE("MAC zero transport and constant fields are exact validated no-ops", "[mac-diffusion][transaction]") {
    PeriodicMacGridConfig c; c.columns=11; c.rows=9; PeriodicMacGrid grid(c); const auto old=Mixed(c); grid.setVelocities(old);
    MacDiffusionConfig options; options.kinematicViscosity=0; options.timeStep=1;
    auto d=grid.diffuse(options); REQUIRE(d.zeroTransportNoOp); REQUIRE(d.iterations==0); REQUIRE(d.finalResidualRms==0);
    EqualVelocity(old,grid.velocities()); REQUIRE(d.initialKineticEnergy==d.finalKineticEnergy);
    options.kinematicViscosity=1; options.timeStep=0; d=grid.diffuse(options); REQUIRE(d.zeroTransportNoOp); EqualVelocity(old,grid.velocities());
    MacVelocityState constant; constant.xFaces.assign(99,.7); constant.yFaces.assign(99,-.2); grid.setVelocities(constant);
    options.timeStep=1; options.maximumIterations=0; d=grid.diffuse(options);
    REQUIRE_FALSE(d.zeroTransportNoOp); REQUIRE(d.iterations==0); REQUIRE(d.gradientDissipation==0); EqualVelocity(constant,grid.velocities());
    auto snapshot=grid.lastDiffusion(); snapshot.finalMeanX=19; REQUIRE(grid.lastDiffusion().finalMeanX!=19);
    PeriodicMacGrid zero(c); d=zero.diffuse(options); REQUIRE(d.roundoffEnergyAllowance==0); REQUIRE(d.initialKineticEnergy==0);
}
TEST_CASE("MAC diffusion failures preserve both fields and earlier diagnostics", "[mac-diffusion][transaction]") {
    PeriodicMacGridConfig c; c.columns=11; c.rows=9; PeriodicMacGrid grid(c); grid.setVelocities(Mixed(c));
    MacDiffusionConfig options; options.kinematicViscosity=.2; options.timeStep=.1;
    grid.diffuse(options); grid.setVelocities(Mixed(c)); const auto old=grid.velocities(); const auto before=grid.lastDiffusion();
    for(std::size_t iterations : {std::size_t(0),std::size_t(1)}) {
        options.maximumIterations=iterations; REQUIRE_THROWS_AS(grid.diffuse(options),std::runtime_error);
        EqualVelocity(old,grid.velocities()); EqualDiagnostics(before,grid.lastDiffusion());
    }
    options.maximumIterations=1000; options.maximumCellVisits=1;
    REQUIRE_THROWS_AS(grid.diffuse(options),std::runtime_error); EqualVelocity(old,grid.velocities()); EqualDiagnostics(before,grid.lastDiffusion());
    options.maximumCellVisits=before.cellVisits-c.columns*c.rows;
    REQUIRE_THROWS_AS(grid.diffuse(options),std::runtime_error); EqualVelocity(old,grid.velocities()); EqualDiagnostics(before,grid.lastDiffusion());
    options.maximumCellVisits=100000000; options.kinematicViscosity=1e308; options.timeStep=1e308;
    REQUIRE_THROWS_AS(grid.diffuse(options),std::overflow_error); EqualVelocity(old,grid.velocities()); EqualDiagnostics(before,grid.lastDiffusion());
    options.kinematicViscosity=.2; options.timeStep=.1; options.density=1e308;
    REQUIRE_THROWS_AS(grid.diffuse(options),std::overflow_error); EqualVelocity(old,grid.velocities()); EqualDiagnostics(before,grid.lastDiffusion());
    options={}; options.timeStep=1; options.kinematicViscosity=.2; options.absoluteVelocityTolerance=0; options.relativeVelocityTolerance=0; options.maximumIterations=20;
    REQUIRE_THROWS_AS(grid.diffuse(options),std::runtime_error); EqualVelocity(old,grid.velocities()); EqualDiagnostics(before,grid.lastDiffusion());
    options={}; options.timeStep=.1; options.kinematicViscosity=.2; options.maximumIterations=1;
    const auto single=Mode(c,1,1,.7,-.4); grid.setVelocities(single);
    REQUIRE_THROWS_AS(grid.diffuse(options),std::runtime_error); EqualVelocity(single,grid.velocities()); EqualDiagnostics(before,grid.lastDiffusion());
}
TEST_CASE("MAC diffusion finite scale diagnostics avoid raw velocity-square overflow and underflow", "[mac-diffusion][range]") {
    PeriodicMacGridConfig c; c.columns=8; c.rows=6;
    double referenceRatio=0;
    for(auto scaleAndDensity : {std::pair<double,double>{1,1},{1e-200,1e200},{1e200,1e-200}}) {
        PeriodicMacGrid grid(c); grid.setVelocities(Mode(c,1,1,scaleAndDensity.first,-.3*scaleAndDensity.first));
        MacDiffusionConfig options; options.kinematicViscosity=.2; options.timeStep=.1; options.density=scaleAndDensity.second;
        options.absoluteVelocityTolerance=0; options.relativeVelocityTolerance=1e-12;
        const auto d=grid.diffuse(options);
        REQUIRE(std::isfinite(d.initialKineticEnergy)); REQUIRE(d.initialKineticEnergy>0);
        REQUIRE(d.finalKineticEnergy<d.initialKineticEnergy); REQUIRE(d.finalResidualRms<=d.targetResidualRms);
        REQUIRE(std::abs(d.storageEnergyError)<=d.roundoffEnergyAllowance);
        const double ratio=d.roundoffEnergyAllowance/d.initialKineticEnergy;
        if(scaleAndDensity.first==1) referenceRatio=ratio;
        else REQUIRE(ratio==Catch::Approx(referenceRatio).epsilon(1e-13));
    }
}
TEST_CASE("MAC diffusion controls and hard ceilings validate before work", "[mac-diffusion][validation]") {
    PeriodicMacGrid grid;
    for(int field=0;field<5;++field) for(double invalid : {-1.,std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN()}) {
        MacDiffusionConfig config;
        if(field==0) config.kinematicViscosity=invalid;
        if(field==1) config.timeStep=invalid;
        if(field==2) config.density=invalid;
        if(field==3) config.absoluteVelocityTolerance=invalid;
        if(field==4) config.relativeVelocityTolerance=invalid;
        REQUIRE_THROWS_AS(grid.diffuse(config),std::invalid_argument);
    }
    MacDiffusionConfig config; config.density=0; REQUIRE_THROWS_AS(grid.diffuse(config),std::invalid_argument);
    config={}; config.maximumIterations=MacDiffusionConfig::MaximumIterations+1; REQUIRE_THROWS_AS(grid.diffuse(config),std::invalid_argument);
    config={}; config.maximumCellVisits=MacDiffusionConfig::MaximumCellVisits+1; REQUIRE_THROWS_AS(grid.diffuse(config),std::invalid_argument);
    config={}; config.maximumCellVisits=0; REQUIRE_THROWS_AS(grid.diffuse(config),std::runtime_error);
}
TEST_CASE("MAC diffusion audits stored one-ulp perturbations without normalized cancellation", "[mac-diffusion][transaction][oracle]") {
    PeriodicMacGridConfig c; c.columns=4; c.rows=4; PeriodicMacGrid grid(c);
    MacVelocityState initial; initial.xFaces.assign(16,1e12); initial.yFaces.assign(16,-1e12);
    initial.xFaces[0]=std::nextafter(initial.xFaces[0],std::numeric_limits<double>::infinity());
    grid.setVelocities(initial); const auto before=grid.lastDiffusion();
    MacDiffusionConfig options; options.kinematicViscosity=.25; options.timeStep=1;
    options.absoluteVelocityTolerance=1e-6; options.relativeVelocityTolerance=0; options.maximumIterations=30;
    bool failed=false;
    try { grid.diffuse(options); } catch(const std::runtime_error&) { failed=true; }
    if(failed) { EqualVelocity(initial,grid.velocities()); EqualDiagnostics(before,grid.lastDiffusion()); }
    else {
        const auto actual=grid.velocities(); double square=0;
        for(auto pair : {std::pair<const std::vector<double>*,const std::vector<double>*>{&actual.xFaces,&initial.xFaces},
                        {&actual.yFaces,&initial.yFaces}}) {
            const auto& next=*pair.first; const auto& old=*pair.second;
            for(std::size_t j=0;j<4;++j) for(std::size_t i=0;i<4;++i) {
                const auto k=i+4*j;
                const double physical=(next[k]-old[k])+.25*((next[k]-next[(i+1)%4+4*j])
                    +(next[k]-next[(i+3)%4+4*j])+(next[k]-next[i+4*((j+1)%4)])+(next[k]-next[i+4*((j+3)%4)]));
                square+=physical*physical;
            }
        }
        REQUIRE(std::sqrt(square/16)<=options.absoluteVelocityTolerance);
    }
}
TEST_CASE("MAC diffusion resolves coefficients when nu times dt alone underflows", "[mac-diffusion][range]") {
    PeriodicMacGridConfig c; c.columns=2; c.rows=2; c.spacingX=1.5e-154; c.spacingY=1.5e-154;
    PeriodicMacGrid grid(c); MacVelocityState initial; initial.xFaces={1,-1,1,-1}; initial.yFaces.assign(4,0); grid.setVelocities(initial);
    MacDiffusionConfig options; options.kinematicViscosity=1e-162; options.timeStep=2e-162; options.density=1e300;
    REQUIRE(options.kinematicViscosity*options.timeStep==0);
    options.absoluteVelocityTolerance=1e-16; options.relativeVelocityTolerance=0;
    const auto d=grid.diffuse(options);
    REQUIRE_FALSE(d.zeroTransportNoOp); REQUIRE(d.iterations>0); REQUIRE(d.gradientDissipation>0);
    REQUIRE(grid.velocities().xFaces[0]<initial.xFaces[0]);
    REQUIRE(d.finalResidualRms<=d.targetResidualRms);
}
TEST_CASE("MAC diffusion rejects lost nonzero stored differences in heterogeneous fields", "[mac-diffusion][range][transaction]") {
    PeriodicMacGridConfig c; c.columns=8; c.rows=6; PeriodicMacGrid grid(c);
    auto initial=grid.velocities(); initial.xFaces[0]=1e200; initial.xFaces[27]=1e-200;
    grid.setVelocities(initial); const auto before=grid.lastDiffusion();
    MacDiffusionConfig options; options.kinematicViscosity=.2; options.timeStep=.1; options.density=1e-200;
    // Loose tolerance reaches the stored audit without requiring iterations.
    // Separate rhs normalization has erased the tiny nonzero entry, which
    // must not turn its lost stored increment into a certified zero residual.
    options.absoluteVelocityTolerance=1e201; options.relativeVelocityTolerance=0;
    REQUIRE_THROWS_AS(grid.diffuse(options),std::overflow_error);
    EqualVelocity(initial,grid.velocities()); EqualDiagnostics(before,grid.lastDiffusion());
}

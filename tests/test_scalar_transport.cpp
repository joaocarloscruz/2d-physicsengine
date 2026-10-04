#include "catch_amalgamated.hpp"
#include "physics/core/fluids/periodic_scalar_transport.h"
#include <algorithm>
#include <cmath>
#include <complex>
#include <limits>
#include <tuple>
using namespace PhysicsEngine;
namespace {
constexpr double Pi=3.1415926535897932384626433832795;
auto Diagnostics(const ScalarTransportDiagnostics& d) {
    return std::make_tuple(d.substeps,d.cellVisits,d.duration,d.timeBefore,d.timeAfter,d.lastSubstep,
        d.maximumOutflowRate,d.outflowRateBound,d.maximumAbsDivergence,d.maximumCfl,
        d.initialIntegratedScalar,d.finalIntegratedScalar,d.initialAbsoluteIntegral,d.finalAbsoluteIntegral,
        d.integratedScalarDrift,d.conservationRoundoffAllowance,d.initialMinimum,d.initialMaximum,
        d.finalMinimum,d.finalMaximum,d.rangeRoundoffAllowance,d.nonnegativeInput,d.discreteDivergenceFree,d.zeroDurationNoOp);
}
MacVelocityState Uniform(const PeriodicScalarGridConfig& c,double u,double v) {
    return {std::vector<double>(c.columns*c.rows,u),std::vector<double>(c.columns*c.rows,v)};
}
// Independent cell-centered donor matrix, not a replay of pair-flux accumulation.
std::vector<double> Matrix(const std::vector<double>& q,const MacVelocityState& v,const PeriodicScalarGridConfig& c,double h) {
    std::vector<double> result(q.size());
    for(std::size_t j=0;j<c.rows;++j) for(std::size_t i=0;i<c.columns;++i) {
        const auto k=i+c.columns*j, l=(i+c.columns-1)%c.columns+c.columns*j, r=(i+1)%c.columns+c.columns*j;
        const auto b=i+c.columns*((j+c.rows-1)%c.rows), t=i+c.columns*((j+1)%c.rows);
        const double out=(std::max(v.xFaces[r],0.0)+std::max(-v.xFaces[k],0.0))/c.spacingX
            +(std::max(v.yFaces[t],0.0)+std::max(-v.yFaces[k],0.0))/c.spacingY;
        result[k]=(1-h*out)*q[k]+h*(std::max(v.xFaces[k],0.0)*q[l]+std::max(-v.xFaces[r],0.0)*q[r])/c.spacingX
            +h*(std::max(v.yFaces[k],0.0)*q[b]+std::max(-v.yFaces[t],0.0)*q[t])/c.spacingY;
    }
    return result;
}
void Near(const std::vector<double>& actual,const std::vector<double>& expected,double tolerance=3e-14) {
    REQUIRE(actual.size()==expected.size());
    for(std::size_t i=0;i<actual.size();++i)
        REQUIRE(actual[i]==Catch::Approx(expected[i]).epsilon(0).margin(tolerance));
}
double Sinc(double x) { return std::sin(x)/x; }
double SpatialError(std::size_t nx) {
    PeriodicScalarGridConfig c{nx,nx/2,1.0/nx,2.0/nx};
    PeriodicScalarTransport grid(c); auto q=grid.state();
    const double averageFactor=Sinc(Pi*c.spacingX)*Sinc(2*Pi*c.spacingY), duration=.4;
    for(std::size_t j=0;j<c.rows;++j) for(std::size_t i=0;i<c.columns;++i)
        q[i+c.columns*j]=1+.3*averageFactor*std::cos(2*Pi*((i+.5)*c.spacingX+2*(j+.5)*c.spacingY));
    grid.setState(q); grid.setVelocities(Uniform(c,.7,-.2));
    ScalarTransportConfig options; options.maxSubstep=duration/nx;
    grid.step(duration,options); const auto actual=grid.state();
    double square=0;
    for(std::size_t j=0;j<c.rows;++j) for(std::size_t i=0;i<c.columns;++i) {
        const double exact=1+.3*averageFactor*std::cos(2*Pi*((i+.5)*c.spacingX+2*(j+.5)*c.spacingY-(.7-2*.2)*duration));
        square+=std::pow(actual[i+c.columns*j]-exact,2);
    }
    return std::sqrt(square/q.size());
}
}

TEST_CASE("Scalar donor matrix matches signed nonuniform anisotropic periodic flow", "[scalar-transport][oracle]") {
    for(const auto dims:{std::pair<std::size_t,std::size_t>{7,5},{2,5},{5,2},{2,2}}) {
        PeriodicScalarGridConfig c{dims.first,dims.second,.23,.41}; PeriodicScalarTransport grid(c);
        auto q=grid.state(); auto velocity=grid.velocities();
        for(std::size_t k=0;k<q.size();++k) {
            q[k]=.2+std::sin(.7*k); velocity.xFaces[k]=.3*std::sin(.31*k+.7); velocity.yFaces[k]=-.2*std::cos(.9*k);
        }
        grid.setState(q); grid.setVelocities(velocity);
        const auto d=grid.step(.02);
        REQUIRE(d.substeps==1); Near(grid.state(),Matrix(q,velocity,c,.02));
        REQUIRE(std::abs(d.integratedScalarDrift)<=d.conservationRoundoffAllowance);
        REQUIRE(d.maximumCfl<.9); REQUIRE(d.lastSubstep<=.1);
    }
}

TEST_CASE("Scalar Fourier amplification follows both signed donor directions and two-cell wraps", "[scalar-transport][oracle]") {
    for(const auto dims:{std::pair<std::size_t,std::size_t>{13,11},{2,5},{5,2},{2,2}})
    for(const double u:{-.3,.3}) for(const double v:{-.2,.2}) {
        PeriodicScalarGridConfig c{dims.first,dims.second,.23,.41}; PeriodicScalarTransport grid(c);
        auto q=grid.state();
        const double tx=2*Pi/c.columns, ty=2*Pi/c.rows, h=.01;
        const std::complex<double> g=1-h*(std::abs(u)/c.spacingX+std::abs(v)/c.spacingY)
            +h*std::abs(u)/c.spacingX*std::exp(std::complex<double>(0,-std::copysign(tx,u)))
            +h*std::abs(v)/c.spacingY*std::exp(std::complex<double>(0,-std::copysign(ty,v)));
        for(std::size_t j=0;j<c.rows;++j) for(std::size_t i=0;i<c.columns;++i)
            q[i+c.columns*j]=2+.4*std::cos(tx*i+ty*j+.31);
        grid.setState(q); grid.setVelocities(Uniform(c,u,v));
        ScalarTransportConfig options; options.maxSubstep=h;
        const auto d=grid.step(.1,options); REQUIRE(d.discreteDivergenceFree); REQUIRE(d.nonnegativeInput);
        const auto amplification=std::pow(g,double(d.substeps)); const auto actual=grid.state();
        for(std::size_t j=0;j<c.rows;++j) for(std::size_t i=0;i<c.columns;++i)
            REQUIRE(actual[i+c.columns*j]==Catch::Approx(2+.4*std::real(amplification*std::exp(std::complex<double>(0,tx*i+ty*j+.31))))
                .epsilon(0).margin(4e-14));
        REQUIRE(std::abs(amplification)<1);
    }
}

TEST_CASE("Scalar two-cell axes retain both distinct face transfers", "[scalar-transport][oracle]") {
    PeriodicScalarGridConfig c{2,2,1,1}; PeriodicScalarTransport grid(c);
    grid.setState({1,2,1,2}); grid.setVelocities({{1,-1,1,-1},{0,0,0,0}});
    const auto d=grid.step(.1);
    Near(grid.state(),{1.4,1.6,1.4,1.6},1e-15);
    REQUIRE_FALSE(d.discreteDivergenceFree); REQUIRE(d.maximumOutflowRate==2);
    REQUIRE(d.initialIntegratedScalar==6); REQUIRE(d.finalIntegratedScalar==Catch::Approx(6).epsilon(0).margin(1e-15));
}

TEST_CASE("Scalar uniform density compresses conservatively instead of being falsely preserved", "[scalar-transport][compression]") {
    PeriodicScalarGridConfig c{4,3,.2,.7}; PeriodicScalarTransport grid(c);
    auto velocity=grid.velocities(); const double faces[]={0,.3,-.2,.1};
    for(std::size_t j=0;j<c.rows;++j) for(std::size_t i=0;i<c.columns;++i) velocity.xFaces[i+c.columns*j]=faces[i];
    const std::vector<double> uniform(12,1); grid.setState(uniform); grid.setVelocities(velocity);
    const auto d=grid.step(.05); Near(grid.state(),Matrix(uniform,velocity,c,.05),2e-15);
    REQUIRE_FALSE(d.discreteDivergenceFree); REQUIRE(d.finalMinimum<1); REQUIRE(d.finalMaximum>1);
    REQUIRE(d.finalMinimum>=0); REQUIRE(d.maximumAbsDivergence==Catch::Approx(2.5).epsilon(0).margin(1e-15));
    REQUIRE(std::abs(d.integratedScalarDrift)<=3e-16);
}

TEST_CASE("Scalar nonnegative pulses retain positivity and incompressible global range", "[scalar-transport][positivity]") {
    PeriodicScalarGridConfig c{32,16,.03,.07}; PeriodicScalarTransport grid(c);
    auto q=grid.state(); auto velocity=grid.velocities();
    for(std::size_t j=0;j<c.rows;++j) for(std::size_t i=0;i<c.columns;++i) {
        const auto k=i+c.columns*j;
        q[k]=(i>=29 || i<3) && j>=6 && j<10 ? 1 : 0;
        velocity.xFaces[k]=.3+.1*std::sin(2*Pi*j/c.rows);
        velocity.yFaces[k]=-.2+.1*std::cos(2*Pi*i/c.columns);
    }
    grid.setState(q); grid.setVelocities(velocity);
    const auto d=grid.step(.6);
    REQUIRE(d.discreteDivergenceFree); REQUIRE(d.nonnegativeInput); REQUIRE(d.maximumCfl<=.9);
    REQUIRE(d.finalMinimum>=0); REQUIRE(d.finalMaximum<1);
    REQUIRE(d.finalIntegratedScalar==Catch::Approx(24*.03*.07).epsilon(0).margin(2e-15));
    for(const double value:grid.state()) { REQUIRE(value>=0); REQUIRE(value<=1+2e-14); }
    const auto constant=std::vector<double>(q.size(),.7); grid.setState(constant); grid.step(.6);
    Near(grid.state(),constant,3e-15);
    grid.setState(std::vector<double>(q.size(),-.7)); const auto negative=grid.step(.6);
    REQUIRE_FALSE(negative.nonnegativeInput); Near(grid.state(),std::vector<double>(q.size(),-.7),3e-15);
}

TEST_CASE("Scalar donor-cell temporal and spatial convergence are first order", "[scalar-transport][convergence]") {
    // The y wavenumber is 2, so use resolved grids to test the asymptotic order;
    // the deliberately coarser headless demonstration reports stronger diffusion.
    const double e32=SpatialError(32), e64=SpatialError(64), e128=SpatialError(128), e256=SpatialError(256);
    INFO(e32<<", "<<e64<<", "<<e128<<", "<<e256);
    REQUIRE(e32/e64>1.6); REQUIRE(e64/e128>1.75); REQUIRE(e128/e256>1.85); REQUIRE(e128/e256<2.1);
    PeriodicScalarGridConfig c{32,2,1.0/32,.5}; const double duration=.4, theta=2*Pi/c.columns;
    const auto exactAmplification=std::exp(duration/c.spacingX*(std::exp(std::complex<double>(0,-theta))-1.0));
    double previous=0;
    for(const std::size_t count:{16,32,64,128}) {
        PeriodicScalarTransport grid(c); auto q=grid.state();
        for(std::size_t k=0;k<q.size();++k) q[k]=1+.3*std::cos(theta*(k%c.columns));
        grid.setState(q); grid.setVelocities(Uniform(c,1,0));
        ScalarTransportConfig options; options.maxSubstep=duration/count;
        grid.step(duration,options); const auto actual=grid.state(); double square=0;
        for(std::size_t k=0;k<q.size();++k) {
            const double exact=1+.3*std::real(exactAmplification*std::exp(std::complex<double>(0,theta*(k%c.columns))));
            square+=std::pow(actual[k]-exact,2);
        }
        const double error=std::sqrt(square/q.size()); INFO(count<<": "<<error);
        if(previous>0) { REQUIRE(previous/error>1.8); REQUIRE(previous/error<2.4); }
        previous=error;
    }
}

TEST_CASE("Scalar snapshots and deterministic replay have independent lifetimes", "[scalar-transport][ownership]") {
    PeriodicScalarGridConfig c{9,7,.2,.3}; PeriodicScalarTransport first(c), second(c);
    auto q=first.state(); for(std::size_t k=0;k<q.size();++k) q[k]=.4+std::sin(.7*k);
    auto velocity=Uniform(c,.3,-.2);
    first.setState(q); second.setState(q); first.setVelocities(velocity); second.setVelocities(velocity);
    q[0]=99; velocity.xFaces[0]=99;
    for(int i=0;i<30;++i) {
        const auto a=first.step(.03), b=second.step(.03);
        REQUIRE(first.state()==second.state()); REQUIRE(first.time()==second.time()); REQUIRE(Diagnostics(a)==Diagnostics(b));
    }
    auto copied=first.state(); auto copiedV=first.velocities(); auto copiedD=first.lastStep();
    copied[0]=99; copiedV.xFaces[0]=99; copiedD.timeAfter=99;
    REQUIRE(first.state()!=copied); REQUIRE(first.velocities().xFaces!=copiedV.xFaces); REQUIRE(first.time()!=99);
    const auto retained=first.lastStep(); first.setState(std::vector<double>(q.size(),0));
    REQUIRE(Diagnostics(first.lastStep())==Diagnostics(retained));
    first.setVelocities(Uniform(c,0,0)); REQUIRE(Diagnostics(first.lastStep())==Diagnostics(retained));
}

TEST_CASE("Scalar zero duration and stationary transport preserve exact stored values", "[scalar-transport][noop]") {
    PeriodicScalarGridConfig c{2,2,1,1}; PeriodicScalarTransport grid(c);
    const std::vector<double> q{.731, -.137, 1.123, 3.331}; grid.setState(q);
    ScalarTransportConfig options; options.maximumSubsteps=0; options.maximumCellVisits=12;
    const auto zero=grid.step(0,options);
    REQUIRE(zero.zeroDurationNoOp); REQUIRE(zero.substeps==0); REQUIRE(zero.cellVisits==12);
    REQUIRE(zero.integratedScalarDrift==0); REQUIRE(zero.timeBefore==0); REQUIRE(zero.timeAfter==0); REQUIRE(grid.state()==q);
    options.maximumSubsteps=7; options.maxSubstep=.03; options.maximumCellVisits=140;
    const auto stationary=grid.step(.21,options);
    REQUIRE(stationary.substeps==7); REQUIRE(stationary.lastSubstep<=.03); REQUIRE(grid.state()==q);
    REQUIRE(stationary.maximumCfl==0); REQUIRE(grid.time()==.21); REQUIRE_FALSE(stationary.zeroDurationNoOp);
}

TEST_CASE("Scalar budgets invalid data and late arithmetic failures roll back every publication", "[scalar-transport][validation]") {
    PeriodicScalarGridConfig c{2,2,1,1}; PeriodicScalarTransport grid(c);
    grid.setState({1,2,3,4}); grid.setVelocities(Uniform(c,.3,-.2));
    const auto success=grid.step(.02); const auto before=grid.state(); const auto velocity=grid.velocities();
    const auto unchanged=[&] { REQUIRE(grid.state()==before); REQUIRE(grid.time()==.02);
        REQUIRE(Diagnostics(grid.lastStep())==Diagnostics(success)); REQUIRE(grid.velocities().xFaces==velocity.xFaces); REQUIRE(grid.velocities().yFaces==velocity.yFaces); };
    for(double bad:{-1.0,std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN()}) {
        REQUIRE_THROWS(grid.step(bad)); unchanged();
        auto q=before; q.back()=bad; if(bad<0) q.back()=std::numeric_limits<double>::infinity();
        REQUIRE_THROWS(grid.setState(q)); unchanged();
    }
    REQUIRE_THROWS(grid.setState({1,2,3})); unchanged();
    auto badVelocity=velocity; badVelocity.yFaces.back()=std::numeric_limits<double>::infinity();
    REQUIRE_THROWS(grid.setVelocities(badVelocity)); unchanged();
    ScalarTransportConfig options;
    for(double bad:{0.0,1.0,-.1,std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN()}) {
        options.cflSafety=bad; REQUIRE_THROWS(grid.step(.02,options)); unchanged();
    }
    options={}; options.maximumSubsteps=0; REQUIRE_THROWS(grid.step(.02,options)); unchanged();
    for(double bad:{0.0,-1.0,std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN()}) {
        options={}; options.maxSubstep=bad; REQUIRE_THROWS(grid.step(.02,options)); unchanged();
    }
    options={}; options.maxSubstep=.01; options.maximumSubsteps=1; REQUIRE_THROWS(grid.step(.02,options)); unchanged();
    options={}; options.maximumSubsteps=1000001; REQUIRE_THROWS(grid.step(0,options)); unchanged();
    options={}; options.maximumCellVisits=1000000001; REQUIRE_THROWS(grid.step(0,options)); unchanged();
    options={}; options.maximumCellVisits=11; REQUIRE_THROWS(grid.step(0,options)); unchanged();
    options={}; options.maximumCellVisits=success.cellVisits-1; REQUIRE_THROWS(grid.step(.02,options)); unchanged();
    options.maximumCellVisits=success.cellVisits;
    PeriodicScalarTransport exact(c); exact.setState({1,2,3,4}); exact.setVelocities(velocity);
    REQUIRE(Diagnostics(exact.step(.02,options))==Diagnostics(success));
    PeriodicScalarGridConfig tiny{2,2,1e-154,1e-154}; PeriodicScalarTransport overflow(tiny);
    overflow.setState(std::vector<double>(4,1e308)); overflow.step(0);
    const auto oldD=overflow.lastStep(); overflow.setVelocities({{1e-154,-1e-154,1e-154,-1e-154},{0,0,0,0}});
    options={}; options.maxSubstep=.4;
    REQUIRE_THROWS(overflow.step(.4,options)); REQUIRE(overflow.time()==0); REQUIRE(overflow.state()==std::vector<double>(4,1e308));
    REQUIRE(Diagnostics(overflow.lastStep())==Diagnostics(oldD));
    for(const auto dimensions:{std::pair<std::size_t,std::size_t>{1,2},{2,1},{262144,2},{std::numeric_limits<std::size_t>::max(),2}}) {
        PeriodicScalarGridConfig invalid{dimensions.first,dimensions.second,1,1}; REQUIRE_THROWS(PeriodicScalarTransport{invalid});
    }
    for(double spacing:{0.0,-1.0,std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN()}) {
        PeriodicScalarGridConfig invalid{2,2,spacing,1}; REQUIRE_THROWS(PeriodicScalarTransport{invalid});
    }
}

TEST_CASE("Scalar scaled arithmetic supports large and tiny units but rejects unresolved ranges", "[scalar-transport][range]") {
    for(const double magnitude:{1e-200,1e200}) {
        PeriodicScalarGridConfig c{2,2,1e100,1e-100}; PeriodicScalarTransport grid(c);
        const std::vector<double> q{magnitude,.5*magnitude,.2*magnitude,.7*magnitude}; grid.setState(q);
        const auto velocity=Uniform(c,.3e100,-.2e-100); grid.setVelocities(velocity);
        const auto d=grid.step(.1); const auto actual=grid.state();
        const std::vector<double> reduced{1,.5,.2,.7}; const auto expected=Matrix(reduced,Uniform({2,2,1,1},.3,-.2),{2,2,1,1},.1);
        for(std::size_t k=0;k<q.size();++k) REQUIRE(actual[k]/magnitude==Catch::Approx(expected[k]).epsilon(0).margin(3e-15));
        REQUIRE(std::abs(d.integratedScalarDrift)<=d.conservationRoundoffAllowance);
    }
    PeriodicScalarTransport range({2,2,1,1}); range.setState({1e308,1e-308,0,0});
    REQUIRE_THROWS(range.step(.01)); REQUIRE(range.time()==0); REQUIRE(range.state()[1]==1e-308);
    PeriodicScalarTransport time({2,2,1,1}); time.setState({1,1,1,1});
    ScalarTransportConfig huge; huge.maxSubstep=1e308;
    time.step(1e308,huge); const auto d=time.lastStep();
    REQUIRE_THROWS(time.step(1,huge)); REQUIRE_THROWS(time.step(1e308,huge)); REQUIRE(time.time()==1e308); REQUIRE(Diagnostics(time.lastStep())==Diagnostics(d));
    const double common=1e200, adjacent=std::nextafter(common,std::numeric_limits<double>::infinity());
    PeriodicScalarTransport faces({2,2,1e100,1e-100}); faces.setState({1,1,1,1});
    faces.setVelocities({{common,adjacent,common,adjacent},{0,0,0,0}});
    ScalarTransportConfig small; small.maxSubstep=1e-103;
    const auto flow=faces.step(1e-103,small);
    REQUIRE_FALSE(flow.discreteDivergenceFree);
    REQUIRE(flow.maximumAbsDivergence==Catch::Approx((adjacent-common)/1e100).epsilon(0).margin(1e68));
}

TEST_CASE("Scalar strict CFL endpoint and compensated signed totals remain physical", "[scalar-transport][numerics]") {
    PeriodicScalarGridConfig c{2,2,1,1}; PeriodicScalarTransport grid(c);
    grid.setState({1,0,0,0}); grid.setVelocities(Uniform(c,.7,.3));
    const auto audit=grid.step(0);
    ScalarTransportConfig options; options.cflSafety=std::nextafter(1.0,0.0); options.maxSubstep=2;
    const double h=std::nextafter(options.cflSafety/audit.outflowRateBound,0.0);
    const auto d=grid.step(h,options);
    REQUIRE(d.substeps==1); REQUIRE(d.lastSubstep==h); REQUIRE(d.maximumCfl<=options.cflSafety);
    REQUIRE(d.finalMinimum>=0); REQUIRE(d.finalMaximum<=1);
    double stored=0; for(const double value:grid.state()) stored+=value;
    REQUIRE(d.finalIntegratedScalar==Catch::Approx(stored).epsilon(0).margin(3e-16));
    PeriodicScalarTransport signedTotal(c); signedTotal.setState({1e200,1,-1e200,1});
    const auto total=signedTotal.step(0);
    REQUIRE(total.initialIntegratedScalar==Catch::Approx(2).epsilon(0).margin(8e-16));
    REQUIRE(total.finalIntegratedScalar==total.initialIntegratedScalar);
    PeriodicScalarTransport cancel({2,2,1e-154,1e-154});
    cancel.setState({-1e308,1e308,-1e308,1e308});
    cancel.setVelocities(Uniform({2,2,1e-154,1e-154},1e-154,0));
    options.cflSafety=.95; options.maxSubstep=.9; const auto large=cancel.step(.9,options);
    const auto actual=cancel.state();
    for(std::size_t k=0;k<4;++k) REQUIRE(actual[k]/1e308==Catch::Approx(k%2==0 ? .8 : -.8).epsilon(0).margin(2e-15));
    REQUIRE(large.initialIntegratedScalar==0); REQUIRE(large.finalIntegratedScalar==0);
    REQUIRE(large.discreteDivergenceFree); REQUIRE_FALSE(large.nonnegativeInput);
    PeriodicScalarTransport largest({512,512,1,1}); REQUIRE(largest.state().size()==262144);
    for(const auto invalid:{PeriodicScalarGridConfig{2,2,1e308,1},PeriodicScalarGridConfig{2,2,1e-200,1e-200}})
        REQUIRE_THROWS(PeriodicScalarTransport{invalid});
    PeriodicScalarTransport difference({2,2,1e307,1e-307}); difference.setState({1,1,1,1});
    difference.setVelocities({{1e308,-1e308,1e308,-1e308},{0,0,0,0}});
    const auto opposite=difference.step(.01);
    REQUIRE(opposite.maximumAbsDivergence==Catch::Approx(20).epsilon(0).margin(8e-15));
    Near(difference.state(),{1.2,.8,1.2,.8},2e-15);
    PeriodicScalarTransport unresolved(c); unresolved.setState({1e308,0,0,0});
    unresolved.setVelocities(Uniform(c,1e-308,0));
    REQUIRE_THROWS(unresolved.step(1e-20)); REQUIRE(unresolved.state()==std::vector<double>{1e308,0,0,0}); REQUIRE(unresolved.time()==0);
}

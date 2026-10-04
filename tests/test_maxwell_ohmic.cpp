#include "catch_amalgamated.hpp"
#include "physics/core/maxwell_grid.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <utility>
using namespace PhysicsEngine;
namespace {
constexpr double Pi = 3.14159265358979323846;
MaxwellFieldState Mode(const MaxwellGridConfig &c, int mx, int my, double e, double b,
                       bool continuum = false) {
    auto s = MaxwellGrid(c).getState();
    const double x = continuum ? 2 * Pi * mx / (c.columns * c.spacingX)
                               : 2 * std::sin(Pi * mx / c.columns) / c.spacingX;
    const double y = continuum ? 2 * Pi * my / (c.rows * c.spacingY)
                               : 2 * std::sin(Pi * my / c.rows) / c.spacingY;
    for (std::size_t j = 0; j < c.rows; ++j)
        for (std::size_t i = 0; i < c.columns; ++i) {
            const auto k = i + c.columns * j;
            const double p = 2 * Pi * (double(i) * mx / c.columns + double(j) * my / c.rows) + .31;
            s.ez[k] = e * std::cos(p);
            s.hx[k] = y * b * std::sin(p + Pi * my / c.rows);
            s.hy[k] = -x * b * std::sin(p + Pi * mx / c.columns);
        }
    return s;
}
std::pair<double, double> Exact(double omega, double a, double mu, double t, int regime) {
    const double half = .5 * a, decay = std::exp(-half * t);
    if (regime == 1)
        return {decay * (1 - half * t), decay * t / mu};
    const double frequency = std::sqrt(std::abs(omega * omega - half * half));
    const double sinc =
        regime == 0 ? std::sin(frequency * t) / frequency : std::sinh(frequency * t) / frequency;
    const double cosine = regime == 0 ? std::cos(frequency * t) : std::cosh(frequency * t);
    return {decay * (cosine - half * sinc), decay * sinc / mu};
}
double Difference(const MaxwellFieldState &a, const MaxwellFieldState &b) {
    double error = 0;
    for (std::size_t i = 0; i < a.ez.size(); ++i)
        error = std::max({error, std::abs(a.ez[i] - b.ez[i]), std::abs(a.hx[i] - b.hx[i]),
                          std::abs(a.hy[i] - b.hy[i])});
    return error;
}
void Same(const MaxwellFieldState &a, const MaxwellFieldState &b) {
    REQUIRE(a.ez == b.ez);
    REQUIRE(a.hx == b.hx);
    REQUIRE(a.hy == b.hy);
}
void Same(const MaxwellGridDiagnostics &a, const MaxwellGridDiagnostics &b) {
    const double x[] = {a.electricEnergy,
                        a.magneticEnergy,
                        a.totalEnergy,
                        a.modifiedEnergy,
                        a.modifiedEnergyStep,
                        a.meanEz,
                        a.meanHx,
                        a.meanHy,
                        a.maxAbsEz,
                        a.maxAbsHx,
                        a.maxAbsHy,
                        a.magneticDivergenceRms,
                        a.maxAbsMagneticDivergence,
                        a.time,
                        a.stableTimeStep,
                        a.lastSubstep};
    const double y[] = {b.electricEnergy,
                        b.magneticEnergy,
                        b.totalEnergy,
                        b.modifiedEnergy,
                        b.modifiedEnergyStep,
                        b.meanEz,
                        b.meanHx,
                        b.meanHy,
                        b.maxAbsEz,
                        b.maxAbsHx,
                        b.maxAbsHy,
                        b.magneticDivergenceRms,
                        b.maxAbsMagneticDivergence,
                        b.time,
                        b.stableTimeStep,
                        b.lastSubstep};
    for (std::size_t k = 0; k < 16; ++k)
        REQUIRE(x[k] == y[k]);
    REQUIRE(a.lastSubsteps == b.lastSubsteps);
    REQUIRE(a.lastCellVisits == b.lastCellVisits);
}
struct Energy {
    double electric = 0, magnetic = 0, correction = 0;
};
Energy Quadrature(const MaxwellGridConfig &c, const MaxwellFieldState &s, double h) {
    Energy e;
    const double area = c.spacingX * c.spacingY;
    for (std::size_t j = 0; j < c.rows; ++j)
        for (std::size_t i = 0; i < c.columns; ++i) {
            const auto k = i + c.columns * j;
            e.electric += area * c.permittivity * .5 * s.ez[k] * s.ez[k];
            e.magnetic += area * c.permeability * .5 * (s.hx[k] * s.hx[k] + s.hy[k] * s.hy[k]);
            const double x = (s.ez[(i + 1) % c.columns + c.columns * j] - s.ez[k]) / c.spacingX;
            const double y = (s.ez[i + c.columns * ((j + 1) % c.rows)] - s.ez[k]) / c.spacingY;
            e.correction += area * h * h / (8 * c.permeability) * (x * x + y * y);
        }
    return e;
}
double Roundoff(double scale, std::size_t cells, std::size_t steps = 1) {
    const double operations = 128. * cells * (steps + 1),
                 u = std::numeric_limits<double>::epsilon();
    return scale * (operations * u / (1 - operations * u));
}
} // namespace
TEST_CASE("Ohmic uniform electric exponential and physical Joule ledger are independent",
          "[maxwell][ohmic]") {
    MaxwellGridConfig c;
    c.columns = 3;
    c.rows = 2;
    c.spacingX = .2;
    c.spacingY = .4;
    c.permittivity = 2;
    c.permeability = 3;
    MaxwellGrid g(c);
    auto s = g.getState();
    s.ez.assign(6, 2);
    s.hx.assign(6, 3);
    s.hy.assign(6, -4);
    g.setState(s);
    constexpr double sigma = 1.7, T = .3;
    const double U = c.spacingX * c.spacingY * c.permittivity * .5 * 6 * 4;
    const auto r = g.stepOhmic(T, sigma);
    const auto final = g.getState();
    const double expected = 2 * std::exp(-sigma * T / c.permittivity);
    for (double e : final.ez)
        REQUIRE(e == Catch::Approx(expected).epsilon(0).margin(8e-15));
    REQUIRE(final.hx == s.hx);
    REQUIRE(final.hy == s.hy);
    const double joule = U * (-std::expm1(-2 * sigma * T / c.permittivity));
    REQUIRE(r.exactJouleEnergy ==
            Catch::Approx(joule).epsilon(0).margin(Roundoff(joule, 6, r.substeps)));
    REQUIRE(r.representedElectricEnergyLoss ==
            Catch::Approx(joule).epsilon(0).margin(Roundoff(U, 6, r.substeps)));
    REQUIRE(r.modifiedEnergyDissipation == r.exactJouleEnergy);
    REQUIRE(r.wavePhysicalEnergyChange == 0);
    REQUIRE(std::abs(r.physicalBalanceResidual) <=
            Roundoff(r.initialPhysicalEnergy, 6, r.substeps));
    REQUIRE(r.cellVisits == 6 * (8 * r.substeps + 1));
    REQUIRE(g.getDiagnostics().lastCellVisits == r.cellVisits);
    REQUIRE(r.startTime == 0);
    REQUIRE(r.endTime == T);
    REQUIRE(r.conductivity == sigma);
}
TEST_CASE("Ohmic split converges to independent under critical and overdamped semidiscrete modes",
          "[maxwell][ohmic][oracle]") {
    const int regime = GENERATE(0, 1, 2);
    const auto shape = GENERATE(
        std::pair<std::size_t, std::size_t>{12, 10}, std::pair<std::size_t, std::size_t>{2, 5},
        std::pair<std::size_t, std::size_t>{5, 2}, std::pair<std::size_t, std::size_t>{2, 2});
    MaxwellGridConfig c;
    c.columns = shape.first;
    c.rows = shape.second;
    c.spacingX = .23;
    c.spacingY = .41;
    c.permittivity = 1.7;
    c.permeability = .8;
    c.maxSubstep = 1;
    const int mx = c.columns == 2 ? 1 : 2, my = c.rows == 2 ? 1 : 2;
    const double x = 2 * std::sin(Pi * mx / c.columns) / c.spacingX,
                 y = 2 * std::sin(Pi * my / c.rows) / c.spacingY;
    const double omega = std::hypot(x, y) / std::sqrt(c.permittivity * c.permeability),
                 a = (regime == 0   ? .6
                      : regime == 1 ? 2.
                                    : 3.) *
                     omega;
    constexpr double T = .17;
    const auto exact = Exact(omega, a, c.permeability, T, regime);
    const auto target = Mode(c, mx, my, exact.first, exact.second);
    const auto initialEnergy = Quadrature(c, Mode(c, mx, my, 1, 0), 0);
    const auto exactEnergy = Quadrature(c, target, 0);
    const double exactJoule = initialEnergy.electric + initialEnergy.magnetic -
                              exactEnergy.electric - exactEnergy.magnetic;
    double previous = 0, previousJoule = 0;
    for (int n : {20, 40, 80}) {
        MaxwellGrid g(c);
        g.setState(Mode(c, mx, my, 1, 0));
        const double h = T / n;
        double totalJoule = 0;
        for (int k = 0; k < n; ++k)
            totalJoule += g.stepOhmic(h, a * c.permittivity).exactJouleEnergy;
        const double error = Difference(g.getState(), target);
        INFO("regime=" << regime << " n=" << n << " error=" << error << " previous=" << previous);
        if (previous > 0) {
            REQUIRE(error / previous > .18);
            REQUIRE(error / previous < .31);
        }
        const double jouleError = std::abs(totalJoule - exactJoule);
        INFO("Joule error=" << jouleError << " previous=" << previousJoule);
        if (previousJoule > 0) {
            REQUIRE(jouleError / previousJoule > .18);
            REQUIRE(jouleError / previousJoule < .31);
        }
        previousJoule = jouleError;
        previous = error;
        REQUIRE(g.getDiagnostics().maxAbsMagneticDivergence < 3e-13);
    }
}
TEST_CASE("Ohmic continuum mode refinement is second order in anisotropic axes and oblique grids",
          "[maxwell][ohmic][convergence]") {
    const auto direction =
        GENERATE(std::pair<int, int>{1, 0}, std::pair<int, int>{0, 1}, std::pair<int, int>{1, 2});
    double previous = 0;
    for (int n : {16, 32, 64}) {
        MaxwellGridConfig c;
        c.columns = c.rows = n;
        c.spacingX = 2. / n;
        c.spacingY = 3. / n;
        c.permittivity = 1.7;
        c.permeability = .8;
        c.maxSubstep = 1;
        const double x = Pi * direction.first, y = 2 * Pi * direction.second / 3,
                     omega = std::hypot(x, y) / std::sqrt(c.permittivity * c.permeability);
        constexpr double T = .3, sigma = 1.3;
        const auto exact = Exact(omega, sigma / c.permittivity, c.permeability, T, 0);
        MaxwellGrid g(c);
        g.setState(Mode(c, direction.first, direction.second, 1, 0, true));
        const int count = int(std::ceil(T / (.2 * g.getStableTimeStep())));
        for (int k = 0; k < count; ++k)
            g.stepOhmic(T / count, sigma);
        const double error = Difference(g.getState(), Mode(c, direction.first, direction.second,
                                                           exact.first, exact.second, true));
        INFO("n=" << n << " error=" << error << " previous=" << previous);
        if (previous > 0) {
            REQUIRE(error / previous > .18);
            REQUIRE(error / previous < .31);
        }
        previous = error;
    }
}
TEST_CASE(
    "Ohmic Joule wave defect and modified contraction agree with independent stage quadrature",
    "[maxwell][ohmic][energy]") {
    MaxwellGridConfig c;
    c.columns = 9;
    c.rows = 7;
    c.spacingX = .2;
    c.spacingY = .35;
    c.permittivity = 2;
    c.permeability = 3;
    c.maxSubstep = 1;
    MaxwellGrid g(c);
    const int mx = 2, my = 3;
    const double h = .7 * g.getStableTimeStep(), sigma = .8;
    auto initial = Mode(c, mx, my, 1, .03);
    g.setState(initial);
    const double x = 2 * std::sin(Pi * mx / c.columns) / c.spacingX,
                 y = 2 * std::sin(Pi * my / c.rows) / c.spacingY;
    const double z2 = h * h * (x * x + y * y) / (c.permittivity * c.permeability), d = 1 - z2 / 2,
                 r = std::exp(-sigma * h / (2 * c.permittivity));
    const double eWave = d * r - h * (x * x + y * y) * .03 / c.permittivity,
                 bWave = d * .03 + h * (1 - z2 / 4) * r / c.permeability;
    const auto q0 = Quadrature(c, initial, h), q1 = Quadrature(c, Mode(c, mx, my, r, .03), h),
               qw = Quadrature(c, Mode(c, mx, my, eWave, bWave), h);
    const auto q2 = Quadrature(c, Mode(c, mx, my, r * eWave, bWave), h);
    const double f = -std::expm1(-sigma * h / c.permittivity),
                 joule = f * (q0.electric + qw.electric);
    const double modified = f * (q0.electric - q0.correction + qw.electric - qw.correction);
    const double wave = (qw.electric + qw.magnetic) - (q1.electric + q1.magnetic);
    const double reference = g.getModifiedEnergy(h);
    const auto ledger = g.stepOhmic(h, sigma);
    const double tol = Roundoff(q0.electric + q0.magnetic, 63);
    REQUIRE(Difference(g.getState(), Mode(c, mx, my, r * eWave, bWave)) < 2e-15);
    REQUIRE(ledger.exactJouleEnergy == Catch::Approx(joule).epsilon(0).margin(tol));
    REQUIRE(ledger.modifiedEnergyDissipation == Catch::Approx(modified).epsilon(0).margin(tol));
    REQUIRE(ledger.wavePhysicalEnergyChange == Catch::Approx(wave).epsilon(0).margin(tol));
    REQUIRE(ledger.representedElectricEnergyLoss ==
            Catch::Approx(q0.electric - q1.electric + qw.electric - q2.electric)
                .epsilon(0)
                .margin(tol));
    REQUIRE(ledger.exactJouleEnergy > ledger.modifiedEnergyDissipation);
    REQUIRE(std::abs(ledger.wavePhysicalEnergyChange) > 1e-6 * (q0.electric + q0.magnetic));
    REQUIRE(g.getModifiedEnergy(h) + ledger.modifiedEnergyDissipation ==
            Catch::Approx(reference).epsilon(0).margin(tol));
    const auto div = g.getMagneticDivergence();
    const auto before = g.getDiagnostics();
    double previous = g.getModifiedEnergy(h);
    for (int k = 0; k < 200; ++k) {
        const auto report = g.stepOhmic(h, sigma);
        const double now = g.getModifiedEnergy(h);
        REQUIRE(now <= previous + Roundoff(previous, 63));
        REQUIRE(now + report.modifiedEnergyDissipation ==
                Catch::Approx(previous).epsilon(0).margin(Roundoff(previous, 63)));
        REQUIRE(std::abs(report.physicalBalanceResidual) <=
                Roundoff(report.initialPhysicalEnergy, 63));
        previous = now;
    }
    const auto final = g.getDiagnostics();
    const auto finalDiv = g.getMagneticDivergence();
    REQUIRE(final.meanHx == Catch::Approx(before.meanHx).epsilon(0).margin(2e-15));
    REQUIRE(final.meanHy == Catch::Approx(before.meanHy).epsilon(0).margin(2e-15));
    for (std::size_t k = 0; k < div.size(); ++k)
        REQUIRE(finalDiv[k] == Catch::Approx(div[k]).epsilon(0).margin(2e-14));
}
TEST_CASE("Ohmic zero path keeps lossless state diagnostics and budgets exactly",
          "[maxwell][ohmic][compatibility]") {
    MaxwellGridConfig c;
    c.columns = c.rows = 2;
    c.maximumCellVisits = 16;
    MaxwellGrid old(c), ohmic(c);
    auto s = Mode(c, 1, 1, .7, .1);
    old.setState(s);
    ohmic.setState(s);
    for (double dt : {.01, 0., .04, .08}) {
        old.step(dt);
        const auto report = ohmic.stepOhmic(dt, 0);
        Same(old.getState(), ohmic.getState());
        Same(old.getDiagnostics(), ohmic.getDiagnostics());
        REQUIRE(report.exactJouleEnergy == 0);
        REQUIRE(report.modifiedEnergyDissipation == 0);
        REQUIRE(report.cellVisits == (dt == 0 ? 0 : old.getDiagnostics().lastCellVisits));
    }
    const auto state = ohmic.getState();
    const auto diagnostics = ohmic.getDiagnostics();
    const auto zero = ohmic.stepOhmic(0, 3);
    Same(ohmic.getState(), state);
    Same(ohmic.getDiagnostics(), diagnostics);
    REQUIRE(zero.cellVisits == 0);
    REQUIRE_THROWS_AS(ohmic.stepOhmic(.01, 1), std::length_error);
    Same(ohmic.getState(), state);
    Same(ohmic.getDiagnostics(), diagnostics);
}
TEST_CASE("Ohmic tiny chi reports exact subflow work rather than hiding rounded stored loss",
          "[maxwell][ohmic][range]") {
    MaxwellGridConfig c;
    c.columns = c.rows = 2;
    MaxwellGrid g(c);
    auto s = g.getState();
    s.ez.assign(4, 1);
    g.setState(s);
    const auto r = g.stepOhmic(.1, 1e-18);
    Same(g.getState(), s);
    REQUIRE(r.exactJouleEnergy > 0);
    REQUIRE(r.representedElectricEnergyLoss == 0);
    REQUIRE(r.exactJouleEnergy == Catch::Approx(4e-19).epsilon(8e-16));
    REQUIRE(r.decayStorageEnergyChange == r.exactJouleEnergy);
    REQUIRE(r.physicalBalanceResidual == r.exactJouleEnergy);
}
TEST_CASE("Ohmic exponent staging supports finite complete products despite overflowing rate",
          "[maxwell][ohmic][range]") {
    MaxwellGridConfig c;
    c.columns = c.rows = 2;
    c.permittivity = 1e-308;
    MaxwellGrid g(c);
    auto s = g.getState();
    s.ez.assign(4, 1e154);
    g.setState(s);
    const double dt = 1e-310, sigma = 100;
    REQUIRE_FALSE(std::isfinite(sigma / c.permittivity));
    const double exponent = (sigma * dt) / c.permittivity, expected = 1e154 * std::exp(-exponent);
    const auto r = g.stepOhmic(dt, sigma);
    for (double e : g.getState().ez)
        REQUIRE(e == Catch::Approx(expected).epsilon(3e-15));
    REQUIRE(r.exactJouleEnergy > 0);
    REQUIRE(std::isfinite(r.physicalBalanceResidual));
}
TEST_CASE("Ohmic work rate clock and late energy failures roll back all published state",
          "[maxwell][ohmic][validation]") {
    MaxwellGridConfig c;
    c.columns = c.rows = 2;
    c.maximumCellVisits = 36;
    c.maximumSubsteps = 1;
    MaxwellGrid g(c);
    auto s = g.getState();
    s.ez.assign(4, 1);
    g.setState(s);
    const auto state = g.getState();
    const auto d = g.getDiagnostics();
    for (double bad :
         {-1., std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()}) {
        REQUIRE_THROWS(g.stepOhmic(0, bad));
        Same(g.getState(), state);
        Same(g.getDiagnostics(), d);
        REQUIRE_THROWS(g.stepOhmic(bad, 1));
        Same(g.getState(), state);
        Same(g.getDiagnostics(), d);
    }
    for (double sigma : {std::numeric_limits<double>::denorm_min(), 1e7}) {
        REQUIRE_THROWS_AS(g.stepOhmic(.1, sigma), std::overflow_error);
        Same(g.getState(), state);
        Same(g.getDiagnostics(), d);
    }
    REQUIRE_THROWS_AS(g.stepOhmic(.2, 1), std::length_error);
    Same(g.getState(), state);
    Same(g.getDiagnostics(), d);
    const auto report = g.stepOhmic(.1, 1);
    REQUIRE(report.cellVisits == 36);
    MaxwellGridConfig lateConfig;
    lateConfig.columns = lateConfig.rows = 2;
    lateConfig.maxSubstep = 1;
    MaxwellGrid late(lateConfig);
    auto large =
        Mode(lateConfig, 1, 1, 0, 3e153); // H components are +/-6e153 times staggered sine.
    late.setState(large);
    const auto lateState = late.getState();
    const auto lateD = late.getDiagnostics();
    REQUIRE_THROWS_AS(late.stepOhmic(.6, .1), std::overflow_error);
    Same(late.getState(), lateState);
    Same(late.getDiagnostics(), lateD);
    MaxwellGridConfig clockConfig;
    clockConfig.columns = clockConfig.rows = 2;
    clockConfig.permittivity = clockConfig.permeability = 1e40;
    clockConfig.maxSubstep = 1e14;
    MaxwellGrid clock(clockConfig);
    clock.step(1e14);
    const auto clockState = clock.getState();
    const auto clockD = clock.getDiagnostics();
    REQUIRE_THROWS_AS(clock.stepOhmic(1e-10, 1), std::overflow_error);
    Same(clock.getState(), clockState);
    Same(clock.getDiagnostics(), clockD);
    MaxwellGridConfig tinyConfig;
    tinyConfig.columns = tinyConfig.rows = 2;
    tinyConfig.permittivity = 1e200;
    MaxwellGrid tiny(tinyConfig);
    auto tinyState = tiny.getState();
    tinyState.ez.assign(4, 1e-100);
    tiny.setState(tinyState);
    const auto tinyD = tiny.getDiagnostics();
    REQUIRE_THROWS_AS(tiny.stepOhmic(.1, 1.2e204), std::overflow_error);
    Same(tiny.getState(), tinyState);
    Same(tiny.getDiagnostics(), tinyD);
}

TEST_CASE("Ohmic large decay keeps a finite field and native replay owns its ledger",
          "[maxwell][ohmic][ownership]") {
    MaxwellGridConfig c;
    c.columns = c.rows = 2;
    MaxwellGrid dc(c);
    auto s = dc.getState();
    s.ez.assign(4, 1);
    s.hx.assign(4, 1);
    dc.setState(s);
    const auto strong = dc.stepOhmic(.1, 1000);
    for (double e : dc.getState().ez) {
        REQUIRE(e > 0);
        REQUIRE(e == Catch::Approx(std::exp(-100)).epsilon(4e-15));
    }
    REQUIRE(dc.getState().hx == s.hx);
    REQUIRE(strong.exactJouleEnergy == Catch::Approx(2).epsilon(4e-15));
    REQUIRE(strong.modifiedEnergyDissipation == strong.exactJouleEnergy);
    MaxwellGrid a(c), b(c);
    s = Mode(c, 1, 1, .7, .1);
    a.setState(s);
    b.setState(s);
    for (double dt : {.021, 0., .137, .004}) {
        auto ra = a.stepOhmic(dt, .7);
        const auto rb = b.stepOhmic(dt, .7);
        Same(a.getState(), b.getState());
        Same(a.getDiagnostics(), b.getDiagnostics());
        REQUIRE(ra.exactJouleEnergy == rb.exactJouleEnergy);
        REQUIRE(ra.wavePhysicalEnergyChange == rb.wavePhysicalEnergyChange);
        REQUIRE(ra.modifiedEnergyDissipation == rb.modifiedEnergyDissipation);
        REQUIRE(ra.physicalBalanceResidual == rb.physicalBalanceResidual);
        const auto before = a.getDiagnostics();
        ra.finalPhysicalEnergy = 123;
        ra.endTime = 100;
        Same(a.getDiagnostics(), before);
    }
}

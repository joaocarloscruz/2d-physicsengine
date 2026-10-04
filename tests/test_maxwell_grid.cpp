#include "catch_amalgamated.hpp"
#include "physics/core/maxwell_grid.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

using namespace PhysicsEngine;
namespace {
constexpr double Pi = 3.1415926535897932384626433832795;
MaxwellFieldState Mode(const MaxwellGridConfig &c, int mx, int my, double electric, double magnetic,
                       bool magneticCosine = false) {
    const auto n = c.columns * c.rows;
    MaxwellFieldState s;
    s.ez.resize(n);
    s.hx.resize(n);
    s.hy.resize(n);
    const double ax = 2 * std::sin(Pi * mx / c.columns) / c.spacingX;
    const double ay = 2 * std::sin(Pi * my / c.rows) / c.spacingY;
    for (std::size_t j = 0; j < c.rows; ++j)
        for (std::size_t i = 0; i < c.columns; ++i) {
            const auto k = i + c.columns * j;
            const double p = 2 * Pi * (mx * double(i) / c.columns + my * double(j) / c.rows) + .31;
            const double px = p + Pi * mx / c.columns, py = p + Pi * my / c.rows;
            s.ez[k] = electric * std::cos(p);
            s.hx[k] = ay * magnetic * (magneticCosine ? std::cos(py) : std::sin(py));
            s.hy[k] = -ax * magnetic * (magneticCosine ? std::cos(px) : std::sin(px));
        }
    return s;
}
double Difference(const MaxwellFieldState &a, const MaxwellFieldState &b) {
    double error = 0;
    for (std::size_t k = 0; k < a.ez.size(); ++k)
        error = std::max({error, std::abs(a.ez[k] - b.ez[k]), std::abs(a.hx[k] - b.hx[k]),
                          std::abs(a.hy[k] - b.hy[k])});
    return error;
}
void Same(const MaxwellFieldState &a, const MaxwellFieldState &b) {
    REQUIRE(a.ez == b.ez);
    REQUIRE(a.hx == b.hx);
    REQUIRE(a.hy == b.hy);
}
void Same(const MaxwellGridDiagnostics &a, const MaxwellGridDiagnostics &b) {
    const double av[] = {a.electricEnergy,
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
    const double bv[] = {b.electricEnergy,
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
        REQUIRE(av[k] == bv[k]);
    REQUIRE(a.lastSubsteps == b.lastSubsteps);
    REQUIRE(a.lastCellVisits == b.lastCellVisits);
}
} // namespace
TEST_CASE("TMz map agrees with independent anisotropic and two-cell Fourier oracle",
          "[maxwell][oracle]") {
    const auto shape = GENERATE(
        std::pair<std::size_t, std::size_t>{12, 10}, std::pair<std::size_t, std::size_t>{2, 5},
        std::pair<std::size_t, std::size_t>{5, 2}, std::pair<std::size_t, std::size_t>{2, 2});
    MaxwellGridConfig c;
    c.columns = shape.first;
    c.rows = shape.second;
    c.spacingX = .23;
    c.spacingY = .41;
    c.permittivity = 2;
    c.permeability = 3;
    c.maxSubstep = 10;
    MaxwellGrid grid(c);
    const double ax = 2 * std::sin(Pi / c.columns) / c.spacingX,
                 ay = 2 * std::sin(Pi / c.rows) / c.spacingY;
    const double g = ax * ax + ay * ay, h = .7 * grid.getStableTimeStep(),
                 z2 = h * h * g / (c.permittivity * c.permeability);
    const double e = .7, b = -.13, d = 1 - z2 / 2;
    grid.setState(Mode(c, 1, 1, e, b));
    grid.step(h);
    const auto expected = Mode(c, 1, 1, d * e - h * g * b / c.permittivity,
                               d * b + h * (1 - z2 / 4) * e / c.permeability);
    REQUIRE(Difference(grid.getState(), expected) < 3e-14);
    REQUIRE(grid.getDiagnostics().lastSubsteps == 1);
    REQUIRE(grid.getDiagnostics().lastCellVisits == 4 * c.columns * c.rows);
    REQUIRE(grid.getDiagnostics().magneticDivergenceRms < 3e-14);
    REQUIRE(grid.getWaveSpeed() == Catch::Approx(1 / std::sqrt(6.)).epsilon(0).margin(1e-15));
}
TEST_CASE("TMz traveling mode has the discrete phase and synchronous polarization",
          "[maxwell][oracle]") {
    MaxwellGridConfig c;
    c.columns = 13;
    c.rows = 11;
    c.spacingX = .2;
    c.spacingY = .37;
    c.permittivity = 2;
    c.permeability = 3;
    c.maxSubstep = .04;
    MaxwellGrid grid(c);
    const int mx = 2, my = 3;
    const double ax = 2 * std::sin(Pi * mx / c.columns) / c.spacingX,
                 ay = 2 * std::sin(Pi * my / c.rows) / c.spacingY;
    const double omega = std::sqrt((ax * ax + ay * ay) / (c.permittivity * c.permeability));
    const double h = c.maxSubstep, z = h * omega, phase = 2 * std::asin(z / 2);
    const double polarization = std::sqrt(1 - z * z / 4) / (c.permeability * omega);
    grid.setState(Mode(c, mx, my, 1, polarization, true));
    for (int n = 0; n < 237; ++n)
        grid.step(h);
    const auto s = grid.getState();
    for (std::size_t j = 0; j < c.rows; ++j)
        for (std::size_t i = 0; i < c.columns; ++i) {
            const auto k = i + c.columns * j;
            const double p =
                2 * Pi * (mx * double(i) / c.columns + my * double(j) / c.rows) + .31 - 237 * phase;
            REQUIRE(s.ez[k] == Catch::Approx(std::cos(p)).epsilon(0).margin(8e-14));
            REQUIRE(s.hx[k] == Catch::Approx(ay * polarization * std::cos(p + Pi * my / c.rows))
                                   .epsilon(0)
                                   .margin(8e-14));
            REQUIRE(s.hy[k] == Catch::Approx(-ax * polarization * std::cos(p + Pi * mx / c.columns))
                                   .epsilon(0)
                                   .margin(8e-14));
        }
}
TEST_CASE("TMz physical energy oscillates while the fixed-h invariant stays constant",
          "[maxwell][energy]") {
    MaxwellGridConfig c;
    c.columns = 12;
    c.rows = 10;
    c.spacingX = .23;
    c.spacingY = .41;
    c.maxSubstep = 10;
    MaxwellGrid grid(c);
    const double h = .8 * grid.getStableTimeStep();
    grid.setState(Mode(c, 3, 2, .7, -.13));
    const double invariant = grid.getModifiedEnergy(h),
                 bound = invariant / (1 - std::pow(h * grid.getWaveSpeed() *
                                                       std::hypot(1 / c.spacingX, 1 / c.spacingY),
                                                   2));
    double minimum = grid.getDiagnostics().totalEnergy, maximum = minimum;
    for (int n = 0; n < 3000; ++n) {
        grid.step(h);
        const auto d = grid.getDiagnostics();
        REQUIRE(d.modifiedEnergy == Catch::Approx(invariant).epsilon(0).margin(2e-11 * invariant));
        REQUIRE(d.modifiedEnergyStep == h);
        REQUIRE(d.totalEnergy >= invariant - 2e-11 * invariant);
        REQUIRE(d.totalEnergy <= bound + 2e-11 * invariant);
        minimum = std::min(minimum, d.totalEnergy);
        maximum = std::max(maximum, d.totalEnergy);
    }
    REQUIRE(maximum - minimum > 1e-3 * invariant);
}
TEST_CASE("TMz time integration converges at second order to the semidiscrete mode",
          "[maxwell][refinement]") {
    auto error = [](int steps) {
        MaxwellGridConfig c;
        c.columns = 12;
        c.rows = 10;
        c.spacingX = .23;
        c.spacingY = .41;
        c.maxSubstep = .5 / steps;
        MaxwellGrid grid(c);
        grid.setState(Mode(c, 2, 1, 1, 0));
        grid.step(.5);
        const double ax = 2 * std::sin(2 * Pi / c.columns) / c.spacingX,
                     ay = 2 * std::sin(Pi / c.rows) / c.spacingY;
        const double omega = std::hypot(ax, ay);
        REQUIRE(grid.getDiagnostics().lastSubsteps == std::size_t(steps));
        return Difference(grid.getState(),
                          Mode(c, 2, 1, std::cos(omega * .5), std::sin(omega * .5) / omega));
    };
    const double coarse = error(16), fine = error(32), finest = error(64);
    REQUIRE(coarse / fine > 3.9);
    REQUIRE(coarse / fine < 4.1);
    REQUIRE(fine / finest > 3.9);
    REQUIRE(fine / finest < 4.1);
}
TEST_CASE("TMz fields converge at second spatial order to the continuum standing wave",
          "[maxwell][refinement]") {
    auto error = [](std::size_t nx) {
        MaxwellGridConfig c;
        c.columns = nx;
        c.rows = 3 * nx / 4;
        c.spacingX = 2 * Pi / c.columns;
        c.spacingY = 2 * Pi / c.rows;
        c.maxSubstep = 1e-4;
        MaxwellGrid grid(c);
        grid.setState(Mode(c, 1, 2, 1, 0));
        grid.step(.8);
        auto exact = grid.getState();
        const double omega = std::sqrt(5.);
        for (std::size_t j = 0; j < c.rows; ++j)
            for (std::size_t i = 0; i < c.columns; ++i) {
                const auto k = i + c.columns * j;
                const double p = i * c.spacingX + 2 * j * c.spacingY + .31;
                exact.ez[k] = std::cos(omega * .8) * std::cos(p);
                exact.hx[k] = 2 * std::sin(omega * .8) / omega * std::sin(p + c.spacingY);
                exact.hy[k] = -std::sin(omega * .8) / omega * std::sin(p + .5 * c.spacingX);
            }
        return Difference(grid.getState(), exact);
    };
    const double coarse = error(16), fine = error(32), finest = error(64);
    REQUIRE(coarse / fine > 3.8);
    REQUIRE(coarse / fine < 4.2);
    REQUIRE(fine / finest > 3.8);
    REQUIRE(fine / finest < 4.2);
}
TEST_CASE("TMz DC components and physical units remain unchanged", "[maxwell][conservation]") {
    MaxwellGridConfig c;
    c.columns = 4;
    c.rows = 3;
    c.spacingX = .5;
    c.spacingY = .25;
    c.permittivity = 2;
    c.permeability = 3;
    MaxwellGrid grid(c);
    MaxwellFieldState s;
    s.ez.assign(12, 2);
    s.hx.assign(12, -1);
    s.hy.assign(12, 3);
    grid.setState(s);
    for (int n = 0; n < 100; ++n)
        grid.step(.25);
    Same(s, grid.getState());
    const auto d = grid.getDiagnostics();
    REQUIRE(d.electricEnergy == Catch::Approx(6).epsilon(0).margin(1e-13));
    REQUIRE(d.magneticEnergy == Catch::Approx(22.5).epsilon(0).margin(1e-13));
    REQUIRE(d.totalEnergy == d.modifiedEnergy);
    REQUIRE(d.meanEz == Catch::Approx(2).epsilon(0).margin(1e-14));
    REQUIRE(d.meanHx == Catch::Approx(-1).epsilon(0).margin(1e-14));
    REQUIRE(d.meanHy == 3);
    REQUIRE(d.time == 25);
    REQUIRE(d.magneticDivergenceRms == 0);
    REQUIRE(d.maxAbsMagneticDivergence == 0);
}
TEST_CASE("TMz curl preserves nonzero initial magnetic divergence and component means",
          "[maxwell][conservation]") {
    MaxwellGridConfig c;
    c.columns = 13;
    c.rows = 11;
    c.spacingX = .2;
    c.spacingY = .37;
    c.maxSubstep = .02;
    MaxwellGrid grid(c);
    auto s = Mode(c, 2, 3, 1, 0);
    for (std::size_t j = 0; j < c.rows; ++j)
        for (std::size_t i = 0; i < c.columns; ++i) {
            const auto k = i + c.columns * j;
            s.ez[k] += .17;
            s.hx[k] = -.2 + .03 * std::sin(2 * Pi * i / c.columns);
            s.hy[k] = .3 + .04 * std::cos(2 * Pi * j / c.rows);
        }
    grid.setState(s);
    const auto initial = grid.getMagneticDivergence();
    const auto before = grid.getDiagnostics();
    REQUIRE(before.magneticDivergenceRms > .01);
    for (int n = 0; n < 1000; ++n)
        grid.step(.02);
    const auto actual = grid.getMagneticDivergence();
    const auto d = grid.getDiagnostics();
    for (std::size_t k = 0; k < actual.size(); ++k)
        REQUIRE(actual[k] == Catch::Approx(initial[k]).epsilon(0).margin(3e-13));
    REQUIRE(d.meanEz == Catch::Approx(before.meanEz).epsilon(0).margin(1e-14));
    REQUIRE(d.meanHx == Catch::Approx(before.meanHx).epsilon(0).margin(1e-14));
    REQUIRE(d.meanHy == Catch::Approx(before.meanHy).epsilon(0).margin(1e-14));
}
TEST_CASE("TMz replay is deterministic and snapshots cannot mutate the grid",
          "[maxwell][transaction]") {
    MaxwellGrid a, b;
    a.setState(Mode(a.getConfig(), 1, 2, .7, -.13));
    b.setState(a.getState());
    for (double dt : {0., .1, .05, .037, .2, 0.}) {
        a.step(dt);
        b.step(dt);
        Same(a.getState(), b.getState());
        Same(a.getDiagnostics(), b.getDiagnostics());
    }
    auto s = a.getState();
    const auto before = a.getState();
    s.ez[0] = 99;
    s.hx.clear();
    Same(a.getState(), before);
    auto c = a.getConfig();
    c.columns = 99;
    REQUIRE(a.getConfig().columns == 16);
    auto d = a.getDiagnostics();
    d.time = 99;
    REQUIRE(a.getDiagnostics().time != 99);
    auto div = a.getMagneticDivergence();
    div[0] = 99;
    REQUIRE(a.getMagneticDivergence()[0] != 99);
    const double clock = a.getDiagnostics().time;
    a.setState(before);
    REQUIRE(a.getDiagnostics().time == clock);
    REQUIRE(a.getDiagnostics().lastSubsteps == 0);
    REQUIRE(a.getDiagnostics().lastCellVisits == 0);
    REQUIRE(a.getDiagnostics().modifiedEnergyStep == 0);
    REQUIRE(a.getDiagnostics().modifiedEnergy == a.getDiagnostics().totalEnergy);
    const auto zero = a.getDiagnostics();
    a.step(0);
    Same(a.getDiagnostics(), zero);
}
TEST_CASE("TMz exact configured duration and strict CFL rounding obey one-step budgets",
          "[maxwell][resource]") {
    MaxwellGridConfig c;
    c.columns = 2;
    c.rows = 2;
    c.maximumSubsteps = 1;
    c.maximumCellVisits = 16;
    SECTION("loose CFL with decimal user limit") {
        c.spacingX = 10;
        c.spacingY = 10;
        c.maxSubstep = .01;
    }
    SECTION("computed CFL limit") {
        c.maxSubstep = 10;
    }
    MaxwellGrid grid(c);
    const double limit = grid.getStableTimeStep();
    grid.step(limit);
    REQUIRE(grid.getDiagnostics().lastSubsteps == 1);
    REQUIRE(grid.getDiagnostics().lastCellVisits == 16);
    const auto s = grid.getState();
    const auto d = grid.getDiagnostics();
    REQUIRE_THROWS_AS(grid.step(std::nextafter(limit, std::numeric_limits<double>::infinity())),
                      std::length_error);
    Same(grid.getState(), s);
    Same(grid.getDiagnostics(), d);
}
TEST_CASE("TMz failed step and setter retain all fields clock and diagnostics",
          "[maxwell][transaction]") {
    MaxwellGridConfig c;
    c.columns = 2;
    c.rows = 2;
    c.maxSubstep = .01;
    c.maximumCellVisits = 16;
    MaxwellGrid grid(c);
    grid.setState(Mode(c, 1, 1, .7, -.13));
    grid.step(.01);
    const auto s = grid.getState();
    const auto d = grid.getDiagnostics();
    for (double dt :
         {-.1, std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()}) {
        REQUIRE_THROWS_AS(grid.step(dt), std::invalid_argument);
        Same(grid.getState(), s);
        Same(grid.getDiagnostics(), d);
    }
    for (double dt : {.02, std::numeric_limits<double>::max()}) {
        REQUIRE_THROWS_AS(grid.step(dt), std::length_error);
        Same(grid.getState(), s);
        Same(grid.getDiagnostics(), d);
    }
    REQUIRE_THROWS_AS(grid.step(std::numeric_limits<double>::denorm_min()), std::overflow_error);
    Same(grid.getState(), s);
    Same(grid.getDiagnostics(), d);
    auto bad = s;
    SECTION("size") {
        bad.hy.pop_back();
    }
    SECTION("nonfinite") {
        bad.hx[0] = std::numeric_limits<double>::infinity();
    }
    REQUIRE_THROWS_AS(grid.setState(bad), std::invalid_argument);
    Same(grid.getState(), s);
    Same(grid.getDiagnostics(), d);
}
TEST_CASE("TMz late energy overflow and finite difference overflow roll back", "[maxwell][range]") {
    MaxwellGridConfig c;
    c.columns = 2;
    c.rows = 2;
    c.maxSubstep = 1;
    SECTION("late physical energy overflow") {
        MaxwellGrid grid(c);
        auto s = Mode(c, 1, 1, 0, 3e153);
        grid.setState(s);
        const auto d = grid.getDiagnostics();
        REQUIRE(std::isfinite(d.totalEnergy));
        REQUIRE(d.totalEnergy > 1e308);
        REQUIRE_THROWS_AS(grid.step(.6), std::overflow_error);
        Same(grid.getState(), s);
        Same(grid.getDiagnostics(), d);
    }
    SECTION("finite field subtraction overflows") {
        c.spacingX = 1e-6;
        c.spacingY = 1e-6;
        c.permittivity = 1e-300;
        c.permeability = 1e300;
        MaxwellGrid grid(c);
        auto s = grid.getState();
        s.ez = {1e308, -1e308, 1e308, -1e308};
        grid.setState(s);
        const auto d = grid.getDiagnostics();
        REQUIRE_THROWS_AS(grid.step(grid.getStableTimeStep()), std::overflow_error);
        Same(grid.getState(), s);
        Same(grid.getDiagnostics(), d);
    }
}
TEST_CASE("TMz dimensions medium coefficients resource ceilings and reference h are checked",
          "[maxwell][validation]") {
    MaxwellGridConfig c;
    for (std::size_t count : {std::size_t(0), std::size_t(1),
                              std::numeric_limits<std::size_t>::max(), std::size_t(262145)}) {
        c.columns = count;
        REQUIRE_THROWS_AS(MaxwellGrid(c), std::length_error);
    }
    c = {};
    c.columns = 65536;
    c.rows = 65536;
    REQUIRE_THROWS_AS(MaxwellGrid(c), std::length_error);
    c = {};
    c.columns = 512;
    c.rows = 512;
    REQUIRE_NOTHROW(MaxwellGrid(c));
    for (double value : {0., -1., std::numeric_limits<double>::quiet_NaN(),
                         std::numeric_limits<double>::infinity(), 1e-320}) {
        c = {};
        c.spacingX = value;
        REQUIRE_THROWS_AS(MaxwellGrid(c), std::invalid_argument);
        c = {};
        c.permittivity = value;
        REQUIRE_THROWS_AS(MaxwellGrid(c), std::invalid_argument);
        c = {};
        c.permeability = value;
        REQUIRE_THROWS_AS(MaxwellGrid(c), std::invalid_argument);
    }
    c = {};
    c.spacingX = 1e308;
    REQUIRE_THROWS_AS(MaxwellGrid(c), std::invalid_argument);
    c = {};
    c.permittivity = 1e308;
    REQUIRE_NOTHROW(MaxwellGrid(c));
    c = {};
    c.permeability = 1e308;
    REQUIRE_NOTHROW(MaxwellGrid(c));
    c = {};
    c.cflSafety = 1;
    REQUIRE_THROWS_AS(MaxwellGrid(c), std::invalid_argument);
    c = {};
    c.maximumSubsteps = 0;
    REQUIRE_THROWS_AS(MaxwellGrid(c), std::invalid_argument);
    c = {};
    c.maximumSubsteps = 1000001;
    REQUIRE_THROWS_AS(MaxwellGrid(c), std::invalid_argument);
    c = {};
    c.maximumCellVisits = 1000000001;
    REQUIRE_THROWS_AS(MaxwellGrid(c), std::invalid_argument);
    MaxwellGrid grid;
    REQUIRE_THROWS_AS(grid.getModifiedEnergy(-1), std::invalid_argument);
    REQUIRE_THROWS_AS(grid.getModifiedEnergy(1 / std::sqrt(2.)), std::invalid_argument);
    REQUIRE_THROWS_AS(grid.getModifiedEnergy(std::numeric_limits<double>::denorm_min()),
                      std::overflow_error);
    grid.setState(Mode(grid.getConfig(), 1, 1, 1e-150, 0));
    grid.step(.1);
    REQUIRE(grid.getDiagnostics().totalEnergy > 0);
    REQUIRE(std::isfinite(grid.getDiagnostics().totalEnergy));
}
TEST_CASE("TMz energy preserves representable aggregate subnormals and rejects complete underflow",
          "[maxwell][range]") {
    MaxwellGrid grid;
    auto s = grid.getState();
    std::fill(s.ez.begin(), s.ez.end(), 2e-163);
    const long double e = s.ez[0];
    const long double reference = .5L * s.ez.size() * e * e;
    // Associate the aggregate weight first: this remains a nonzero oracle when
    // long double has the same exponent range as double (including MSVC).
    const double scaledReference = (.5 * static_cast<double>(s.ez.size()) * s.ez[0]) * s.ez[0];
    REQUIRE(scaledReference == std::numeric_limits<double>::denorm_min());
    REQUIRE(scaledReference == static_cast<double>(reference));
    REQUIRE(static_cast<double>(reference) > 0);
    grid.setState(s);
    REQUIRE(grid.getDiagnostics().electricEnergy == static_cast<double>(reference));
    REQUIRE(grid.getDiagnostics().electricEnergy == std::numeric_limits<double>::denorm_min());
    grid.step(.1);
    Same(grid.getState(), s);
    REQUIRE(grid.getDiagnostics().totalEnergy == static_cast<double>(reference));
    const auto before = grid.getState();
    const auto d = grid.getDiagnostics();
    grid.step(0);
    Same(grid.getState(), before);
    Same(grid.getDiagnostics(), d);
    SECTION("electric norm energy below storage range") {
        std::fill(s.ez.begin(), s.ez.end(), 1e-200);
    }
    SECTION("magnetic norm energy below storage range") {
        std::fill(s.hx.begin(), s.hx.end(), 1e-200);
    }
    REQUIRE_THROWS_AS(grid.setState(s), std::overflow_error);
    Same(grid.getState(), before);
    Same(grid.getDiagnostics(), d);
}
TEST_CASE("TMz tiny uniform medium constants use scaled speed and finite CFL coefficients",
          "[maxwell][range]") {
    MaxwellGridConfig c;
    c.columns = 4;
    c.rows = 4;
    c.maxSubstep = 1;
    MaxwellGrid base(c);
    base.setState(Mode(c, 1, 1, .7, -.13));
    c.permittivity = 1e-308;
    c.permeability = 1e-308;
    MaxwellGrid tiny(c);
    tiny.setState(base.getState());
    const double h = tiny.getStableTimeStep();
    REQUIRE(h > 0);
    REQUIRE(tiny.getWaveSpeed() > 1e307);
    tiny.step(h);
    base.step(h / 1e-308);
    REQUIRE(Difference(tiny.getState(), base.getState()) < 3e-15);
    REQUIRE(tiny.getDiagnostics().totalEnergy > 0);
    REQUIRE(std::isfinite(tiny.getDiagnostics().totalEnergy));
}
TEST_CASE("TMz modified quadratic form rejects a lost positive subnormal remainder",
          "[maxwell][range]") {
    MaxwellGridConfig c;
    c.columns = 2;
    c.rows = 2;
    MaxwellGrid grid(c);
    auto s = grid.getState();
    s.ez = {2e-162, -2e-162, -2e-162, 2e-162};
    grid.setState(s);
    const auto before = grid.getDiagnostics();
    REQUIRE(before.totalEnergy > 0);
    // The strict physical CFL permits .69, but the highest-mode invariant is
    // (1-2*.69²)*raw energy: positive and below the double subnormal range.
    REQUIRE_THROWS_AS(grid.getModifiedEnergy(.69), std::overflow_error);
    Same(grid.getState(), s);
    Same(grid.getDiagnostics(), before);
}

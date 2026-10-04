#include "catch_amalgamated.hpp"
#include "physics/core/maxwell_grid.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
using namespace PhysicsEngine;
namespace {
// Wider raw arithmetic with opposite large values adjacent is independent of
// the binary-scaled compensated production observer. These four-value controls
// fit its precision/range even on platforms where long double is double.
double StoredMean(std::vector<double> values) {
    std::sort(values.begin(), values.end(),
              [](double a, double b) { return std::abs(a) > std::abs(b); });
    long double sum = 0;
    for (double x : values)
        sum += x;
    return double(sum / values.size());
}
void Coherent(const MaxwellGrid &g) {
    const auto s = g.getState();
    const auto d = g.getDiagnostics();
    REQUIRE(d.meanEz == StoredMean(s.ez));
    REQUIRE(d.meanHx == StoredMean(s.hx));
    REQUIRE(d.meanHy == StoredMean(s.hy));
}
void SameDiagnostics(const MaxwellGridDiagnostics &a, const MaxwellGridDiagnostics &b) {
    for (auto member :
         {&MaxwellGridDiagnostics::electricEnergy, &MaxwellGridDiagnostics::magneticEnergy,
          &MaxwellGridDiagnostics::totalEnergy, &MaxwellGridDiagnostics::modifiedEnergy,
          &MaxwellGridDiagnostics::modifiedEnergyStep, &MaxwellGridDiagnostics::meanEz,
          &MaxwellGridDiagnostics::meanHx, &MaxwellGridDiagnostics::meanHy,
          &MaxwellGridDiagnostics::maxAbsEz, &MaxwellGridDiagnostics::maxAbsHx,
          &MaxwellGridDiagnostics::maxAbsHy, &MaxwellGridDiagnostics::magneticDivergenceRms,
          &MaxwellGridDiagnostics::maxAbsMagneticDivergence, &MaxwellGridDiagnostics::time,
          &MaxwellGridDiagnostics::stableTimeStep, &MaxwellGridDiagnostics::lastSubstep})
        REQUIRE(a.*member == b.*member);
    REQUIRE(a.lastSubsteps == b.lastSubsteps);
    REQUIRE(a.lastCellVisits == b.lastCellVisits);
}
} // namespace
TEST_CASE("Maxwell non-power-of-two constant means retain their represented value",
          "[maxwell][means]") {
    const double x = GENERATE(.1, .7, -1.2345678901234567);
    MaxwellGridConfig c;
    c.columns = 3;
    c.rows = 2;
    MaxwellGrid g(c);
    auto s = g.getState();
    s.ez.assign(6, x);
    s.hx.assign(6, -x);
    s.hy.assign(6, x);
    g.setState(s);
    auto check = [&]() {
        const auto fields = g.getState();
        const auto d = g.getDiagnostics();
        REQUIRE(d.meanEz == fields.ez.front());
        REQUIRE(d.meanHx == fields.hx.front());
        REQUIRE(d.meanHy == fields.hy.front());
    };
    check();
    g.step(.01);
    check();
    g.stepOhmic(.01, .7);
    check();
}
TEST_CASE("Maxwell subnormal constant means survive independently weighted aggregate energy",
          "[maxwell][means][range]") {
    const double magnitude = GENERATE(6e-319, 7e-319);
    const double sign = GENERATE(-1., 1.);
    MaxwellGridConfig c;
    c.columns = c.rows = 512;
    c.permittivity = c.permeability = 1e308;
    MaxwellGrid g(c);
    const auto n = c.columns * c.rows;
    const double x = sign * magnitude;
    auto s = g.getState();
    s.ez.assign(n, x);
    s.hx.assign(n, -x);
    s.hy.assign(n, x);
    const double weighted = std::sqrt(double(n)) * std::sqrt(c.permittivity / 2) * magnitude;
    const double electric = weighted * weighted,
                 magnetic = (std::sqrt(2.) * weighted) * (std::sqrt(2.) * weighted);
    REQUIRE(electric == std::numeric_limits<double>::denorm_min());
    REQUIRE(magnetic > 0);
    g.setState(s);
    auto check = [&]() {
        const auto d = g.getDiagnostics();
        REQUIRE(d.meanEz == x);
        REQUIRE(d.meanHx == -x);
        REQUIRE(d.meanHy == x);
        REQUIRE(d.electricEnergy == electric);
        REQUIRE(d.magneticEnergy == magnetic);
        REQUIRE(g.getState().ez == s.ez);
        REQUIRE(g.getState().hx == s.hx);
        REQUIRE(g.getState().hy == s.hy);
    };
    check();
    REQUIRE(g.getModifiedEnergy(.01) == electric + magnetic);
    g.step(.01);
    check();
    REQUIRE(g.getDiagnostics().lastCellVisits == 4 * n);
    const auto report = g.stepOhmic(.01, 0);
    check();
    REQUIRE(report.cellVisits == 4 * n);
    auto copy = g.getState();
    copy.ez[0] = 0;
    REQUIRE(g.getDiagnostics().meanEz == x);
}
TEST_CASE("Maxwell signed cancellation means observe represented fields on both stepping paths",
          "[maxwell][means]") {
    const double sign = GENERATE(-1., 1.);
    MaxwellGridConfig c;
    c.columns = c.rows = 2;
    MaxwellGrid g(c);
    std::array<double, 4> values{1e16, 1, -1e16, 0};
    std::sort(values.begin(), values.end());
    do {
        MaxwellFieldState s;
        for (double x : values) {
            s.ez.push_back(sign * x);
            s.hx.push_back(-sign * x);
            s.hy.push_back(sign * x);
        }
        g.setState(s);
        REQUIRE(g.getDiagnostics().meanEz == sign * .25);
        REQUIRE(g.getDiagnostics().meanHx == -sign * .25);
        REQUIRE(g.getDiagnostics().meanHy == sign * .25);
    } while (std::next_permutation(values.begin(), values.end()));
    MaxwellFieldState s{{1e16, 1, -1e16, 0}, {1e16, 1, -1e16, 0}, {-1e16, -1, 1e16, 0}};
    g.setState(s);
    g.step(.01);
    Coherent(g);
    REQUIRE(g.getDiagnostics().lastCellVisits == 16);
    const auto report = g.stepOhmic(.01, .7);
    Coherent(g);
    REQUIRE(report.cellVisits == 36);
}
TEST_CASE("Maxwell mean erased scaling and nonzero final underflow reject transactionally",
          "[maxwell][means][validation]") {
    MaxwellGridConfig c;
    c.columns = c.rows = 2;
    MaxwellGrid g(c);
    auto s = g.getState();
    s.ez.assign(4, 1);
    g.setState(s);
    g.stepOhmic(.01, .7);
    const auto before = g.getState();
    const auto diagnostics = g.getDiagnostics();
    // Relative to ilogb(1e150)==498 this term becomes 1.5 denorm_min:
    // normalization/rescaling rounds it rather than erasing it completely.
    const double partial = std::scalbn(3 * std::numeric_limits<double>::denorm_min(), 497);
    for (auto fields :
         {MaxwellFieldState{{1e150, std::numeric_limits<double>::denorm_min(), -1e150, 0},
                            {0, 0, 0, 0},
                            {0, 0, 0, 0}},
          MaxwellFieldState{{std::numeric_limits<double>::denorm_min(), 1e150, -1e150, 0},
                            {0, 0, 0, 0},
                            {0, 0, 0, 0}},
          MaxwellFieldState{{1e150, partial, -1e150, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}},
          MaxwellFieldState{{partial, 1e150, -1e150, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}}}) {
        for (int f = 0; f < 3; ++f) {
            if (f == 1)
                std::swap(fields.ez, fields.hx);
            if (f == 2)
                std::swap(fields.hx, fields.hy);
            REQUIRE_THROWS_AS(g.setState(fields), std::overflow_error);
            REQUIRE(g.getState().ez == before.ez);
            REQUIRE(g.getState().hx == before.hx);
            REQUIRE(g.getState().hy == before.hy);
            SameDiagnostics(g.getDiagnostics(), diagnostics);
        }
    }
    c.columns = c.rows = 512;
    c.permittivity = 1e308;
    MaxwellGrid sub(c);
    auto a = sub.getState();
    a.ez.assign(a.ez.size(), 6e-319);
    sub.setState(a);
    const auto subBefore = sub.getState();
    const auto subD = sub.getDiagnostics();
    for (std::size_t k = 0; k < a.ez.size(); ++k)
        a.ez[k] = k % 2 ? -6e-319 : 6e-319;
    a.ez[1] += std::numeric_limits<double>::denorm_min();
    REQUIRE_THROWS_AS(sub.setState(a), std::overflow_error);
    REQUIRE(sub.getState().ez == subBefore.ez);
    REQUIRE(sub.getState().hx == subBefore.hx);
    REQUIRE(sub.getState().hy == subBefore.hy);
    SameDiagnostics(sub.getDiagnostics(), subD);
}

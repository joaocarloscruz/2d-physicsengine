#include "catch_amalgamated.hpp"
#include "physics/core/periodic_electrostatic_grid.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <tuple>
using namespace PhysicsEngine;
namespace {
constexpr double Pi = 3.1415926535897932384626433832795;
auto Values(const ElectrostaticDiagnostics &d) {
    return std::make_tuple(
        d.iterations, d.cellVisits, d.residualRestarts, d.permittivity, d.originalChargeMean,
        d.effectiveChargeMean, d.originalIntegratedCharge, d.effectiveIntegratedCharge,
        d.neutralityMeanAllowance, d.removedChargeMean, d.maximumSourceCorrection,
        d.sourceCorrectionAllowance, d.effectiveChargeRms, d.targetGaussRms, d.finalGaussRms,
        d.maximumAbsGauss, d.originalGaussRms, d.maximumAbsOriginalGauss, d.potentialMean,
        d.meanFieldX, d.meanFieldY, d.curlRms, d.maximumAbsCurl, d.fieldEnergy, d.sourceEnergy,
        d.residualEnergyCorrection, d.residualEnergyBound, d.energyIdentityError,
        d.roundoffEnergyAllowance, d.hasSolution, d.zeroSource);
}
void Same(const ElectrostaticSnapshot &a, const ElectrostaticSnapshot &b) {
    REQUIRE(a.originalCharge == b.originalCharge);
    REQUIRE(a.effectiveCharge == b.effectiveCharge);
    REQUIRE(a.potential == b.potential);
    REQUIRE(a.field.xFaces == b.field.xFaces);
    REQUIRE(a.field.yFaces == b.field.yFaces);
    REQUIRE(a.gaussResidual == b.gaussResidual);
    REQUIRE(a.curl == b.curl);
    REQUIRE(Values(a.diagnostics) == Values(b.diagnostics));
}
void Near(double a, double b, double tolerance) {
    REQUIRE(a == Catch::Approx(b).epsilon(0).margin(tolerance));
}
double Mean(const std::vector<double> &v) {
    return std::accumulate(v.begin(), v.end(), 0.0) / v.size();
}
double Rms(const std::vector<double> &v) {
    double norm = 0;
    for (double x : v)
        norm = std::hypot(norm, x);
    return norm / std::sqrt(double(v.size()));
}
std::vector<double> Mode(const ElectrostaticGridConfig &c, int mx, int my, double amplitude = 1,
                         double phase = .31) {
    std::vector<double> phi(c.columns * c.rows);
    for (std::size_t j = 0; j < c.rows; ++j)
        for (std::size_t i = 0; i < c.columns; ++i)
            phi[i + c.columns * j] =
                amplitude *
                std::cos(2 * Pi * (mx * (i + .5) / c.columns + my * (j + .5) / c.rows) + phase);
    return phi;
}
double Eigenvalue(const ElectrostaticGridConfig &c, int mx, int my) {
    return 4 * std::pow(std::sin(Pi * mx / c.columns) / c.spacingX, 2) +
           4 * std::pow(std::sin(Pi * my / c.rows) / c.spacingY, 2);
}
std::vector<double> Charge(const ElectrostaticGridConfig &c, int mx, int my, double amplitude = 1,
                           double phase = .31) {
    auto q = Mode(c, mx, my, amplitude, phase);
    for (double &x : q)
        x *= c.permittivity * Eigenvalue(c, mx, my);
    return q;
}
// Independent dense reduced matrix: fix final potential to zero, solve Gaussian
// elimination with pivoting, then choose the mean-zero gauge. No CG code used.
std::vector<double> Dense(const ElectrostaticGridConfig &c, const std::vector<double> &rho) {
    const auto n = rho.size(), m = n - 1;
    std::vector<std::vector<double>> a(m, std::vector<double>(m + 1));
    for (std::size_t j = 0; j < c.rows; ++j)
        for (std::size_t i = 0; i < c.columns; ++i) {
            const auto k = i + c.columns * j;
            if (k == m)
                continue;
            const std::size_t neighbors[] = {(i + 1) % c.columns + c.columns * j,
                                             (i + c.columns - 1) % c.columns + c.columns * j,
                                             i + c.columns * ((j + 1) % c.rows),
                                             i + c.columns * ((j + c.rows - 1) % c.rows)};
            for (std::size_t edge = 0; edge < 4; ++edge) {
                const double coefficient =
                    c.permittivity / std::pow(edge < 2 ? c.spacingX : c.spacingY, 2);
                a[k][k] += coefficient;
                if (neighbors[edge] < m)
                    a[k][neighbors[edge]] -= coefficient;
            }
            a[k][m] = rho[k];
        }
    for (std::size_t k = 0; k < m; ++k) {
        std::size_t pivot = k;
        for (std::size_t i = k + 1; i < m; ++i)
            if (std::abs(a[i][k]) > std::abs(a[pivot][k]))
                pivot = i;
        std::swap(a[k], a[pivot]);
        REQUIRE(a[k][k] != 0);
        for (std::size_t i = k + 1; i < m; ++i) {
            const double ratio = a[i][k] / a[k][k];
            for (std::size_t col = k; col <= m; ++col)
                a[i][col] -= ratio * a[k][col];
        }
    }
    std::vector<double> p(n);
    for (std::size_t reverse = m; reverse > 0; --reverse) {
        const auto i = reverse - 1;
        double rhs = a[i][m];
        for (std::size_t col = i + 1; col < m; ++col)
            rhs -= a[i][col] * p[col];
        p[i] = rhs / a[i][i];
    }
    const double mean = Mean(p);
    for (double &x : p)
        x -= mean;
    return p;
}
void IndependentAudit(const ElectrostaticSnapshot &s, const ElectrostaticGridConfig &c) {
    const auto nx = c.columns, ny = c.rows, n = nx * ny;
    std::vector<double> r(n), original(n), curl(n);
    double field = 0, source = 0, correction = 0;
    for (std::size_t j = 0; j < ny; ++j)
        for (std::size_t i = 0; i < nx; ++i) {
            const auto k = i + nx * j, left = (i + nx - 1) % nx + nx * j,
                       down = i + nx * ((j + ny - 1) % ny);
            const auto right = (i + 1) % nx + nx * j, up = i + nx * ((j + 1) % ny);
            Near(s.field.xFaces[k], -(s.potential[k] - s.potential[left]) / c.spacingX, 2e-14);
            Near(s.field.yFaces[k], -(s.potential[k] - s.potential[down]) / c.spacingY, 2e-14);
            const double div = (s.field.xFaces[right] - s.field.xFaces[k]) / c.spacingX +
                               (s.field.yFaces[up] - s.field.yFaces[k]) / c.spacingY;
            r[k] = c.permittivity * div - s.effectiveCharge[k];
            original[k] = c.permittivity * div - s.originalCharge[k];
            curl[k] = (s.field.yFaces[k] - s.field.yFaces[left]) / c.spacingX -
                      (s.field.xFaces[k] - s.field.xFaces[down]) / c.spacingY;
            Near(s.gaussResidual[k], r[k], 2e-13);
            Near(s.curl[k], curl[k], 2e-14);
            field +=
                .5 * c.permittivity * c.spacingX * c.spacingY *
                (s.field.xFaces[k] * s.field.xFaces[k] + s.field.yFaces[k] * s.field.yFaces[k]);
            source += .5 * c.spacingX * c.spacingY * s.potential[k] * s.effectiveCharge[k];
            correction += .5 * c.spacingX * c.spacingY * s.potential[k] * r[k];
        }
    const auto &d = s.diagnostics;
    Near(d.finalGaussRms, Rms(r), 3e-14);
    Near(d.originalGaussRms, Rms(original), 3e-14);
    Near(d.curlRms, Rms(curl), 2e-14);
    Near(d.fieldEnergy, field, 4e-12);
    Near(d.sourceEnergy, source, 4e-12);
    Near(d.residualEnergyCorrection, correction, 4e-13);
    Near(field - source, correction, 5e-12);
    REQUIRE(std::abs(d.residualEnergyCorrection) <= d.residualEnergyBound + 3e-13);
    REQUIRE(std::abs(d.energyIdentityError) <= d.roundoffEnergyAllowance);
    Near(d.potentialMean, 0, 2e-14);
    Near(d.meanFieldX, 0, 2e-14);
    Near(d.meanFieldY, 0, 2e-14);
    REQUIRE(d.finalGaussRms <= d.targetGaussRms);
}
} // namespace

TEST_CASE("Periodic electrostatics matches independently reduced dense gauge solves",
          "[electrostatic][dense]") {
    for (const auto dims :
         {std::pair<std::size_t, std::size_t>{2, 2}, {2, 3}, {3, 2}, {4, 3}, {7, 5}}) {
        ElectrostaticGridConfig c{dims.first, dims.second, .23, .41, 2.5};
        PeriodicElectrostaticGrid grid(c);
        std::vector<double> charge(c.columns * c.rows);
        for (std::size_t k = 0; k + 1 < charge.size(); ++k)
            charge[k] = int(k % 5) - 2;
        charge.back() = -std::accumulate(charge.begin(), charge.end() - 1, 0.0);
        ElectrostaticSolveConfig options;
        options.absoluteGaussTolerance = 1e-12;
        options.relativeGaussTolerance = 1e-12;
        grid.solve(charge, options);
        const auto s = grid.getSnapshot();
        const auto expected = Dense(c, charge);
        for (std::size_t k = 0; k < charge.size(); ++k)
            Near(s.potential[k], expected[k], 3e-13);
        IndependentAudit(s, c);
    }
}
TEST_CASE("Periodic electrostatic Fourier eigenvalues and staggered polarization are exact",
          "[electrostatic][fourier]") {
    for (const auto dims : {std::pair<std::size_t, std::size_t>{2, 2}, {2, 5}, {5, 2}, {13, 11}}) {
        ElectrostaticGridConfig c{dims.first, dims.second, .23, .41, 2.5};
        PeriodicElectrostaticGrid grid(c);
        for (const auto modes : {std::pair<int, int>{1, 0}, {0, 1}, {1, 1}}) {
            const auto mx = modes.first, my = modes.second;
            grid.solve(Charge(c, mx, my, .7));
            const auto s = grid.getSnapshot();
            const auto expected = Mode(c, mx, my, .7);
            for (std::size_t j = 0; j < c.rows; ++j)
                for (std::size_t i = 0; i < c.columns; ++i) {
                    const auto k = i + c.columns * j;
                    Near(s.potential[k], expected[k], 3e-13);
                    Near(s.field.xFaces[k],
                         1.4 * std::sin(Pi * mx / c.columns) / c.spacingX *
                             std::sin(2 * Pi *
                                          (mx * double(i) / c.columns + my * (j + .5) / c.rows) +
                                      .31),
                         2e-12);
                    Near(s.field.yFaces[k],
                         1.4 * std::sin(Pi * my / c.rows) / c.spacingY *
                             std::sin(2 * Pi *
                                          (mx * (i + .5) / c.columns + my * double(j) / c.rows) +
                                      .31),
                         2e-12);
                }
            IndependentAudit(s, c);
            REQUIRE(s.diagnostics.iterations <= 2);
        }
    }
}
TEST_CASE("Electrostatic stored residual correction explains finite-tolerance energy",
          "[electrostatic][energy]") {
    ElectrostaticGridConfig c{9, 7, .31, .47, 1.7};
    PeriodicElectrostaticGrid grid(c);
    auto a = Charge(c, 1, 2, .6), b = Charge(c, 3, 1, .3);
    for (std::size_t k = 0; k < a.size(); ++k)
        a[k] += b[k];
    ElectrostaticSolveConfig options;
    options.absoluteGaussTolerance = 0;
    options.relativeGaussTolerance = .6;
    const auto d = grid.solve(a, options);
    REQUIRE(d.iterations == 1);
    REQUIRE(d.finalGaussRms > 1);
    IndependentAudit(grid.getSnapshot(), c);
    // Zero-start CG has a Galerkin-orthogonal residual: energy work can agree
    // despite substantial solution error. Compare to an independent exact solve.
    const auto exact = Dense(c, a);
    double exactEnergy = 0;
    for (std::size_t k = 0; k < a.size(); ++k)
        exactEnergy += .5 * c.spacingX * c.spacingY * a[k] * exact[k];
    REQUIRE(std::abs(d.fieldEnergy - exactEnergy) > 1e-3);
    REQUIRE(d.residualEnergyBound > 1);
}
TEST_CASE("Electrostatic superposition sign permittivity and charge scale independently",
          "[electrostatic][linearity]") {
    ElectrostaticGridConfig c{9, 7, .31, .47, 1.7};
    PeriodicElectrostaticGrid a(c), b(c), combined(c);
    const auto qa = Charge(c, 1, 2, .6), qb = Charge(c, 3, 1, .3);
    a.solve(qa);
    b.solve(qb);
    auto q = qa;
    for (std::size_t k = 0; k < q.size(); ++k)
        q[k] += qb[k];
    combined.solve(q);
    const auto sa = a.getSnapshot(), sb = b.getSnapshot(), sum = combined.getSnapshot();
    for (std::size_t k = 0; k < q.size(); ++k)
        Near(sum.potential[k], sa.potential[k] + sb.potential[k], 4e-13);
    for (double scale : {-1., 1e-100, 1e100}) {
        auto scaled = q;
        for (double &x : scaled)
            x *= scale;
        ElectrostaticSolveConfig o;
        o.absoluteGaussTolerance = 0;
        o.relativeGaussTolerance = 1e-11;
        combined.solve(scaled, o);
        const auto s = combined.getSnapshot();
        for (std::size_t k = 0; k < q.size(); ++k)
            Near(s.potential[k] / scale, sum.potential[k], 4e-12);
        Near(s.diagnostics.fieldEnergy / (scale * scale), sum.diagnostics.fieldEnergy, 1e-10);
    }
    c.permittivity *= 3;
    PeriodicElectrostaticGrid dielectric(c);
    dielectric.solve(q);
    const auto changed = dielectric.getSnapshot();
    for (std::size_t k = 0; k < q.size(); ++k)
        Near(changed.potential[k] * 3, sum.potential[k], 4e-13);
    Near(changed.diagnostics.fieldEnergy * 3, sum.diagnostics.fieldEnergy, 1e-11);
}
TEST_CASE("Electrostatic physical Gauss scaling avoids overflowing unweighted divergence",
          "[electrostatic][range]") {
    ElectrostaticGridConfig c{2, 2, 3e-154, 3e-154, 1e-308};
    PeriodicElectrostaticGrid grid(c);
    for (double magnitude : {1e150, 1e154}) {
        const auto d = grid.solve({magnitude, -magnitude, magnitude, -magnitude});
        const auto s = grid.getSnapshot();
        // The second case also overflows the unweighted difference of opposite
        // finite faces. Analytical phi=rho*dx^2/(4 epsilon), Ex=-rho*dx/(2 epsilon).
        for (std::size_t k = 0; k < 4; ++k) {
            const double sign = k % 2 == 0 ? 1 : -1;
            Near(s.potential[k] / magnitude, sign * 2.25, 3e-15);
            Near(s.field.xFaces[k] / (magnitude * 1e154), -sign * 1.5, 3e-15);
            REQUIRE(s.field.yFaces[k] == 0);
            REQUIRE(s.curl[k] == 0);
        }
        Near(d.fieldEnergy / (magnitude / 1e150) / (magnitude / 1e150), 4.05e-7, 3e-21);
        REQUIRE(d.finalGaussRms <= d.targetGaussRms);
        REQUIRE(d.originalGaussRms <= d.targetGaussRms);
        REQUIRE(std::abs(d.energyIdentityError) <= d.roundoffEnergyAllowance);
    }
}
TEST_CASE("Electrostatic continuum potential and face field converge at second order",
          "[electrostatic][refinement]") {
    double priorPhi = 0, priorField = 0;
    for (std::size_t nx : {16, 32, 64, 128}) {
        ElectrostaticGridConfig c{nx, nx / 2, 1.0 / nx, 2.0 / nx, 2};
        PeriodicElectrostaticGrid grid(c);
        auto q = Mode(c, 1, 1, .7);
        for (double &x : q)
            x *= c.permittivity * 8 * Pi * Pi;
        grid.solve(q);
        const auto s = grid.getSnapshot();
        const auto exact = Mode(c, 1, 1, .7);
        double ep = 0, ef = 0;
        for (std::size_t j = 0; j < c.rows; ++j)
            for (std::size_t i = 0; i < c.columns; ++i) {
                const auto k = i + c.columns * j;
                ep += std::pow(s.potential[k] - exact[k], 2);
                const double ex =
                    1.4 * Pi * std::sin(2 * Pi * (i * c.spacingX + (j + .5) * c.spacingY) + .31);
                const double ey =
                    1.4 * Pi * std::sin(2 * Pi * ((i + .5) * c.spacingX + j * c.spacingY) + .31);
                ef += std::pow(s.field.xFaces[k] - ex, 2) + std::pow(s.field.yFaces[k] - ey, 2);
            }
        ep = std::sqrt(ep / q.size());
        ef = std::sqrt(ef / (2 * q.size()));
        if (priorPhi) {
            REQUIRE(priorPhi / ep > 3.9);
            REQUIRE(priorPhi / ep < 4.1);
            REQUIRE(priorField / ef > 3.9);
            REQUIRE(priorField / ef < 4.1);
        }
        priorPhi = ep;
        priorField = ef;
    }
}
TEST_CASE("Electrostatic neutrality correction is explicit narrow and original-source audited",
          "[electrostatic][neutrality]") {
    PeriodicElectrostaticGrid grid({2, 2, 1, 1, 1});
    const std::vector<double> q{1, -1 + 1e-14, 2, -2};
    const auto d = grid.solve(q);
    const auto s = grid.getSnapshot();
    REQUIRE(s.originalCharge == q);
    REQUIRE(d.originalChargeMean != 0);
    REQUIRE(std::abs(d.originalChargeMean) <= d.neutralityMeanAllowance);
    REQUIRE(d.maximumSourceCorrection > 0);
    REQUIRE(d.maximumSourceCorrection <= d.sourceCorrectionAllowance);
    REQUIRE(std::abs(d.removedChargeMean) <= 2 * d.neutralityMeanAllowance);
    Near(d.originalIntegratedCharge, (q[0] + q[1]) + (q[2] + q[3]), 2e-17);
    IndependentAudit(s, grid.getConfig());
    for (const auto bad :
         {std::vector<double>{1, 1, 1, 1}, std::vector<double>{1, -1 + 1e-8, 2, -2}}) {
        REQUIRE_THROWS_AS(grid.solve(bad), std::invalid_argument);
        Same(grid.getSnapshot(), s);
    }
    // Uniform subtraction cannot change the two huge entries at this precision;
    // the remaining stored mean must not be hidden by another internal projection.
    ElectrostaticSolveConfig strict;
    strict.absoluteGaussTolerance = 1e-12;
    strict.relativeGaussTolerance = 0;
    REQUIRE_THROWS_AS(grid.solve({1e100, 1, -1e100, 1}, strict), std::runtime_error);
    Same(grid.getSnapshot(), s);
}
TEST_CASE("Electrostatic zero source default snapshots replay and copies have owned lifetimes",
          "[electrostatic][ownership]") {
    PeriodicElectrostaticGrid defaults;
    REQUIRE(defaults.getConfig().columns == 16);
    const auto initial = defaults.getSnapshot();
    REQUIRE(initial.potential == std::vector<double>(256));
    REQUIRE_FALSE(initial.diagnostics.hasSolution);
    const auto d = defaults.solve(std::vector<double>(256));
    REQUIRE(d.hasSolution);
    REQUIRE(d.zeroSource);
    REQUIRE(d.iterations == 0);
    REQUIRE(d.cellVisits == 52 * 256);
    REQUIRE(d.fieldEnergy == 0);
    REQUIRE(d.sourceEnergy == 0);
    REQUIRE(d.finalGaussRms == 0);
    ElectrostaticGridConfig c{7, 5, .2, .3, 2};
    PeriodicElectrostaticGrid first(c), second(c);
    auto q = Charge(c, 1, 1, .6);
    first.solve(q);
    second.solve(q);
    Same(first.getSnapshot(), second.getSnapshot());
    auto copy = first.getSnapshot();
    const auto original = copy;
    copy.originalCharge[0] = 99;
    copy.effectiveCharge[0] = 99;
    copy.field.xFaces[0] = 99;
    copy.potential[0] = 99;
    copy.gaussResidual[0] = 99;
    copy.curl[0] = 99;
    copy.diagnostics.fieldEnergy = 99;
    q[0] = 99;
    Same(first.getSnapshot(), original);
    auto retained = [] {
        PeriodicElectrostaticGrid grid({2, 2, 1, 1, 1});
        grid.solve({1, -1, 1, -1});
        return grid.getSnapshot();
    }();
    REQUIRE(retained.potential.size() == 4);
    Near(retained.potential[0], .25, 1e-14);
    PeriodicElectrostaticGrid zero({2, 2, 1, 1, 1});
    ElectrostaticSolveConfig exactBudget;
    exactBudget.maximumIterations = 0;
    exactBudget.maximumCellVisits = 52 * 4;
    REQUIRE(zero.solve(std::vector<double>(4), exactBudget).cellVisits == 52 * 4);
    const auto zeroSnapshot = zero.getSnapshot();
    --exactBudget.maximumCellVisits;
    REQUIRE_THROWS(zero.solve(std::vector<double>(4), exactBudget));
    Same(zero.getSnapshot(), zeroSnapshot);
    ElectrostaticSolveConfig loose;
    loose.maximumIterations = 0;
    loose.absoluteGaussTolerance = 2;
    const auto looseResult = zero.solve({1, -1, 1, -1}, loose);
    REQUIRE(looseResult.iterations == 0);
    REQUIRE_FALSE(looseResult.zeroSource);
    REQUIRE(looseResult.hasSolution);
    Near(looseResult.finalGaussRms, 1, 0);
    REQUIRE(zero.getSnapshot().potential == std::vector<double>(4));
    IndependentAudit(zero.getSnapshot(), zero.getConfig());
}
TEST_CASE("Electrostatic work convergence data and range failures retain complete snapshot",
          "[electrostatic][validation]") {
    ElectrostaticGridConfig c{7, 5, .2, .3, 2};
    PeriodicElectrostaticGrid grid(c);
    const auto q = Charge(c, 1, 1, .6);
    grid.solve(q);
    const auto saved = grid.getSnapshot();
    ElectrostaticSolveConfig o;
    for (std::size_t work : {std::size_t(0), std::size_t(1), saved.diagnostics.cellVisits - 1}) {
        o.maximumCellVisits = work;
        REQUIRE_THROWS(grid.solve(q, o));
        Same(grid.getSnapshot(), saved);
    }
    o = {};
    o.maximumIterations = 0;
    REQUIRE_THROWS(grid.solve(q, o));
    Same(grid.getSnapshot(), saved);
    o = {};
    o.absoluteGaussTolerance = 0;
    o.relativeGaussTolerance = 0;
    REQUIRE_THROWS(grid.solve(q, o));
    Same(grid.getSnapshot(), saved);
    for (double invalid :
         {-1., std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()}) {
        o = {};
        o.absoluteGaussTolerance = invalid;
        REQUIRE_THROWS(grid.solve(q, o));
        Same(grid.getSnapshot(), saved);
        o = {};
        o.relativeGaussTolerance = invalid;
        REQUIRE_THROWS(grid.solve(q, o));
        Same(grid.getSnapshot(), saved);
    }
    o = {};
    o.maximumIterations = 1000001;
    REQUIRE_THROWS(grid.solve(q, o));
    o = {};
    o.maximumCellVisits = 1000000001;
    REQUIRE_THROWS(grid.solve(q, o));
    auto invalid = q;
    invalid[1] = std::numeric_limits<double>::quiet_NaN();
    REQUIRE_THROWS(grid.solve(invalid));
    Same(grid.getSnapshot(), saved);
    REQUIRE_THROWS(grid.solve({}));
    Same(grid.getSnapshot(), saved);
    // A loose requested residual cannot certify unrepresentable stored energy.
    for (double scale : {1e200, 1e-200}) {
        auto ranged = q;
        for (double &x : ranged)
            x *= scale;
        o = {};
        o.absoluteGaussTolerance = 0;
        o.relativeGaussTolerance = 1e-10;
        REQUIRE_THROWS(grid.solve(ranged, o));
        Same(grid.getSnapshot(), saved);
    }
    PeriodicElectrostaticGrid zero({2, 2, 1, 1, 1});
    o = {};
    o.maximumIterations = 0;
    o.maximumCellVisits = 0;
    REQUIRE_THROWS(zero.solve(std::vector<double>(4), o));
    REQUIRE_FALSE(zero.getSnapshot().diagnostics.hasSolution);
    for (auto bad :
         {ElectrostaticGridConfig{1, 2, 1, 1, 1}, ElectrostaticGridConfig{512, 513, 1, 1, 1},
          ElectrostaticGridConfig{2, 2, 0, 1, 1}, ElectrostaticGridConfig{2, 2, 1, 1, -1},
          ElectrostaticGridConfig{2, 2, 1e-200, 1e-200, 1},
          ElectrostaticGridConfig{2, 2, 1e200, 1e200, 1},
          ElectrostaticGridConfig{2, 2, 1e-150, 1e150, 1},
          ElectrostaticGridConfig{2, 2, 1, std::numeric_limits<double>::infinity(), 1}})
        REQUIRE_THROWS(PeriodicElectrostaticGrid{bad});
    PeriodicElectrostaticGrid largest({512, 512, 1, 1, 1});
    REQUIRE(largest.getSnapshot().potential.size() == 262144);
}

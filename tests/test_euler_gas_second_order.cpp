#include "euler_gas_oracles.h"
namespace {
void WorkAudit(const EulerGasSecondOrderDiagnostics &d, std::size_t n) {
    REQUIRE(d.attempts == d.substeps + d.rejectedAttempts);
    REQUIRE(d.reconstructionPreparations == 2 * d.attempts);
    REQUIRE(d.forwardEulerStages == d.attempts + d.substeps);
    REQUIRE(d.blendPasses == d.substeps);
    REQUIRE(d.reconstructionTrials >= 2 * d.attempts * n);
    REQUIRE(d.reconstructionTrials <= 34 * 2 * d.attempts * n);
    REQUIRE(d.cellVisits == (4 + 8 * d.attempts + 5 * d.substeps) * n + d.reconstructionTrials);
    REQUIRE(d.maximumCfl < .5);
}
double IndependentMC(double l, double r) {
    if (l * r <= 0)
        return 0;
    return std::copysign(std::min({2 * std::abs(l), .5 * std::abs(l + r), 2 * std::abs(r)}), l);
}
// Raw physical conserved reconstruction and fluxes: deliberately no production
// normalization, exponent products, transfer routine or positivity limiter.
EulerGasState PhysicalFE(const EulerGasState &s, const EulerGasGridConfig &g, double h) {
    std::vector<std::array<Cell, 4>> faces(s.density.size());
    double ax = 0, ay = 0;
    for (std::size_t j = 0; j < g.rows; ++j)
        for (std::size_t i = 0; i < g.columns; ++i) {
            const auto k = i + g.columns * j;
            const auto l = (i + g.columns - 1) % g.columns + g.columns * j,
                       r = (i + 1) % g.columns + g.columns * j;
            const auto b = i + g.columns * ((j + g.rows - 1) % g.rows),
                       t = i + g.columns * ((j + 1) % g.rows);
            for (std::size_t a = 0; a < 4; ++a) {
                const double center = At(s, k)[a];
                const double sx = IndependentMC(center - At(s, l)[a], At(s, r)[a] - center);
                const double sy = IndependentMC(center - At(s, b)[a], At(s, t)[a] - center);
                faces[k][0][a] = center - .5 * sx;
                faces[k][1][a] = center + .5 * sx;
                faces[k][2][a] = center - .5 * sy;
                faces[k][3][a] = center + .5 * sy;
            }
            for (std::size_t face = 0; face < 4; ++face) {
                const auto q = faces[k][face];
                const double u = q[1] / q[0], v = q[2] / q[0];
                const double p = (g.gamma - 1) * (q[3] - .5 * q[0] * (u * u + v * v));
                REQUIRE(q[0] > 0);
                REQUIRE(p > 0);
                const double c = std::sqrt(g.gamma * p / q[0]);
                if (face < 2)
                    ax = std::max(ax, std::abs(u) + c);
                else
                    ay = std::max(ay, std::abs(v) + c);
            }
        }
    auto result = s;
    for (std::size_t j = 0; j < g.rows; ++j)
        for (std::size_t i = 0; i < g.columns; ++i) {
            const auto k = i + g.columns * j, l = (i + g.columns - 1) % g.columns + g.columns * j;
            const auto r = (i + 1) % g.columns + g.columns * j,
                       b = i + g.columns * ((j + g.rows - 1) % g.rows),
                       t = i + g.columns * ((j + 1) % g.rows);
            const auto fl = Face(faces[l][1], faces[k][0], true, ax, g.gamma),
                       fr = Face(faces[k][1], faces[r][0], true, ax, g.gamma);
            const auto fb = Face(faces[b][3], faces[k][2], false, ay, g.gamma),
                       ft = Face(faces[k][3], faces[t][2], false, ay, g.gamma);
            auto q = At(s, k);
            for (std::size_t a = 0; a < 4; ++a)
                q[a] -= h / g.spacingX * (fr[a] - fl[a]) + h / g.spacingY * (ft[a] - fb[a]);
            Put(result, k, q);
        }
    return result;
}
EulerGasState PhysicalRK2(const EulerGasState &s, const EulerGasGridConfig &g, double h) {
    auto second = PhysicalFE(PhysicalFE(s, g, h), g, h);
    for (std::size_t k = 0; k < s.density.size(); ++k) {
        auto q = At(second, k);
        for (std::size_t a = 0; a < 4; ++a)
            q[a] = .5 * (At(s, k)[a] + q[a]);
        Put(second, k, q);
    }
    return second;
}
EulerGasState Shift(const EulerGasState &s, const EulerGasState &derivative, double h) {
    auto shifted = s;
    for (std::size_t k = 0; k < s.density.size(); ++k) {
        auto q = At(s, k);
        for (std::size_t a = 0; a < 4; ++a)
            q[a] += h * At(derivative, k)[a];
        Put(shifted, k, q);
    }
    return shifted;
}
EulerGasState Derivative(const EulerGasState &s, const EulerGasGridConfig &g) {
    auto derivative = PhysicalFE(s, g, 1);
    for (std::size_t k = 0; k < s.density.size(); ++k) {
        auto q = At(derivative, k);
        for (std::size_t a = 0; a < 4; ++a)
            q[a] -= At(s, k)[a];
        Put(derivative, k, q);
    }
    return derivative;
}
EulerGasState ReferenceRK4(EulerGasState s, const EulerGasGridConfig &g, double duration,
                           std::size_t steps) {
    const double h = duration / steps;
    for (std::size_t step = 0; step < steps; ++step) {
        const auto k1 = Derivative(s, g), k2 = Derivative(Shift(s, k1, .5 * h), g);
        const auto k3 = Derivative(Shift(s, k2, .5 * h), g), k4 = Derivative(Shift(s, k3, h), g);
        for (std::size_t k = 0; k < s.density.size(); ++k) {
            auto q = At(s, k);
            for (std::size_t a = 0; a < 4; ++a)
                q[a] += h / 6 * (At(k1, k)[a] + 2 * At(k2, k)[a] + 2 * At(k3, k)[a] + At(k4, k)[a]);
            Put(s, k, q);
        }
    }
    return s;
}
double StateError(const EulerGasState &a, const EulerGasState &b) {
    double sum = 0;
    for (std::size_t k = 0; k < a.density.size(); ++k)
        for (std::size_t c = 0; c < 4; ++c)
            sum += std::abs(At(a, k)[c] - At(b, k)[c]);
    return sum / a.density.size();
}
EulerGasState Rotate(const EulerGasState &s, std::size_t nx, std::size_t ny) {
    auto rotated = s;
    for (std::size_t j = 0; j < ny; ++j)
        for (std::size_t i = 0; i < nx; ++i) {
            auto q = At(s, i + nx * j);
            std::swap(q[1], q[2]);
            Put(rotated, j + ny * i, q);
        }
    return rotated;
}
std::vector<double> SecondValues(const EulerGasSecondOrderDiagnostics &d) {
    auto values = Values(d);
    for (auto count :
         {d.attempts, d.rejectedAttempts, d.reconstructionPreparations, d.reconstructionTrials,
          d.limitedSlopeCells, d.positivityLimitedCells, d.rangeLimitedCells,
          d.zeroSlopeFallbackCells, d.forwardEulerStages, d.blendPasses})
        values.push_back(double(count));
    values.push_back(d.minimumSlopeScale);
    values.push_back(d.maximumRejectedCfl);
    return values;
}
double CubicError(double h) {
    constexpr double b = 1e-7, u = 1, t = .4;
    EulerGasGridConfig g{1024, 2, .5, 1, 1.4};
    PeriodicEulerGasGrid grid(g);
    auto s = grid.state();
    for (std::size_t i = 0; i < g.columns; ++i) {
        const double x = (i + .5) * g.spacingX - 256;
        const double z = std::clamp(x, -100.0, 100.0) + 100;
        const double rho =
            1 + b * (z * z * z + (std::abs(x) < 100 ? .25 * g.spacingX * g.spacingX * z : 0));
        for (std::size_t j = 0; j < g.rows; ++j)
            Put(s, i + g.columns * j, Conserved(rho, u, 0, 1));
    }
    grid.setState(s);
    EulerGasSecondOrderConfig options;
    options.maxSubstep = h;
    const auto d = grid.stepSecondOrder(t, options);
    WorkAudit(d, s.density.size());
    REQUIRE(d.rejectedAttempts == 0);
    REQUIRE(d.positivityLimitedCells == 0);
    // A full RK2 stencil reaches at most four cells per accepted step.
    // The polynomial patch extends 200 cells each side of the observation.
    REQUIRE(4 * d.substeps < 190);
    const std::size_t i = 512;
    const double x = (i + .5) * g.spacingX - 256, z = x - u * t + 100;
    const double exact = 1 + b * (z * z * z + .25 * g.spacingX * g.spacingX * z);
    const double spatial = .5 * b * u * g.spacingX * g.spacingX * t;
    const double defect = grid.state().density[i] - exact - spatial;
    std::cout << "Euler cubic contact dt=" << h
              << ", continuum defect=" << grid.state().density[i] - exact
              << ", temporal defect=" << defect << '\n';
    Near(defect, b * u * u * u * t * h * h, 1e-14);
    return std::abs(defect);
}
// Independent symmetric two-rarefaction solution: u*=0 by symmetry,
// c*=cL+(gamma-1)*uL/2 from the left Riemann invariant.
Cell Expansion(double xi) {
    constexpr double gamma = 1.4, uL = -1, pL = .4;
    const double cL = std::sqrt(gamma * pL), cStar = cL + .5 * (gamma - 1) * uL;
    const double sign = xi > 0 ? -1 : 1;
    xi = -std::abs(xi);
    double u, c;
    if (xi < uL - cL) {
        u = uL;
        c = cL;
    } else if (xi < -cStar) {
        u = 2 / (gamma + 1) * (cL + .5 * (gamma - 1) * uL + xi);
        c = 2 / (gamma + 1) * (cL + .5 * (gamma - 1) * (uL - xi));
    } else {
        u = 0;
        c = cStar;
    }
    return Conserved(std::pow(c / cL, 2 / (gamma - 1)), sign * u, 0,
                     pL * std::pow(c / cL, 2 * gamma / (gamma - 1)));
}
std::pair<double, EulerGasState> ExpansionError(std::size_t nx, double length) {
    constexpr double duration = .1;
    EulerGasGridConfig g{nx, 2, length / nx, .5, 1.4};
    PeriodicEulerGasGrid grid(g);
    auto s = grid.state();
    for (std::size_t i = 0; i < nx; ++i) {
        const auto q = Conserved(1, i < nx / 2 ? -1 : 1, 0, .4);
        Put(s, i, q);
        Put(s, i + nx, q);
    }
    grid.setState(s);
    const auto d = grid.stepSecondOrder(duration);
    const auto actual = grid.state();
    WorkAudit(d, s.density.size());
    Audit(s, actual, d, g.spacingX * g.spacingY, true);
    double error = 0;
    const double cL = std::sqrt(.56), cStar = cL - .2;
    for (std::size_t i = 0; i < nx; ++i) {
        const double left = i * g.spacingX - length / 2, right = left + g.spacingX;
        if (std::abs(.5 * (left + right)) >= .3)
            continue;
        std::vector<double> cuts{left, right};
        for (double speed : {-1 - cL, -cStar, 0.0, cStar, 1 + cL})
            if (speed * duration > left && speed * duration < right)
                cuts.push_back(speed * duration);
        std::sort(cuts.begin(), cuts.end());
        Cell exact{};
        for (std::size_t piece = 1; piece < cuts.size(); ++piece) {
            const auto q = Average(cuts[piece - 1], cuts[piece],
                                   [duration](double x) { return Expansion(x / duration); });
            for (std::size_t a = 0; a < 4; ++a)
                exact[a] += q[a] * (cuts[piece] - cuts[piece - 1]) / g.spacingX;
        }
        for (std::size_t a = 0; a < 4; ++a)
            error += g.spacingX * std::abs(At(actual, i)[a] - exact[a]);
    }
    return {error, actual};
}
} // namespace
TEST_CASE("Second-order Euler independent double-rarefaction retains positive center",
          "[euler2][riemann]") {
    const auto a = ExpansionError(128, 4), b = ExpansionError(256, 4), c = ExpansionError(512, 4);
    const auto control = ExpansionError(512, 8);
    std::cout << "Euler MC/RK2 double-rarefaction L1: " << a.first << ", " << b.first << ", "
              << c.first << '\n';
    REQUIRE(b.first < .85 * a.first);
    REQUIRE(c.first < .85 * b.first);
    for (std::size_t i = 0; i < 256; ++i)
        if (std::abs((i + .5) * 4 / 256 - 2) < .3)
            for (std::size_t component = 0; component < 4; ++component)
                Near(At(b.second, i)[component], At(control.second, i + 128)[component], 2e-13);
}
TEST_CASE("Second-order Euler agrees with independent physical RK2 and axis rotation", "[euler2]") {
    for (const auto dims : {std::pair<std::size_t, std::size_t>{7, 5}, {2, 5}, {5, 2}, {2, 2}}) {
        EulerGasGridConfig g{dims.first, dims.second, .7, 1.3, 1.4};
        PeriodicEulerGasGrid grid(g);
        auto s = grid.state();
        for (std::size_t k = 0; k < s.density.size(); ++k)
            Put(s, k,
                Conserved(1 + .1 * std::sin(double(k)), .2 * std::cos(double(k)),
                          .15 * std::sin(2.0 * k), 1 + .1 * std::cos(3.0 * k)));
        grid.setState(s);
        const auto expected = PhysicalRK2(s, g, .001);
        const auto d = grid.stepSecondOrder(.001);
        const auto actual = grid.state();
        REQUIRE(d.substeps == 1);
        REQUIRE(d.positivityLimitedCells == 0);
        WorkAudit(d, s.density.size());
        Audit(s, actual, d, g.spacingX * g.spacingY, true);
        for (std::size_t k = 0; k < s.density.size(); ++k)
            for (std::size_t a = 0; a < 4; ++a)
                Near(At(actual, k)[a], At(expected, k)[a], 3e-15);
        PeriodicEulerGasGrid rotated({g.rows, g.columns, g.spacingY, g.spacingX, g.gamma});
        rotated.setState(Rotate(s, g.columns, g.rows));
        rotated.stepSecondOrder(.001);
        const auto rotatedExpected = Rotate(actual, g.columns, g.rows);
        for (std::size_t k = 0; k < s.density.size(); ++k)
            for (std::size_t a = 0; a < 4; ++a)
                Near(At(rotated.state(), k)[a], At(rotatedExpected, k)[a], 3e-15);
    }
}
TEST_CASE("Second-order Euler contact and nonlinear wave improve with spatial refinement",
          "[euler2][continuum]") {
    const auto a = Contact(32, true), b = Contact(64, true), c = Contact(128, true),
               d = Contact(256, true);
    std::cout << "Euler MC/RK2 contact RMS: " << a.error << ", " << b.error << ", " << c.error
              << ", " << d.error << '\n';
    REQUIRE(c.error / d.error > 3.2);
    REQUIRE(b.error / c.error > 3);
    REQUIRE(d.error < Contact(256).error / 10);
    for (const auto result : {a, b, c, d}) {
        REQUIRE(result.pressureError < 2e-12);
        REQUIRE(result.velocityError < 2e-12);
    }
    const double w32 = SimpleWaveError(32, true), w64 = SimpleWaveError(64, true),
                 w128 = SimpleWaveError(128, true), w256 = SimpleWaveError(256, true);
    std::cout << "Euler MC/RK2 nonlinear wave L1: " << w32 << ", " << w64 << ", " << w128 << ", "
              << w256 << '\n';
    REQUIRE(w64 / w128 > 3.2);
    REQUIRE(w128 / w256 > 3.2);
    REQUIRE(w256 < SimpleWaveError(256) / 5);
}
TEST_CASE("Second-order Euler contact has independently derived temporal and spatial defects",
          "[euler2][temporal]") {
    const double a = CubicError(.05), b = CubicError(.025), c = CubicError(.0125);
    REQUIRE(a / b > 3.9);
    REQUIRE(b / c > 3.9);
    REQUIRE(a / b < 4.1);
    REQUIRE(b / c < 4.1);
}
TEST_CASE("Second-order Euler nonlinear temporal order is separated from spatial error",
          "[euler2][temporal]") {
    EulerGasGridConfig g{64, 2, 1.0 / 64, .5, 1.4};
    PeriodicEulerGasGrid seed(g);
    auto s = seed.state();
    constexpr double duration = .01;
    for (std::size_t i = 0; i < g.columns; ++i) {
        const auto q = Average(i * g.spacingX, (i + 1) * g.spacingX,
                               [](double x) { return SimpleWave(x + .123, 0); });
        Put(s, i, q);
        Put(s, i + g.columns, q);
    }
    const auto reference = ReferenceRK4(s, g, duration, 1024);
    const auto refined = ReferenceRK4(s, g, duration, 2048);
    std::array<double, 3> errors{};
    for (std::size_t level = 0; level < errors.size(); ++level) {
        PeriodicEulerGasGrid grid(g);
        grid.setState(s);
        EulerGasSecondOrderConfig o;
        o.maxSubstep = .0025 / double(1u << level);
        const auto d = grid.stepSecondOrder(duration, o);
        REQUIRE(d.rejectedAttempts == 0);
        REQUIRE(d.positivityLimitedCells == 0);
        errors[level] = StateError(grid.state(), refined);
        WorkAudit(d, s.density.size());
    }
    double continuum = 0;
    for (std::size_t i = 0; i < g.columns; ++i) {
        const auto exact = Average(i * g.spacingX, (i + 1) * g.spacingX,
                                   [duration](double x) { return SimpleWave(x + .123, duration); });
        for (std::size_t a = 0; a < 4; ++a)
            continuum += std::abs(At(refined, i)[a] - exact[a]);
    }
    continuum /= g.columns;
    std::cout << "Euler nonlinear temporal L1: " << errors[0] << ", " << errors[1] << ", "
              << errors[2] << "; independently refined spatial L1=" << continuum << '\n';
    REQUIRE(StateError(reference, refined) < errors[2] / 100);
    REQUIRE(errors[0] / errors[1] > 3.5);
    REQUIRE(errors[1] / errors[2] > 3.5);
    REQUIRE(errors[0] / errors[1] < 4.5);
    REQUIRE(errors[1] / errors[2] < 4.5);
    REQUIRE(continuum > errors[0]); // A fixed-grid temporal test cannot certify continuum accuracy.
}
TEST_CASE("Second-order Euler exact Sod oracle improves without claiming shock second order",
          "[euler2][riemann]") {
    const auto a = Shock(128, 2, true), b = Shock(256, 2, true), c = Shock(512, 2, true),
               control = Shock(512, 4, true);
    std::cout << "Euler MC/RK2 Sod conserved L1: " << a.error << ", " << b.error << ", " << c.error
              << '\n';
    REQUIRE(b.error < a.error * .85);
    REQUIRE(c.error < b.error * .85);
    REQUIRE(c.error < Shock(512, 2).error * .7);
    for (std::size_t i = 0; i < 256; ++i)
        if (std::abs((i + .5) * 2 / 256 - 1) < .4)
            for (std::size_t component = 0; component < 4; ++component)
                Near(At(b.state, i)[component], At(control.state, i + 128)[component], 2e-13);
}
TEST_CASE("Second-order Euler uniform replay snapshots and historical observers", "[euler2]") {
    PeriodicEulerGasGrid a({4, 3, .5, .7, 1.4}), b(a.config());
    auto s = a.state();
    for (std::size_t k = 0; k < s.density.size(); ++k)
        Put(s, k, Conserved(2, -.7, .2, 3));
    a.setState(s);
    b.setState(s);
    EulerGasSecondOrderConfig o;
    o.maxSubstep = .01;
    o.maximumSubsteps = 1;
    o.maximumAttempts = 1;
    o.maximumCellVisits = 19 * s.density.size();
    const auto d = a.stepSecondOrder(.01, o);
    b.stepSecondOrder(.01, o);
    Same(a.state(), s);
    Same(a.state(), b.state());
    REQUIRE(SecondValues(d) == SecondValues(b.lastSecondOrderStep()));
    WorkAudit(d, s.density.size());
    REQUIRE(d.cellVisits == 19 * s.density.size());
    REQUIRE(d.limitedSlopeCells == 0);
    REQUIRE(d.reconstructionTrials == 2 * s.density.size());
    auto snapshot = a.state();
    snapshot.density[0] = 999;
    auto history = a.lastSecondOrderStep();
    a.setState(s);
    REQUIRE(SecondValues(a.lastSecondOrderStep()) == SecondValues(history));
    a.step(.01);
    REQUIRE(SecondValues(a.lastSecondOrderStep()) == SecondValues(history));
    REQUIRE(a.time() == .02);
    o.maximumSubsteps = 0;
    o.maximumAttempts = 0;
    o.maximumCellVisits = 3 * s.density.size();
    const auto zero = a.stepSecondOrder(0, o);
    Same(a.state(), s);
    REQUIRE(zero.cellVisits == 3 * s.density.size());
    REQUIRE(zero.zeroDurationNoOp);
    REQUIRE(zero.attempts == 0);
    REQUIRE(zero.minimumSlopeScale == 1);
    REQUIRE(a.time() == .02);
    REQUIRE(Values(zero) == Values(a.lastStep()));
    const auto saved = a.state();
    const auto savedD = a.lastSecondOrderStep();
    o.maximumCellVisits -= 1;
    REQUIRE_THROWS(a.stepSecondOrder(0, o));
    Same(a.state(), saved);
    REQUIRE(SecondValues(savedD) == SecondValues(a.lastSecondOrderStep()));
}
TEST_CASE("Second-order Euler positivity scaling cold compression and stage retry accounting",
          "[euler2]") {
    for (double gamma : {1.01, 1.4, 3.0, 20.0})
        for (double scale : {1e-150, 1.0, 1e150}) {
            EulerGasGridConfig g{8, 6, .4, .7, gamma};
            PeriodicEulerGasGrid grid(g);
            auto s = grid.state();
            for (std::size_t j = 0; j < g.rows; ++j)
                for (std::size_t i = 0; i < g.columns; ++i) {
                    const double x = 2 * Pi * (i + .5) / g.columns, y = 2 * Pi * (j + .5) / g.rows;
                    Put(s, i + g.columns * j,
                        Conserved(scale * (1 + .5 * std::cos(x + y)), .8 * std::sin(x),
                                  -.6 * std::cos(y), 1e-12 * scale, gamma));
                }
            grid.setState(s);
            EulerGasSecondOrderConfig o;
            o.cflSafety = .99;
            const auto d = grid.stepSecondOrder(.4, o);
            WorkAudit(d, s.density.size());
            Audit(s, grid.state(), d, g.spacingX * g.spacingY, true);
            REQUIRE(d.minimumSlopeScale < 1);
            REQUIRE(d.positivityLimitedCells > 0);
            if (gamma >= 1.4)
                REQUIRE(d.zeroSlopeFallbackCells > 0);
            std::cout << "Euler cold gamma=" << gamma << ", scale=" << scale
                      << ", retries=" << d.rejectedAttempts << ", trials=" << d.reconstructionTrials
                      << ", min theta=" << d.minimumSlopeScale << '\n';
        }
}

TEST_CASE("Second-order Euler stage-CFL retries consume bounded work transactionally", "[euler2]") {
    EulerGasGridConfig g{64, 2, 2.0 / 64, .5, 1.4};
    PeriodicEulerGasGrid grid(g);
    auto s = grid.state();
    for (std::size_t j = 0; j < g.rows; ++j)
        for (std::size_t i = 0; i < g.columns; ++i)
            Put(s, i + g.columns * j,
                i < g.columns / 2 ? Conserved(1, 0, 0, 1) : Conserved(.125, 0, 0, .1));
    grid.setState(s);
    EulerGasSecondOrderConfig o;
    o.cflSafety = .99;
    const auto d = grid.stepSecondOrder(.12, o);
    std::cout << "Euler stage retry attempts=" << d.attempts << ", accepted=" << d.substeps
              << ", rejected=" << d.rejectedAttempts << ", visits=" << d.cellVisits << '\n';
    REQUIRE(d.rejectedAttempts > 0);
    REQUIRE(d.maximumRejectedCfl > o.cflSafety / 2);
    WorkAudit(d, s.density.size());
    Audit(s, grid.state(), d, g.spacingX * g.spacingY, true);
    const auto expected = grid.state();
    PeriodicEulerGasGrid exact(g);
    exact.setState(s);
    o.maximumCellVisits = d.cellVisits;
    o.maximumAttempts = d.attempts;
    o.maximumSubsteps = d.substeps;
    const auto replay = exact.stepSecondOrder(.12, o);
    Same(expected, exact.state());
    REQUIRE(SecondValues(d) == SecondValues(replay));
    for (int budget = 0; budget < 4; ++budget) {
        auto failing = o;
        if (budget == 0)
            --failing.maximumCellVisits;
        if (budget == 1)
            --failing.maximumAttempts;
        if (budget == 2)
            --failing.maximumSubsteps;
        if (budget == 3)
            failing.maximumRetriesPerSubstep = 0;
        PeriodicEulerGasGrid reject(g);
        reject.setState(s);
        reject.stepSecondOrder(0);
        const auto before = reject.lastSecondOrderStep();
        REQUIRE_THROWS(reject.stepSecondOrder(.12, failing));
        Same(reject.state(), s);
        REQUIRE(reject.time() == 0);
        REQUIRE(SecondValues(before) == SecondValues(reject.lastSecondOrderStep()));
        REQUIRE(Values(before) == Values(reject.lastStep()));
    }
}

TEST_CASE("Second-order Euler invalid options and representability failures retain publication",
          "[euler2]") {
    PeriodicEulerGasGrid grid({4, 2, 1, 1, 1.4});
    grid.stepSecondOrder(.01);
    const auto before = grid.state();
    const auto history = grid.lastSecondOrderStep();
    for (double duration : {-1.0, std::numeric_limits<double>::quiet_NaN(),
                            std::numeric_limits<double>::infinity()}) {
        REQUIRE_THROWS(grid.stepSecondOrder(duration));
        Same(grid.state(), before);
        REQUIRE(SecondValues(history) == SecondValues(grid.lastSecondOrderStep()));
    }
    for (int setting = 0; setting < 7; ++setting) {
        EulerGasSecondOrderConfig o;
        if (setting == 0)
            o.cflSafety = 1;
        if (setting == 1)
            o.maxSubstep = 0;
        if (setting == 2)
            o.maximumAttempts = EulerGasSecondOrderConfig::MaximumAttempts + 1;
        if (setting == 3)
            o.maximumRetriesPerSubstep = 65;
        if (setting == 4)
            o.maximumCellVisits = 18 * before.density.size();
        if (setting == 5)
            o.maximumAttempts = 0;
        if (setting == 6)
            o.maximumSubsteps = 0;
        REQUIRE_THROWS(grid.stepSecondOrder(.1, o));
        Same(grid.state(), before);
        REQUIRE(SecondValues(history) == SecondValues(grid.lastSecondOrderStep()));
    }
    // Stored and integrated initial state is representable, but compression
    // requires an unrepresentable stored density. No clipping is permissible.
    PeriodicEulerGasGrid huge({4, 2, 1e-154, 1e-154, 1.4});
    auto s = huge.state();
    for (std::size_t k = 0; k < s.density.size(); ++k)
        Put(s, k, {{1.5e308, k % 4 < 2 ? .75e308 : -.75e308, 0, 1.7e308}});
    huge.setState(s);
    const auto hd = huge.lastSecondOrderStep();
    REQUIRE_THROWS(huge.stepSecondOrder(4e-155));
    Same(huge.state(), s);
    REQUIRE(huge.time() == 0);
    REQUIRE(SecondValues(hd) == SecondValues(huge.lastSecondOrderStep()));
    // Huge finite clock can advance once; stagnation/overflow still reject.
    PeriodicEulerGasGrid cold({2, 2, 1e154, 1e154, 1.4});
    s = cold.state();
    for (std::size_t k = 0; k < s.density.size(); ++k)
        Put(s, k, {{.1, 0, 0, 1e-320}});
    cold.setState(s);
    EulerGasSecondOrderConfig o;
    o.maxSubstep = 1e308;
    cold.stepSecondOrder(1e308, o);
    const auto cd = cold.lastSecondOrderStep();
    Same(cold.state(), s);
    REQUIRE_THROWS(cold.stepSecondOrder(1, o));
    REQUIRE_THROWS(cold.stepSecondOrder(1e308, o));
    REQUIRE(SecondValues(cd) == SecondValues(cold.lastSecondOrderStep()));
    REQUIRE(cold.time() == 1e308);
}

TEST_CASE("Second-order Euler limiter exposes extrema and admits the full supported gamma domain",
          "[euler2]") {
    // At a smooth density maximum both MC slopes are exactly zero in the
    // independent oracle. The method is intentionally locally first order.
    EulerGasGridConfig g{33, 2, 1.0 / 33, .5, 1.4};
    PeriodicEulerGasGrid grid(g);
    auto s = grid.state();
    for (std::size_t i = 0; i < g.columns; ++i) {
        const double density = 1 + .2 * std::cos(2 * Pi * double(i) / g.columns);
        Put(s, i, Conserved(density, .7, 0, 1));
        Put(s, i + g.columns, Conserved(density, .7, 0, 1));
    }
    REQUIRE(IndependentMC(s.density[0] - s.density[32], s.density[1] - s.density[0]) == 0);
    grid.setState(s);
    const auto expected = PhysicalRK2(s, g, .0001);
    const auto d = grid.stepSecondOrder(.0001);
    REQUIRE(d.limitedSlopeCells > 0);
    REQUIRE(d.positivityLimitedCells == 0);
    REQUIRE(grid.state().density[0] < s.density[0]);
    Near(grid.state().density[0], expected.density[0], 2e-15);
    for (double gamma : {std::nextafter(1.0, 2.0), 1.01, 3.0, 20.0, 1e150}) {
        PeriodicEulerGasGrid uniform({2, 2, .5, .7, gamma});
        auto state = uniform.state();
        const auto q = gamma > 1e100 ? Conserved(1, 1e-150, 0, 1e-150, gamma)
                                     : Conserved(1, .3, -.2, gamma - 1, gamma);
        for (std::size_t k = 0; k < state.density.size(); ++k)
            Put(state, k, q);
        uniform.setState(state);
        const auto result = uniform.stepSecondOrder(.001);
        Same(uniform.state(), state);
        REQUIRE(result.final.minimumPressure > 0);
        REQUIRE(result.positivityLimitedCells == 0);
        WorkAudit(result, state.density.size());
    }
}

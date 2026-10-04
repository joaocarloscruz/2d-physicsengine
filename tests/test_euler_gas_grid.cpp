#include "catch_amalgamated.hpp"
#include "physics/core/fluids/periodic_euler_gas_grid.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <numeric>
#include <tuple>
using namespace PhysicsEngine;
namespace {
constexpr double Pi = 3.14159265358979323846;
using Cell = std::array<double, 4>;
Cell Conserved(double r, double u, double v, double p, double gamma = 1.4) {
    return {{r, r * u, r * v, p / (gamma - 1) + .5 * r * (u * u + v * v)}};
}
Cell At(const EulerGasState &s, std::size_t k) {
    return {{s.density[k], s.momentumX[k], s.momentumY[k], s.totalEnergy[k]}};
}
void Put(EulerGasState &s, std::size_t k, const Cell &q) {
    s.density[k] = q[0];
    s.momentumX[k] = q[1];
    s.momentumY[k] = q[2];
    s.totalEnergy[k] = q[3];
}
void Near(double a, double b, double tolerance) {
    REQUIRE(a == Catch::Approx(b).epsilon(0).margin(tolerance));
}
std::vector<double> Values(const EulerGasSummary &s) {
    return {s.mass,           s.momentumX,         s.momentumY,
            s.totalEnergy,    s.absoluteMomentumX, s.absoluteMomentumY,
            s.internalEnergy, s.kineticEnergy,     s.minimumDensity,
            s.maximumDensity, s.minimumPressure,   s.maximumPressure};
}
std::vector<double> Values(const EulerGasDiagnostics &d) {
    auto values = Values(d.initial);
    const auto final = Values(d.final);
    values.insert(values.end(), final.begin(), final.end());
    const double rest[] = {d.massDefect,
                           d.momentumXDefect,
                           d.momentumYDefect,
                           d.totalEnergyDefect,
                           d.massRoundoffAllowance,
                           d.momentumXRoundoffAllowance,
                           d.momentumYRoundoffAllowance,
                           d.totalEnergyRoundoffAllowance,
                           d.duration,
                           d.timeBefore,
                           d.timeAfter,
                           d.lastSubstep,
                           d.maximumSignalSpeedX,
                           d.maximumSignalSpeedY,
                           d.maximumCfl,
                           double(d.substeps),
                           double(d.cellVisits),
                           double(d.zeroDurationNoOp)};
    values.insert(values.end(), std::begin(rest), std::end(rest));
    return values;
}
void Same(const EulerGasState &a, const EulerGasState &b) {
    REQUIRE(a.density == b.density);
    REQUIRE(a.momentumX == b.momentumX);
    REQUIRE(a.momentumY == b.momentumY);
    REQUIRE(a.totalEnergy == b.totalEnergy);
}
void Audit(const EulerGasState &before, const EulerGasState &after, const EulerGasDiagnostics &d,
           double area) {
    // Direct, independently associated sums of the stored conserved variables;
    // total energy is never reconstructed from primitive internal/kinetic data.
    Cell a{}, b{};
    for (std::size_t k = 0; k < before.density.size(); ++k)
        for (std::size_t c = 0; c < 4; ++c) {
            a[c] += At(before, k)[c] * area;
            b[c] += At(after, k)[c] * area;
        }
    const double initial[] = {d.initial.mass, d.initial.momentumX, d.initial.momentumY,
                              d.initial.totalEnergy};
    const double final[] = {d.final.mass, d.final.momentumX, d.final.momentumY,
                            d.final.totalEnergy};
    for (std::size_t c = 0; c < 4; ++c) {
        Near(initial[c], a[c], 2e-12 * std::max(1.0, std::abs(a[c])));
        Near(final[c], b[c], 2e-12 * std::max(1.0, std::abs(b[c])));
        Near(a[c], b[c], 3e-12 * std::max(1.0, std::abs(a[c])));
    }
    Near(d.totalEnergyDefect, d.final.totalEnergy - d.initial.totalEnergy, 0);
    REQUIRE(std::abs(d.totalEnergyDefect) <= d.totalEnergyRoundoffAllowance);
    REQUIRE(d.final.minimumDensity > 0);
    REQUIRE(d.final.minimumPressure > 0);
    REQUIRE(d.maximumCfl < 1);
    REQUIRE(d.cellVisits == (4 + 6 * d.substeps) * before.density.size());
}
Cell Flux(const Cell &q, bool x, double gamma) {
    const double u = q[1] / q[0], v = q[2] / q[0];
    const double p = (gamma - 1) * (q[3] - .5 * (q[1] * q[1] + q[2] * q[2]) / q[0]);
    return x ? Cell{{q[1], q[1] * u + p, q[2] * u, (q[3] + p) * u}}
             : Cell{{q[2], q[1] * v, q[2] * v + p, (q[3] + p) * v}};
}
Cell Face(const Cell &l, const Cell &r, bool x, double alpha, double gamma) {
    const auto fl = Flux(l, x, gamma), fr = Flux(r, x, gamma);
    Cell result{};
    for (std::size_t a = 0; a < 4; ++a)
        result[a] = .5 * (fl[a] + fr[a]) - .5 * alpha * (r[a] - l[a]);
    return result;
}
EulerGasState Oracle(const EulerGasState &old, const EulerGasGridConfig &g, double h) {
    double ax = 0, ay = 0;
    for (std::size_t k = 0; k < old.density.size(); ++k) {
        const auto q = At(old, k);
        const double u = q[1] / q[0], v = q[2] / q[0];
        const double p = (g.gamma - 1) * (q[3] - .5 * q[0] * (u * u + v * v));
        const double c = std::sqrt(g.gamma * p / q[0]);
        ax = std::max(ax, std::abs(u) + c);
        ay = std::max(ay, std::abs(v) + c);
    }
    auto result = old;
    for (std::size_t j = 0; j < g.rows; ++j)
        for (std::size_t i = 0; i < g.columns; ++i) {
            const auto k = i + g.columns * j, l = (i + g.columns - 1) % g.columns + g.columns * j;
            const auto r = (i + 1) % g.columns + g.columns * j;
            const auto b = i + g.columns * ((j + g.rows - 1) % g.rows),
                       t = i + g.columns * ((j + 1) % g.rows);
            const auto fl = Face(At(old, l), At(old, k), true, ax, g.gamma);
            const auto fr = Face(At(old, k), At(old, r), true, ax, g.gamma);
            const auto fb = Face(At(old, b), At(old, k), false, ay, g.gamma);
            const auto ft = Face(At(old, k), At(old, t), false, ay, g.gamma);
            Cell value = At(old, k);
            for (std::size_t a = 0; a < 4; ++a)
                value[a] -= h / g.spacingX * (fr[a] - fl[a]) + h / g.spacingY * (ft[a] - fb[a]);
            Put(result, k, value);
        }
    return result;
}
double Sinc(double x) {
    return x == 0 ? 1 : std::sin(x) / x;
}
struct ContactResult {
    double error, amplitude, pressureError, velocityError;
};
ContactResult Contact(std::size_t nx) {
    EulerGasGridConfig g{nx, nx / 2, 1.0 / nx, 2.0 / nx, 1.4};
    PeriodicEulerGasGrid grid(g);
    auto s = grid.state();
    constexpr double u = .7, v = -.2, duration = .15;
    const double factor = Sinc(Pi * g.spacingX) * Sinc(2 * Pi * g.spacingY);
    for (std::size_t j = 0; j < g.rows; ++j)
        for (std::size_t i = 0; i < nx; ++i) {
            const double phase = 2 * Pi * ((i + .5) * g.spacingX + 2 * (j + .5) * g.spacingY);
            Put(s, i + nx * j, Conserved(1 + .2 * factor * std::cos(phase), u, v, 1));
        }
    grid.setState(s);
    const auto d = grid.step(duration);
    const auto actual = grid.state();
    const auto p = grid.primitives();
    Audit(s, actual, d, g.spacingX * g.spacingY);
    double error = 0, amplitude = 0, pe = 0, ve = 0;
    for (std::size_t j = 0; j < g.rows; ++j)
        for (std::size_t i = 0; i < nx; ++i) {
            const auto k = i + nx * j;
            const double phase =
                2 * Pi *
                ((i + .5) * g.spacingX + 2 * (j + .5) * g.spacingY - (u + 2 * v) * duration);
            const double exact = 1 + .2 * factor * std::cos(phase);
            error += std::pow(actual.density[k] - exact, 2);
            amplitude += 2 * (actual.density[k] - 1) * std::cos(phase);
            pe = std::max(pe, std::abs(p.pressure[k] - 1));
            ve = std::max({ve, std::abs(p.velocityX[k] - u), std::abs(p.velocityY[k] - v)});
        }
    return {std::sqrt(error / s.density.size()), amplitude / s.density.size(), pe, ve};
}
// Independent smooth nonlinear right-moving simple wave, before gradient
// catastrophe. u+2c/(gamma-1) varies, u-2c/(gamma-1) is fixed, p=rho^gamma.
Cell SimpleWave(double x, double t) {
    constexpr double gamma = 1.4, amplitude = .04;
    const double c0 = std::sqrt(gamma);
    double lo = -amplitude, hi = amplitude;
    for (int iteration = 0; iteration < 70; ++iteration) {
        const double u = .5 * (lo + hi);
        const double f = u - amplitude * std::sin(2 * Pi * (x - (c0 + .5 * (gamma + 1) * u) * t));
        if (f > 0)
            hi = u;
        else
            lo = u;
    }
    const double u = .5 * (lo + hi), c = c0 + .5 * (gamma - 1) * u;
    const double rho = std::pow(c / c0, 2 / (gamma - 1));
    return Conserved(rho, u, 0, std::pow(rho, gamma), gamma);
}
// Four-point Gauss-Legendre cell averages. This oracle integrates conserved
// variables, rather than treating center samples as finite-volume averages.
template <class Function> Cell Average(double left, double right, Function function) {
    const double points[] = {-.8611363115940525752, -.3399810435848562648, .3399810435848562648,
                             .8611363115940525752};
    const double weights[] = {.3478548451374538574, .6521451548625461426, .6521451548625461426,
                              .3478548451374538574};
    Cell value{};
    for (int i = 0; i < 4; ++i) {
        const auto q = function(.5 * (left + right) + .5 * (right - left) * points[i]);
        for (std::size_t a = 0; a < 4; ++a)
            value[a] += .5 * weights[i] * q[a];
    }
    return value;
}
double SimpleWaveError(std::size_t nx) {
    EulerGasGridConfig g{nx, 2, 1.0 / nx, .5, 1.4};
    PeriodicEulerGasGrid grid(g);
    auto s = grid.state();
    constexpr double duration = .2;
    for (std::size_t i = 0; i < nx; ++i) {
        const auto q = Average(i * g.spacingX, (i + 1) * g.spacingX,
                               [](double x) { return SimpleWave(x, 0); });
        Put(s, i, q);
        Put(s, i + nx, q);
    }
    grid.setState(s);
    const auto d = grid.step(duration);
    const auto actual = grid.state();
    Audit(s, actual, d, g.spacingX * g.spacingY);
    double error = 0;
    for (std::size_t i = 0; i < nx; ++i) {
        const auto exact = Average(i * g.spacingX, (i + 1) * g.spacingX,
                                   [](double x) { return SimpleWave(x, duration); });
        for (std::size_t a = 0; a < 4; ++a)
            error += std::abs(At(actual, i)[a] - exact[a]);
    }
    return error / nx;
}
// Exact Sod solution from independently derived shock/isentropic-rarefaction
// pressure matching. No production flux, wave-speed or iteration code is used.
struct Sod {
    const double gamma = 1.4, rhoL = 1, pL = 1, rhoR = .125, pR = .1;
    double pStar = 0, uStar = 0, rhoStarL = 0, rhoStarR = 0, head = 0, tail = 0, shock = 0;
    double Curve(double p, double rho, double ps) const {
        if (p > ps)
            return (p - ps) *
                   std::sqrt(2 / ((gamma + 1) * rho) / (p + (gamma - 1) / (gamma + 1) * ps));
        return 2 * std::sqrt(gamma * ps / rho) / (gamma - 1) *
               (std::pow(p / ps, (gamma - 1) / (2 * gamma)) - 1);
    }
    Sod() {
        double lo = pR, hi = pL;
        for (int i = 0; i < 80; ++i) {
            const double p = .5 * (lo + hi);
            if (Curve(p, rhoL, pL) + Curve(p, rhoR, pR) > 0)
                hi = p;
            else
                lo = p;
        }
        pStar = .5 * (lo + hi);
        uStar = .5 * (Curve(pStar, rhoR, pR) - Curve(pStar, rhoL, pL));
        const double beta = (gamma - 1) / (gamma + 1);
        rhoStarL = rhoL * std::pow(pStar / pL, 1 / gamma);
        rhoStarR = rhoR * ((pStar / pR + beta) / (beta * pStar / pR + 1));
        head = -std::sqrt(gamma * pL / rhoL);
        tail = uStar - std::sqrt(gamma * pStar / rhoStarL);
        shock = std::sqrt(gamma * pR / rhoR) *
                std::sqrt((gamma + 1) / (2 * gamma) * pStar / pR + (gamma - 1) / (2 * gamma));
    }
    Cell sample(double xi) const {
        if (xi < head)
            return Conserved(rhoL, 0, 0, pL);
        if (xi < tail) {
            const double cL = -head, u = 2 / (gamma + 1) * (cL + xi);
            const double c = 2 / (gamma + 1) * (cL - .5 * (gamma - 1) * xi);
            return Conserved(rhoL * std::pow(c / cL, 2 / (gamma - 1)), u, 0,
                             pL * std::pow(c / cL, 2 * gamma / (gamma - 1)));
        }
        if (xi < uStar)
            return Conserved(rhoStarL, uStar, 0, pStar);
        if (xi < shock)
            return Conserved(rhoStarR, uStar, 0, pStar);
        return Conserved(rhoR, 0, 0, pR);
    }
    Cell average(double left, double right, double t) const {
        std::vector<double> cuts{left, right};
        for (double speed : {head, tail, uStar, shock})
            if (speed * t > left && speed * t < right)
                cuts.push_back(speed * t);
        std::sort(cuts.begin(), cuts.end());
        Cell q{};
        for (std::size_t i = 1; i < cuts.size(); ++i) {
            const auto a = Average(cuts[i - 1], cuts[i], [&](double x) { return sample(x / t); });
            for (std::size_t c = 0; c < 4; ++c)
                q[c] += a[c] * (cuts[i] - cuts[i - 1]) / (right - left);
        }
        return q;
    }
};
struct ShockResult {
    double error;
    EulerGasState state;
    EulerGasDiagnostics diagnostics;
};
ShockResult Shock(std::size_t nx, double length) {
    const double dx = length / nx, center = length / 2, t = .12;
    EulerGasGridConfig g{nx, 2, dx, .5, 1.4};
    PeriodicEulerGasGrid grid(g);
    auto s = grid.state();
    const Sod sod;
    for (std::size_t i = 0; i < nx; ++i) {
        const auto q = (i + .5) * dx < center ? Conserved(1, 0, 0, 1) : Conserved(.125, 0, 0, .1);
        Put(s, i, q);
        Put(s, i + nx, q);
    }
    grid.setState(s);
    const auto d = grid.step(t);
    const auto actual = grid.state();
    Audit(s, actual, d, dx * .5);
    double error = 0;
    for (std::size_t i = 0; i < nx; ++i)
        if (std::abs((i + .5) * dx - center) < .4) {
            const auto exact = sod.average(i * dx - center, (i + 1) * dx - center, t);
            for (std::size_t a = 0; a < 4; ++a)
                error += dx * std::abs(At(actual, i)[a] - exact[a]);
        }
    return {error, actual, d};
}
} // namespace

TEST_CASE("Euler gas one-step flux agrees with independent unsplit face oracle", "[euler]") {
    for (const auto dims : {std::pair<std::size_t, std::size_t>{7, 5}, {2, 5}, {5, 2}, {2, 2}}) {
        EulerGasGridConfig g{dims.first, dims.second, .7, 1.3, 1.4};
        PeriodicEulerGasGrid grid(g);
        auto s = grid.state();
        for (std::size_t k = 0; k < s.density.size(); ++k)
            Put(s, k,
                Conserved(.9 + .15 * std::sin(double(k)), .3 * std::cos(double(k)),
                          -.2 * std::sin(2.0 * k), 1 + .1 * std::cos(3.0 * k)));
        grid.setState(s);
        const auto expected = Oracle(s, g, .001);
        const auto d = grid.step(.001);
        const auto actual = grid.state();
        REQUIRE(d.substeps == 1);
        Audit(s, actual, d, g.spacingX * g.spacingY);
        for (std::size_t k = 0; k < s.density.size(); ++k)
            for (std::size_t a = 0; a < 4; ++a)
                Near(At(actual, k)[a], At(expected, k)[a], 2e-15);
    }
}
TEST_CASE("Euler gas uniform mode, owned snapshots and replay are exact", "[euler]") {
    EulerGasGridConfig g{4, 3, .5, .7, 1.4};
    PeriodicEulerGasGrid a(g), b(g);
    auto s = a.state();
    for (std::size_t k = 0; k < s.density.size(); ++k)
        Put(s, k, Conserved(2, -.7, .2, 3));
    a.setState(s);
    b.setState(s);
    auto copy = s;
    s.density[0] = 999;
    auto cfg = a.config();
    cfg.gamma = 20;
    auto primitives = a.primitives();
    primitives.pressure[0] = 999;
    EulerGasStepConfig options;
    options.maxSubstep = .01;
    options.maximumSubsteps = 1;
    options.maximumCellVisits = 10 * copy.density.size();
    const auto d = a.step(.01, options);
    b.step(.01, options);
    Same(a.state(), copy);
    Same(a.state(), b.state());
    REQUIRE(Values(d) == Values(b.lastStep()));
    REQUIRE(d.substeps == 1);
    Audit(copy, a.state(), d, g.spacingX * g.spacingY);
    auto retained = a.state();
    auto retainedD = a.lastStep();
    a.setState(copy);
    a.step(.02);
    Same(retained, copy);
    REQUIRE(Values(retainedD) == Values(d));
    REQUIRE(a.time() == .03);
    const auto before = a.state();
    const auto zero = a.step(0, EulerGasStepConfig{.9, .1, 0, 3 * copy.density.size()});
    Same(before, a.state());
    REQUIRE(zero.zeroDurationNoOp);
    REQUIRE(zero.cellVisits == 3 * copy.density.size());
    REQUIRE(zero.substeps == 0);
    REQUIRE(zero.lastSubstep == 0);
    REQUIRE(zero.timeBefore == zero.timeAfter);
}
TEST_CASE("Euler gas periodic moving contact converges with measured first-order diffusion",
          "[euler][continuum]") {
    const auto a = Contact(32), b = Contact(64), c = Contact(128), d = Contact(256);
    std::cout << "Euler contact RMS: " << a.error << ", " << b.error << ", " << c.error << ", "
              << d.error << '\n';
    REQUIRE(b.error < .7 * a.error);
    REQUIRE(c.error < .65 * b.error);
    REQUIRE(b.error / c.error > 1.65);
    REQUIRE(d.error < .6 * c.error);
    REQUIRE(c.error / d.error > 1.8);
    REQUIRE(a.amplitude < b.amplitude);
    REQUIRE(b.amplitude < c.amplitude);
    REQUIRE(c.amplitude < .2);
    REQUIRE(c.amplitude < d.amplitude);
    REQUIRE(d.amplitude < .2);
    for (const auto r : {a, b, c, d}) {
        REQUIRE(r.pressureError < 2e-13);
        REQUIRE(r.velocityError < 2e-13);
    }
}
TEST_CASE("Euler gas smooth nonlinear acoustic simple wave converges before breaking",
          "[euler][continuum]") {
    const double a = SimpleWaveError(32), b = SimpleWaveError(64), c = SimpleWaveError(128),
                 d = SimpleWaveError(256);
    std::cout << "Euler nonlinear simple-wave L1: " << a << ", " << b << ", " << c << ", " << d
              << '\n';
    REQUIRE(b < .7 * a);
    REQUIRE(c < .65 * b);
    REQUIRE(d < .6 * c);
    REQUIRE(c / d > 1.7);
}
TEST_CASE(
    "Euler gas shock tube matches independent exact Riemann solution and isolated-image control",
    "[euler][continuum]") {
    const Sod sod;
    Near(sod.pStar, .303130178050647, 2e-15);
    Near(sod.uStar, .92745262004895, 3e-15);
    const auto a = Shock(128, 2), b = Shock(256, 2), c = Shock(512, 2);
    std::cout << "Euler Sod conserved L1 window: " << a.error << ", " << b.error << ", " << c.error
              << '\n';
    REQUIRE(b.error < .8 * a.error);
    REQUIRE(c.error < .8 * b.error);
    REQUIRE(c.error < .08);
    // Same dx and central states, twice the separation from periodic images.
    // Global speeds can differ slightly as distant waves evolve. Compare stored
    // conservative fields in the oracle window, relative to discretization error.
    const auto control = Shock(512, 4);
    double imageDifference = 0;
    for (std::size_t i = 0; i < 256; ++i)
        if (std::abs((i + .5) * 2 / 256 - 1) < .4)
            for (std::size_t q = 0; q < 4; ++q)
                imageDifference +=
                    2.0 / 256 * std::abs(At(b.state, i)[q] - At(control.state, i + 128)[q]);
    std::cout << "Euler doubled-domain image difference: " << imageDifference << '\n';
    REQUIRE(imageDifference < 1e-5 * b.error);
}
TEST_CASE("Euler gas positivity split proof supports gamma above two and near one",
          "[euler][positivity]") {
    for (double gamma : {std::nextafter(1.0, 2.0), 1.01, 1.4, 3.0, 20.0, 1e150}) {
        EulerGasGridConfig g{4, 2, .3, .7, gamma};
        PeriodicEulerGasGrid grid(g);
        auto s = grid.state();
        for (std::size_t k = 0; k < s.density.size(); ++k) {
            const double rho = 1 + .1 * double(k),
                         pressure = gamma > 1e100 ? 1e-150 : .3 + .01 * double(k);
            // Very large gamma requires small velocities so positive internal
            // energy survives the unavoidable stored E-K subtraction.
            const double speed = gamma > 1e100 ? 1e-150 : .02;
            Put(s, k,
                Conserved(rho, speed * std::sin(double(k)), speed * std::cos(double(k)), pressure,
                          gamma));
        }
        grid.setState(s);
        const auto before = grid.state();
        const auto d = grid.step(.001);
        Audit(before, grid.state(), d, .21);
        const auto q = grid.primitives();
        for (std::size_t k = 0; k < s.density.size(); ++k) {
            REQUIRE(q.pressure[k] > 0);
            REQUIRE(q.internalEnergy[k] > 0);
        }
    }
    // Check the LF split algebra independently on ordinary states, including
    // gamma > 2. Neither a production limiter nor an arbitrary gamma ceiling.
    for (double gamma : {1.01, 1.4, 3.0, 20.0})
        for (double u : {-20., -.1, 0., .1, 20.}) {
            const double rho = .7, pressure = 2,
                         alpha = std::abs(u) + std::sqrt(gamma * pressure / rho);
            const auto q = Conserved(rho, u, -.3, pressure, gamma), f = Flux(q, true, gamma);
            for (double sign : {-1., 1.}) {
                Cell split{};
                for (std::size_t a = 0; a < 4; ++a)
                    split[a] = q[a] + sign * f[a] / alpha;
                REQUIRE(split[0] > 0);
                REQUIRE(split[3] - .5 * (split[1] * split[1] + split[2] * split[2]) / split[0] > 0);
            }
        }
}
TEST_CASE("Euler gas finite scales, cold subnormal mode and clock range", "[euler][range]") {
    for (double scale : {1e-150, 1e150, 1e300}) {
        EulerGasGridConfig g{3, 2, .1, .1, 1.4};
        PeriodicEulerGasGrid grid(g);
        auto s = grid.state();
        for (std::size_t k = 0; k < s.density.size(); ++k)
            Put(s, k, Conserved(scale, .1, -.2, scale));
        grid.setState(s);
        const auto d = grid.step(.001);
        Same(grid.state(), s);
        REQUIRE(d.final.minimumPressure > 0);
        Near(d.totalEnergyDefect, 0, 0);
        REQUIRE(d.totalEnergyRoundoffAllowance < d.initial.totalEnergy * 1e-10);
    }
    EulerGasGridConfig g{2, 2, 1e154, 1e154, 1.4};
    PeriodicEulerGasGrid cold(g);
    auto s = cold.state();
    std::fill(s.density.begin(), s.density.end(), .1);
    std::fill(s.totalEnergy.begin(), s.totalEnergy.end(), 1e-320);
    cold.setState(s);
    EulerGasStepConfig options;
    options.maxSubstep = 1e308;
    options.maximumSubsteps = 1;
    const auto d = cold.step(1e308, options);
    Same(cold.state(), s);
    REQUIRE(d.substeps == 1);
    REQUIRE(cold.primitives().pressure[0] > 0);
    REQUIRE(d.final.totalEnergy > 0);
    REQUIRE(cold.time() == 1e308);
    for (double duration : {1., 1e308}) {
        REQUIRE_THROWS(cold.step(duration, options));
        Same(cold.state(), s);
        REQUIRE(Values(cold.lastStep()) == Values(d));
        REQUIRE(cold.time() == 1e308);
    }
}
TEST_CASE("Euler gas cold nonuniform compressions remain strictly admissible without floors",
          "[euler][positivity]") {
    for (double scale : {1e-150, 1., 1e150}) {
        EulerGasGridConfig g{12, 8, .1, .17, 1.4};
        PeriodicEulerGasGrid grid(g);
        auto s = grid.state();
        for (std::size_t j = 0; j < g.rows; ++j)
            for (std::size_t i = 0; i < g.columns; ++i) {
                const double x = 2 * Pi * (i + .5) / g.columns;
                const double y = 2 * Pi * (j + .5) / g.rows;
                Put(s, i + g.columns * j,
                    Conserved(scale * (1 + .4 * std::cos(x + y)), .3 * std::sin(x),
                              -.2 * std::cos(y), 1e-12 * scale));
            }
        grid.setState(s);
        const auto d = grid.step(.2);
        const auto actual = grid.state();
        const auto q = grid.primitives();
        REQUIRE(d.substeps > 1);
        for (std::size_t k = 0; k < s.density.size(); ++k) {
            REQUIRE(actual.density[k] > 0);
            REQUIRE(q.pressure[k] > 0);
            REQUIRE(q.internalEnergy[k] > 0);
        }
        REQUIRE(std::abs(d.massDefect) <= d.massRoundoffAllowance);
        REQUIRE(std::abs(d.totalEnergyDefect) <= d.totalEnergyRoundoffAllowance);
        REQUIRE(d.maximumCfl <= .9);
    }
}
TEST_CASE("Euler gas rejects shape, admissibility and geometry errors transactionally",
          "[euler][validation]") {
    for (const auto g : {EulerGasGridConfig{1, 2, 1, 1, 1.4},
                         {2, 1, 1, 1, 1.4},
                         {EulerGasGridConfig::MaximumCells, 2, 1, 1, 1.4},
                         {std::numeric_limits<std::size_t>::max(), 2, 1, 1, 1.4},
                         {2, 2, 0, 1, 1.4},
                         {2, 2, 1, -1, 1.4},
                         {2, 2, 1e308, 1, 1.4},
                         {2, 2, 1e-300, 1e-300, 1.4},
                         {2, 2, 1, 1, 1},
                         {2, 2, 1, 1, std::numeric_limits<double>::infinity()},
                         {2, 2, std::numeric_limits<double>::quiet_NaN(), 1, 1.4},
                         {2, 2, 1, 1, std::numeric_limits<double>::quiet_NaN()}})
        REQUIRE_THROWS(PeriodicEulerGasGrid(g));
    PeriodicEulerGasGrid grid({4, 2, .3, .7, 1.4});
    grid.step(.001);
    const auto before = grid.state();
    const auto d = grid.lastStep();
    for (int variant = 0; variant < 10; ++variant) {
        auto s = before;
        if (variant == 0)
            s.density.pop_back();
        if (variant == 1)
            s.momentumX.pop_back();
        if (variant == 2)
            s.momentumY.pop_back();
        if (variant == 3)
            s.totalEnergy.pop_back();
        if (variant == 4)
            s.density[0] = 0;
        if (variant == 5)
            s.totalEnergy[0] = 0;
        if (variant == 6)
            s.momentumX[0] = std::numeric_limits<double>::quiet_NaN();
        if (variant == 7)
            s.momentumY[0] = std::numeric_limits<double>::infinity();
        if (variant == 8) {
            s.momentumX[0] = 2;
            s.totalEnergy[0] = 2;
        } // Rounded E-K=0.
        if (variant == 9) {
            s.density[0] = 1e-320;
            s.totalEnergy[0] = 1e308;
        } // Unrepresentable sound speed.
        REQUIRE_THROWS(grid.setState(s));
        Same(grid.state(), before);
        REQUIRE(Values(grid.lastStep()) == Values(d));
        REQUIRE(grid.time() == .001);
    }
}
TEST_CASE("Euler gas budget, arithmetic and late staged failures retain the complete snapshot",
          "[euler][rollback]") {
    PeriodicEulerGasGrid grid({4, 2, .1, .2, 1.4});
    auto start = grid.state();
    for (std::size_t k = 0; k < start.density.size(); ++k)
        Put(start, k, Conserved(1 + .1 * std::sin(double(k)), .2, -.1, 1));
    grid.setState(start);
    grid.step(.001);
    const auto before = grid.state();
    const auto prior = grid.lastStep();
    auto retains = [&] {
        Same(grid.state(), before);
        REQUIRE(Values(grid.lastStep()) == Values(prior));
        REQUIRE(grid.time() == .001);
    };
    for (double duration :
         {-1., std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()}) {
        REQUIRE_THROWS(grid.step(duration));
        retains();
    }
    for (const auto options : {EulerGasStepConfig{0, .1, 10, 1000},
                               {1, .1, 10, 1000},
                               {.9, 0, 10, 1000},
                               {.9, .1, 0, 1000},
                               {.9, .1, 1, 1000},
                               {.9, .1, 100, 79},
                               {.9, .001, 100, 80},
                               {.9, .1, EulerGasStepConfig::MaximumSubsteps + 1, 1000},
                               {.9, .1, 10, EulerGasStepConfig::MaximumCellVisits + 1}}) {
        REQUIRE_THROWS(grid.step(.2, options));
        retains();
    }
    REQUIRE_THROWS(grid.step(0, EulerGasStepConfig{.9, .1, 0, 23}));
    retains();
    // Counts right at the hard cell cap remain valid before allocation.
    EulerGasGridConfig{512, 512, 1, 1, 1.4}.Validate();
    // All supplied fields are individually finite/admissible. Compression makes
    // the next density exceed float64 only during the staged face update.
    PeriodicEulerGasGrid large({4, 2, 1e-154, 1e-154, 1.4});
    auto s = large.state();
    for (std::size_t k = 0; k < s.density.size(); ++k) {
        s.density[k] = 1.5e308;
        s.totalEnergy[k] = 1.7e308;
        s.momentumX[k] = (k % 4 < 2 ? 1 : -1) * .75e308;
    }
    large.setState(s);
    const auto d = large.step(0);
    REQUIRE_THROWS_AS(large.step(4e-155), std::overflow_error);
    Same(large.state(), s);
    REQUIRE(Values(large.lastStep()) == Values(d));
    REQUIRE(large.time() == 0);
    // Rate overflow is checked on step, including zero-duration observations.
    PeriodicEulerGasGrid tiny({2, 2, 1e-308, 1, 1.4});
    auto q = tiny.state();
    for (std::size_t k = 0; k < q.density.size(); ++k)
        Put(q, k, Conserved(1, 100, 0, 1));
    tiny.setState(q);
    REQUIRE_THROWS(tiny.step(0));
    Same(tiny.state(), q);
    REQUIRE(tiny.lastStep().cellVisits == 0);
}
TEST_CASE("Euler gas strict rounded CFL and representable scaled fluxes", "[euler][range]") {
    PeriodicEulerGasGrid grid({2, 2, .1, .2, 1.4});
    EulerGasStepConfig options;
    options.maximumSubsteps = 1;
    // The independent real-valued CFL endpoint exceeds the rounded-down native
    // limit by a few ulps. A one-substep budget must not silently admit it.
    const double endpoint = options.cflSafety / (std::sqrt(1.4) * (10 + 5));
    REQUIRE_THROWS(grid.step(endpoint, options));
    REQUIRE(grid.time() == 0);
    double inside = endpoint;
    for (int i = 0; i < 20; ++i)
        inside = std::nextafter(inside, 0.0);
    const auto d = grid.step(inside, options);
    REQUIRE(d.substeps == 1);
    REQUIRE(d.maximumCfl <= options.cflSafety);
    // Raw E+p and (E+p)*u exceed float64. The complete scaled transfer and
    // stored uniform state are representable, so neither intermediate is formed.
    PeriodicEulerGasGrid huge({3, 2, .1, .1, 1.4});
    auto s = huge.state();
    for (std::size_t k = 0; k < s.density.size(); ++k) {
        s.density[k] = 1e307;
        s.momentumX[k] = 2e307;
        s.totalEnergy[k] = 1.7e308;
    }
    huge.setState(s);
    const auto hd = huge.step(.001);
    Same(huge.state(), s);
    REQUIRE(hd.totalEnergyDefect == 0);
    REQUIRE(hd.final.minimumPressure > 0);
}

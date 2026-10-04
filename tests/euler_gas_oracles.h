#pragma once
// Independent physical oracles shared by both integration modes.
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
           double area, bool secondOrder = false) {
    // Direct, independently associated sums of the stored conserved variables;
    // total energy is never reconstructed from primitive internal/kinetic data.
    Cell a{}, b{}, absolute{};
    for (std::size_t k = 0; k < before.density.size(); ++k)
        for (std::size_t c = 0; c < 4; ++c) {
            a[c] += At(before, k)[c] * area;
            b[c] += At(after, k)[c] * area;
            absolute[c] += std::abs(At(before, k)[c] * area) + std::abs(At(after, k)[c] * area);
        }
    const double initial[] = {d.initial.mass, d.initial.momentumX, d.initial.momentumY,
                              d.initial.totalEnergy};
    const double final[] = {d.final.mass, d.final.momentumX, d.final.momentumY,
                            d.final.totalEnergy};
    for (std::size_t c = 0; c < 4; ++c) {
        Near(initial[c], a[c], 2e-12 * (secondOrder ? absolute[c] : std::max(1.0, std::abs(a[c]))));
        Near(final[c], b[c], 2e-12 * (secondOrder ? absolute[c] : std::max(1.0, std::abs(b[c]))));
        Near(a[c], b[c], 3e-12 * (secondOrder ? absolute[c] : std::max(1.0, std::abs(a[c]))));
    }
    Near(d.totalEnergyDefect, d.final.totalEnergy - d.initial.totalEnergy, 0);
    REQUIRE(std::abs(d.totalEnergyDefect) <= d.totalEnergyRoundoffAllowance);
    REQUIRE(d.final.minimumDensity > 0);
    REQUIRE(d.final.minimumPressure > 0);
    REQUIRE(d.maximumCfl < 1);
    if (!secondOrder)
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
ContactResult Contact(std::size_t nx, bool secondOrder = false) {
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
    EulerGasSecondOrderConfig highOptions;
    highOptions.maximumCellVisits = 500000000; // The 256x128 refinement is explicitly budgeted.
    const auto d = secondOrder ? grid.stepSecondOrder(duration, highOptions) : grid.step(duration);
    const auto actual = grid.state();
    const auto p = grid.primitives();
    Audit(s, actual, d, g.spacingX * g.spacingY, secondOrder);
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
double SimpleWaveError(std::size_t nx, bool secondOrder = false) {
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
    const auto d = secondOrder ? grid.stepSecondOrder(duration) : grid.step(duration);
    const auto actual = grid.state();
    Audit(s, actual, d, g.spacingX * g.spacingY, secondOrder);
    double error = 0;
    for (std::size_t i = 0; i < nx; ++i) {
        const auto exact = Average(i * g.spacingX, (i + 1) * g.spacingX,
                                   [duration](double x) { return SimpleWave(x, duration); });
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
ShockResult Shock(std::size_t nx, double length, bool secondOrder = false) {
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
    const auto d = secondOrder ? grid.stepSecondOrder(t) : grid.step(t);
    const auto actual = grid.state();
    Audit(s, actual, d, dx * .5, secondOrder);
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

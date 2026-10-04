#include "physics/core/fluids/periodic_euler_gas_grid.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <initializer_list>
#include <limits>
#include <stdexcept>
#include <utility>

namespace PhysicsEngine {
namespace {
double Checked(double x) {
    if (!std::isfinite(x))
        throw std::overflow_error("Euler gas arithmetic exceeds float64 range.");
    return x;
}
// Avoid intermediate overflow in products/quotients. A lost nonzero complete
// product is a range error, not a pressure/density floor.
double Product(std::initializer_list<double> factors, std::initializer_list<double> divisors = {}) {
    double m = 1;
    int exponent = 0, e = 0;
    for (double x : factors) {
        if (x == 0)
            return 0;
        m *= std::frexp(x, &e);
        exponent += e;
    }
    for (double x : divisors) {
        m /= std::frexp(x, &e);
        exponent -= e;
    }
    const double result = Checked(std::ldexp(m, exponent));
    if (result == 0)
        throw std::overflow_error("Euler gas nonzero product underflows float64 range.");
    return result;
}
double RootProduct(std::initializer_list<double> factors,
                   std::initializer_list<double> divisors = {}) {
    double m = 1;
    int exponent = 0, e = 0;
    for (double x : factors) {
        m *= std::frexp(x, &e);
        exponent += e;
    }
    for (double x : divisors) {
        m /= std::frexp(x, &e);
        exponent -= e;
    }
    if (exponent % 2 != 0) {
        m *= 2;
        --exponent;
    }
    const double result = Checked(std::ldexp(std::sqrt(m), exponent / 2));
    if (result == 0)
        throw std::overflow_error("Euler gas nonzero square root underflows float64 range.");
    return result;
}
double Normalize(double value, double scale) {
    return Product({value}, {scale});
}
double Upper(double value) {
    return Checked(std::nextafter(Checked(value), std::numeric_limits<double>::infinity()));
}
void Add(double &sum, double &correction, double value) {
    const double next = Checked(sum + value);
    correction = Checked(correction + (std::abs(sum) >= std::abs(value) ? (sum - next) + value
                                                                        : (value - next) + sum));
    sum = next;
}
struct Primitive {
    double u, v, p, c, internal, kinetic;
};
Primitive Decode(double rho, double mx, double my, double energy, double gamma) {
    if (!std::isfinite(rho) || rho <= 0 || !std::isfinite(mx) || !std::isfinite(my) ||
        !std::isfinite(energy) || energy <= 0)
        throw std::invalid_argument(
            "Euler gas requires finite fields and strictly positive density/energy.");
    Primitive q;
    q.u = Product({mx}, {rho});
    q.v = Product({my}, {rho});
    q.kinetic = Checked(Product({.5, mx, mx}, {rho}) + Product({.5, my, my}, {rho}));
    q.internal = Checked(energy - q.kinetic);
    if (q.internal <= 0)
        throw std::invalid_argument("Euler gas stored internal energy must be strictly positive.");
    q.p = Product({gamma - 1, q.internal});
    q.c = RootProduct({gamma, q.p}, {rho});
    return q;
}
Primitive Decode(const EulerGasState &s, std::size_t k, double gamma) {
    return Decode(s.density[k], s.momentumX[k], s.momentumY[k], s.totalEnergy[k], gamma);
}
using Fields = std::array<std::vector<double>, 4>;
std::array<const std::vector<double> *, 4> Arrays(const EulerGasState &s) {
    return {{&s.density, &s.momentumX, &s.momentumY, &s.totalEnergy}};
}
std::array<std::vector<double> *, 4> Arrays(EulerGasState &s) {
    return {{&s.density, &s.momentumX, &s.momentumY, &s.totalEnergy}};
}
EulerGasSummary Summarize(const EulerGasState &s, const EulerGasGridConfig &g, double area) {
    EulerGasSummary result;
    result.minimumDensity = result.maximumDensity = s.density[0];
    const auto n = s.density.size();
    std::array<double, 6> scale{}, sum{}, correction{};
    for (std::size_t k = 0; k < n; ++k) {
        const auto q = Decode(s, k, g.gamma);
        const double values[] = {s.density[k],     s.momentumX[k], s.momentumY[k],
                                 s.totalEnergy[k], q.internal,     q.kinetic};
        for (std::size_t a = 0; a < 6; ++a)
            scale[a] = std::max(scale[a], std::abs(values[a]));
        result.minimumDensity = std::min(result.minimumDensity, s.density[k]);
        result.maximumDensity = std::max(result.maximumDensity, s.density[k]);
        if (k == 0)
            result.minimumPressure = result.maximumPressure = q.p;
        result.minimumPressure = std::min(result.minimumPressure, q.p);
        result.maximumPressure = std::max(result.maximumPressure, q.p);
    }
    double absX = 0, absY = 0, correctionX = 0, correctionY = 0;
    for (std::size_t k = 0; k < n; ++k) {
        const auto q = Decode(s, k, g.gamma);
        const double values[] = {s.density[k],     s.momentumX[k], s.momentumY[k],
                                 s.totalEnergy[k], q.internal,     q.kinetic};
        for (std::size_t a = 0; a < 6; ++a) {
            const double value = scale[a] == 0 ? 0 : Normalize(values[a], scale[a]);
            Add(sum[a], correction[a], value);
            if (a == 1)
                Add(absX, correctionX, std::abs(value));
            if (a == 2)
                Add(absY, correctionY, std::abs(value));
        }
    }
    std::array<double *, 6> output{{&result.mass, &result.momentumX, &result.momentumY,
                                    &result.totalEnergy, &result.internalEnergy,
                                    &result.kineticEnergy}};
    for (std::size_t a = 0; a < 6; ++a)
        *output[a] = Product({Checked(sum[a] + correction[a]), scale[a], area});
    result.absoluteMomentumX = Product({Checked(absX + correctionX), scale[1], area});
    result.absoluteMomentumY = Product({Checked(absY + correctionY), scale[2], area});
    return result;
}
struct Waves {
    double x = 0, y = 0, rate = 0, densityScale = 0, energyScale = 0;
};
Waves Scan(const EulerGasState &s, const EulerGasGridConfig &g,
           std::vector<Primitive> *cache = nullptr) {
    Waves w;
    for (std::size_t k = 0; k < s.density.size(); ++k) {
        const auto q = Decode(s, k, g.gamma);
        if (cache)
            (*cache)[k] = q;
        w.x = std::max(w.x, Upper(Checked(std::abs(q.u) + Upper(q.c))));
        w.y = std::max(w.y, Upper(Checked(std::abs(q.v) + Upper(q.c))));
        w.densityScale = std::max(w.densityScale, s.density[k]);
        w.energyScale = std::max(w.energyScale, s.totalEnergy[k]);
    }
    w.rate =
        Upper(Checked(Upper(Product({w.x}, {g.spacingX})) + Upper(Product({w.y}, {g.spacingY}))));
    return w;
}
double Updated(double old, double increment, double scale) {
    if (increment == 0)
        return old;
    if (std::abs(increment) > 1 && scale > std::numeric_limits<double>::max() / std::abs(increment))
        return Product({Checked(Normalize(old, scale) + increment), scale});
    return Checked(old + Product({increment, scale}));
}
// A face transfer h*F*/spacing, normalized by the shared component scale.
// E+p is split before multiplication so an avoidable sum overflow cannot
// reject a representable CFL-scaled energy transfer.
double Transfer(const EulerGasState &s, const std::vector<Primitive> &q, const Fields &normalized,
                std::size_t left, std::size_t right, std::size_t a, bool x, double h,
                double spacing, double alpha, double scale) {
    double sum = 0, correction = 0;
    for (const auto k : {left, right}) {
        const double velocity = x ? q[k].u : q[k].v;
        if (a == 0)
            Add(sum, correction,
                Product({.5, h, x ? s.momentumX[k] : s.momentumY[k]}, {spacing, scale}));
        else if (a == 1 || a == 2) {
            Add(sum, correction,
                Product({.5, h, a == 1 ? s.momentumX[k] : s.momentumY[k], velocity},
                        {spacing, scale}));
            if ((a == 1) == x)
                Add(sum, correction, Product({.5, h, q[k].p}, {spacing, scale}));
        } else {
            Add(sum, correction, Product({.5, h, s.totalEnergy[k], velocity}, {spacing, scale}));
            Add(sum, correction, Product({.5, h, q[k].p, velocity}, {spacing, scale}));
        }
    }
    Add(sum, correction,
        -Product({.5, h, alpha, Checked(normalized[a][right] - normalized[a][left])}, {spacing}));
    return Checked(sum + correction);
}
void Conservation(EulerGasDiagnostics &d, std::size_t n) {
    // Scale-aware heuristic accumulation guard, not an interval error proof.
    const double factor = (64.0 * double(n) + 96.0 * double(d.substeps) + 128.0) *
                          std::numeric_limits<double>::epsilon();
    d.massDefect = Checked(d.final.mass - d.initial.mass);
    d.momentumXDefect = Checked(d.final.momentumX - d.initial.momentumX);
    d.momentumYDefect = Checked(d.final.momentumY - d.initial.momentumY);
    d.totalEnergyDefect = Checked(d.final.totalEnergy - d.initial.totalEnergy);
    d.massRoundoffAllowance = Product({factor, std::max(d.initial.mass, d.final.mass)});
    d.momentumXRoundoffAllowance =
        Product({factor, std::max(d.initial.absoluteMomentumX, d.final.absoluteMomentumX)});
    d.momentumYRoundoffAllowance =
        Product({factor, std::max(d.initial.absoluteMomentumY, d.final.absoluteMomentumY)});
    d.totalEnergyRoundoffAllowance =
        Product({factor, std::max(d.initial.totalEnergy, d.final.totalEnergy)});
    if (std::abs(d.massDefect) > d.massRoundoffAllowance ||
        std::abs(d.momentumXDefect) > d.momentumXRoundoffAllowance ||
        std::abs(d.momentumYDefect) > d.momentumYRoundoffAllowance ||
        std::abs(d.totalEnergyDefect) > d.totalEnergyRoundoffAllowance)
        throw std::runtime_error("Euler gas stored conservation audit failed.");
}
} // namespace

void EulerGasGridConfig::Validate() const {
    if (columns < 2 || rows < 2 || columns > MaximumCells || rows > MaximumCells ||
        columns > MaximumCells / rows)
        throw std::invalid_argument("Euler gas grid dimensions exceed supported bounds.");
    if (!std::isfinite(spacingX) || spacingX <= 0 || !std::isfinite(spacingY) || spacingY <= 0 ||
        !std::isfinite(gamma) || gamma <= 1)
        throw std::invalid_argument(
            "Euler gas requires positive finite spacings and finite gamma > 1.");
    (void)Product({spacingX, spacingY});
    (void)Product({spacingX, double(columns)});
    (void)Product({spacingY, double(rows)});
}
void EulerGasStepConfig::Validate() const {
    if (!std::isfinite(cflSafety) || cflSafety <= 0 || cflSafety >= 1 ||
        !std::isfinite(maxSubstep) || maxSubstep <= 0)
        throw std::invalid_argument(
            "Euler gas requires strict (0,1) CFL safety and positive finite maxSubstep.");
    if (maximumSubsteps > MaximumSubsteps || maximumCellVisits > MaximumCellVisits)
        throw std::invalid_argument("Euler gas work budgets exceed hard ceilings.");
}
PeriodicEulerGasGrid::PeriodicEulerGasGrid(const EulerGasGridConfig &config) : config_(config) {
    config_.Validate();
    const auto n = config_.columns * config_.rows;
    const double energy = Product({1}, {config_.gamma - 1});
    (void)Decode(1, 0, 0, energy, config_.gamma);
    state_.density.assign(n, 1);
    state_.momentumX.assign(n, 0);
    state_.momentumY.assign(n, 0);
    state_.totalEnergy.assign(n, energy);
}
EulerGasPrimitives PeriodicEulerGasGrid::primitives() const {
    EulerGasPrimitives result;
    const auto n = state_.density.size();
    result.velocityX.resize(n);
    result.velocityY.resize(n);
    result.pressure.resize(n);
    result.soundSpeed.resize(n);
    result.internalEnergy.resize(n);
    for (std::size_t k = 0; k < n; ++k) {
        const auto q = Decode(state_, k, config_.gamma);
        result.velocityX[k] = q.u;
        result.velocityY[k] = q.v;
        result.pressure[k] = q.p;
        result.soundSpeed[k] = q.c;
        result.internalEnergy[k] = q.internal;
    }
    return result;
}
void PeriodicEulerGasGrid::setState(const EulerGasState &state) {
    const auto n = state_.density.size();
    for (const auto *field : Arrays(state))
        if (field->size() != n)
            throw std::invalid_argument("Euler gas all state arrays must match the grid.");
    for (std::size_t k = 0; k < n; ++k)
        (void)Decode(state, k, config_.gamma);
    auto staged = state;
    state_ = std::move(staged);
}
EulerGasDiagnostics PeriodicEulerGasGrid::step(double duration, const EulerGasStepConfig &options) {
    options.Validate();
    if (!std::isfinite(duration) || duration < 0)
        throw std::invalid_argument("Euler gas duration must be finite and nonnegative.");
    const auto n = state_.density.size(), nx = config_.columns, ny = config_.rows;
    const auto passes = options.maximumCellVisits / n;
    if (passes < (duration == 0 ? 3u : 10u))
        throw std::runtime_error("Euler gas cell-visit budget exhausted.");
    if (duration > 0 && options.maximumSubsteps == 0)
        throw std::runtime_error("Euler gas substep budget exhausted.");
    EulerGasDiagnostics d;
    d.duration = duration;
    d.timeBefore = time_;
    d.timeAfter = Checked(time_ + duration);
    if (duration > 0 && d.timeAfter <= time_)
        throw std::overflow_error("Euler gas clock increment is unrepresentable.");
    const double area = Product({config_.spacingX, config_.spacingY});
    d.initial = Summarize(state_, config_, area);
    d.cellVisits = 2 * n;
    if (duration == 0) {
        const auto waves = Scan(state_, config_);
        d.maximumSignalSpeedX = waves.x;
        d.maximumSignalSpeedY = waves.y;
        d.cellVisits += n;
        d.final = d.initial;
        d.zeroDurationNoOp = true;
        Conservation(d, n);
        diagnostics_ = d;
        return d;
    }
    auto staged = state_;
    std::vector<Primitive> q(n);
    Fields normalized, delta, correction;
    for (std::size_t a = 0; a < 4; ++a) {
        normalized[a].resize(n);
        delta[a].resize(n);
        correction[a].resize(n);
    }
    double elapsed = 0, remaining = duration;
    while (remaining > 0) {
        if (d.substeps == options.maximumSubsteps)
            throw std::runtime_error("Euler gas substep budget exhausted.");
        if ((options.maximumCellVisits - d.cellVisits) / n < 8)
            throw std::runtime_error("Euler gas adaptive cell-visit budget exhausted.");
        const auto waves = Scan(staged, config_, &q);
        d.cellVisits += n;
        d.maximumSignalSpeedX = std::max(d.maximumSignalSpeedX, waves.x);
        d.maximumSignalSpeedY = std::max(d.maximumSignalSpeedY, waves.y);
        double h = std::min(remaining, options.maxSubstep);
        const double cflLimit = options.cflSafety / waves.rate;
        if (std::isfinite(cflLimit))
            h = std::min(h, std::nextafter(cflLimit, 0.0));
        if (h <= 0)
            throw std::overflow_error("Euler gas stable substep is unrepresentable.");
        const double cfl = Product({h, waves.rate});
        if (cfl > options.cflSafety || cfl >= 1)
            throw std::runtime_error("Euler gas rounded CFL exceeds safety.");
        const double nextElapsed = h == remaining ? duration : Checked(elapsed + h);
        if (nextElapsed <= elapsed)
            throw std::overflow_error("Euler gas adaptive duration increment is unrepresentable.");
        const std::array<double, 4> scale{
            {waves.densityScale, RootProduct({waves.densityScale, waves.energyScale}),
             RootProduct({waves.densityScale, waves.energyScale}), waves.energyScale}};
        const auto old = Arrays(static_cast<const EulerGasState &>(staged));
        for (std::size_t k = 0; k < n; ++k)
            for (std::size_t a = 0; a < 4; ++a)
                normalized[a][k] = Normalize((*old[a])[k], scale[a]);
        d.cellVisits += n;
        for (std::size_t k = 0; k < n; ++k)
            for (std::size_t a = 0; a < 4; ++a)
                delta[a][k] = correction[a][k] = 0;
        d.cellVisits += n;
        for (const bool x : {true, false}) {
            for (std::size_t j = 0; j < ny; ++j)
                for (std::size_t i = 0; i < nx; ++i) {
                    const auto right = i + nx * j;
                    const auto left = x ? (i + nx - 1) % nx + nx * j : i + nx * ((j + ny - 1) % ny);
                    for (std::size_t a = 0; a < 4; ++a) {
                        const double transfer = Transfer(staged, q, normalized, left, right, a, x,
                                                         h, x ? config_.spacingX : config_.spacingY,
                                                         x ? waves.x : waves.y, scale[a]);
                        Add(delta[a][left], correction[a][left], -transfer);
                        Add(delta[a][right], correction[a][right], transfer);
                    }
                }
            d.cellVisits += n;
        }
        auto output = Arrays(staged);
        for (std::size_t k = 0; k < n; ++k) {
            for (std::size_t a = 0; a < 4; ++a)
                (*output[a])[k] =
                    Updated((*output[a])[k], Checked(delta[a][k] + correction[a][k]), scale[a]);
            (void)Decode(staged, k, config_.gamma);
        }
        d.cellVisits += n;
        ++d.substeps;
        d.lastSubstep = h;
        d.maximumCfl = std::max(d.maximumCfl, cfl);
        elapsed = nextElapsed;
        remaining = h == remaining ? 0 : Checked(duration - elapsed);
        if (remaining < 0)
            throw std::overflow_error("Euler gas adaptive duration lost range.");
    }
    d.final = Summarize(staged, config_, area);
    d.cellVisits += 2 * n;
    Conservation(d, n);
    state_ = std::move(staged);
    time_ = d.timeAfter;
    diagnostics_ = d;
    return d;
}
} // namespace PhysicsEngine

#include "euler_gas_numeric.h"

namespace PhysicsEngine {
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

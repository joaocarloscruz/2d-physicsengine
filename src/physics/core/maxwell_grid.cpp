#include "physics/core/maxwell_grid.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace PhysicsEngine {
namespace {
double Checked(double value) {
    if (!std::isfinite(value))
        throw std::overflow_error("Maxwell arithmetic exceeds float64 range.");
    return value;
}
double Positive(double value) {
    if (!std::isfinite(value) || value <= 0)
        throw std::invalid_argument("Maxwell coefficient must be positively representable.");
    return value;
}
// Scaling avoids per-sample division underflow; compensation retains ordinary
// signed cancellation. Reject erased normalized/rescaled terms rather than
// silently presenting a mean from a different set of samples.
class ScaledMean {
    int exponent_ = 0;
    bool nonzero_ = false;
    bool initialized_ = false, constant_ = true;
    double first_ = 0;
    double sum_ = 0, correction_ = 0;
    static double Rescale(double x, int exponent) {
        const double result = Checked(std::scalbn(x, exponent));
        if (std::scalbn(result, -exponent) != x)
            throw std::overflow_error("Maxwell mean scaling erases a contribution.");
        return result;
    }

  public:
    void add(double x) {
        if (!initialized_) {
            initialized_ = true;
            first_ = x;
        } else if (x != first_) {
            constant_ = false;
        }
        if (x == 0)
            return;
        const int exponent = std::ilogb(std::abs(x));
        if (!nonzero_) {
            nonzero_ = true;
            exponent_ = exponent;
        } else if (exponent > exponent_) {
            sum_ = Rescale(sum_, exponent_ - exponent);
            correction_ = Rescale(correction_, exponent_ - exponent);
            exponent_ = exponent;
        }
        const double term = Rescale(x, -exponent_);
        const double next = sum_ + term;
        correction_ +=
            std::abs(sum_) >= std::abs(term) ? (sum_ - next) + term : (term - next) + sum_;
        sum_ = next;
    }
    double value(std::size_t count) const {
        // N*x rounded and divided by N need not round back to x for arbitrary
        // N. A constant observed sample has exactly that stored-value mean.
        if (constant_)
            return first_ == 0 ? 0 : first_;
        if (!nonzero_)
            return 0;
        const double normalized = Checked(sum_ + correction_);
        if (normalized == 0)
            return 0;
        int exponent;
        const double mantissa = std::frexp(normalized, &exponent);
        const double result = Checked(std::scalbn(mantissa / double(count), exponent + exponent_));
        if (result == 0)
            throw std::overflow_error("Maxwell mean underflows float64 range.");
        return result;
    }
};
struct Coefficients {
    double ix, iy, ex, ey, mx, my, electricScale, magneticScale, speed, rate, limit;
};
Coefficients Validate(const MaxwellGridConfig &c) {
    if (c.columns < 2 || c.rows < 2 || c.columns > MaxwellGridConfig::MaximumCells ||
        c.rows > MaxwellGridConfig::MaximumCells ||
        c.columns > MaxwellGridConfig::MaximumCells / c.rows)
        throw std::length_error("Maxwell dimensions exceed the hard cell limit.");
    if (!std::isfinite(c.cflSafety) || c.cflSafety <= 0 || c.cflSafety >= 1 ||
        c.maximumSubsteps == 0 || c.maximumSubsteps > MaxwellGridConfig::MaximumSubsteps ||
        c.maximumCellVisits == 0 || c.maximumCellVisits > MaxwellGridConfig::MaximumCellVisits)
        throw std::invalid_argument("Invalid Maxwell CFL safety or work budgets.");
    Positive(c.spacingX);
    Positive(c.spacingY);
    Positive(c.permittivity);
    Positive(c.permeability);
    Positive(c.maxSubstep);
    const double ix = Positive(1 / c.spacingX), iy = Positive(1 / c.spacingY);
    const double ie = Positive(1 / c.permittivity), im = Positive(1 / c.permeability);
    const double area = Positive(c.spacingX * c.spacingY);
    Positive(c.spacingX * c.columns);
    Positive(c.spacingY * c.rows);
    const double ex = Positive(ie * ix), ey = Positive(ie * iy);
    const double mx = Positive(im * ix), my = Positive(im * iy);
    const double electricScale = std::sqrt(Positive(0.5 * Positive(c.permittivity * area)));
    const double magneticScale = std::sqrt(Positive(0.5 * Positive(c.permeability * area)));
    const double speed = Positive(std::sqrt(ie) * std::sqrt(im));
    const double rate = Positive(speed * Positive(std::hypot(ix, iy)));
    const double cfl = Positive(c.cflSafety / rate);
    // Only the computed physical duration is rounded down. An exactly supplied
    // maxSubstep must not acquire an extra substep when the CFL is loose.
    const double limit = Positive(std::min(c.maxSubstep, std::nextafter(cfl, 0.0)));
    return {ix, iy, ex, ey, mx, my, electricScale, magneticScale, speed, rate, limit};
}
double NormEnergy(double norm, bool nonzero) {
    const double energy = Checked(norm * norm);
    if (nonzero && energy == 0)
        throw std::overflow_error("Maxwell aggregate energy underflows float64 range.");
    return energy;
}
struct Sum {
    double sum = 0, correction = 0;
    void add(double x) {
        const double next = Checked(sum + x);
        correction = Checked(correction +
                             (std::abs(sum) >= std::abs(x) ? (sum - next) + x : (x - next) + sum));
        sum = next;
    }
    double value() const { return Checked(sum + correction); }
};
double DecayExponent(double duration, double sigma, double epsilon) {
    int eh, es, ee;
    const double mh = std::frexp(duration, &eh), ms = std::frexp(sigma, &es),
                 me = std::frexp(epsilon, &ee);
    const double chi = Checked(std::scalbn((mh * ms) / me, eh + es - ee - 1));
    if (chi <= 0)
        throw std::overflow_error("Maxwell Ohmic exponent underflows float64 range.");
    return chi;
}
double Loss(double fraction, double energy) {
    const double result = Checked(fraction * energy);
    if (energy > 0 && result == 0)
        throw std::overflow_error("Maxwell Ohmic subflow loss underflows float64 range.");
    return result;
}
double Balance(double final, double initial, double joule, double wave) {
    const double scale = std::max({final, initial, joule, std::abs(wave)});
    if (scale == 0)
        return 0;
    Sum normalized;
    for (double x : {final, -initial, joule, -wave}) {
        const double ratio = x / scale;
        if (x != 0 && ratio == 0)
            throw std::overflow_error("Maxwell Ohmic ledger normalization erases a term.");
        normalized.add(ratio);
    }
    const double residual = Checked(scale * normalized.value());
    if (normalized.value() != 0 && residual == 0)
        throw std::overflow_error("Maxwell Ohmic balance underflows float64 range.");
    return residual;
}
std::size_t Index(std::size_t i, std::size_t j, std::size_t columns) { return i + columns * j; }
} // namespace
void MaxwellGridConfig::Validate() const { PhysicsEngine::Validate(*this); }
MaxwellGrid::MaxwellGrid(const MaxwellGridConfig &config) : config_(config) {
    const auto c = Validate(config_);
    ix_ = c.ix;
    iy_ = c.iy;
    electricX_ = c.ex;
    electricY_ = c.ey;
    magneticX_ = c.mx;
    magneticY_ = c.my;
    electricScale_ = c.electricScale;
    magneticScale_ = c.magneticScale;
    speed_ = c.speed;
    rate_ = c.rate;
    limit_ = c.limit;
    const auto count = config_.columns * config_.rows;
    state_.ez.resize(count);
    state_.hx.resize(count);
    state_.hy.resize(count);
    diagnostics_ = measure(state_, 0);
}
double MaxwellGrid::divergenceAt(const MaxwellFieldState &s, std::size_t i, std::size_t j) const {
    const auto nx = config_.columns, ny = config_.rows, k = Index(i, j, nx);
    return Checked(Checked(Checked(s.hx[Index((i + 1) % nx, j, nx)] - s.hx[k]) * ix_) +
                   Checked(Checked(s.hy[Index(i, (j + 1) % ny, nx)] - s.hy[k]) * iy_));
}
MaxwellGridDiagnostics MaxwellGrid::measure(const MaxwellFieldState &s, double h,
                                            double *electricModifiedPart) const {
    MaxwellGridDiagnostics d;
    d.stableTimeStep = limit_;
    d.modifiedEnergyStep = h;
    const auto nx = config_.columns, ny = config_.rows, n = nx * ny;
    double electricNorm = 0, magneticNorm = 0, correctionNorm = 0;
    ScaledMean meanEz, meanHx, meanHy;
    bool nonzeroElectric = false, nonzeroMagnetic = false, nonzeroCorrection = false;
    const double hx = 0.5 * h * magneticX_, hy = 0.5 * h * magneticY_, rootN = std::sqrt(double(n));
    for (std::size_t j = 0; j < ny; ++j)
        for (std::size_t i = 0; i < nx; ++i) {
            const auto k = Index(i, j, nx);
            electricNorm = Checked(std::hypot(electricNorm, Checked(electricScale_ * s.ez[k])));
            magneticNorm = Checked(std::hypot(magneticNorm, Checked(magneticScale_ * s.hx[k])));
            magneticNorm = Checked(std::hypot(magneticNorm, Checked(magneticScale_ * s.hy[k])));
            nonzeroElectric = nonzeroElectric || s.ez[k] != 0;
            nonzeroMagnetic = nonzeroMagnetic || s.hx[k] != 0 || s.hy[k] != 0;
            meanEz.add(s.ez[k]);
            meanHx.add(s.hx[k]);
            meanHy.add(s.hy[k]);
            d.maxAbsEz = std::max(d.maxAbsEz, std::abs(s.ez[k]));
            d.maxAbsHx = std::max(d.maxAbsHx, std::abs(s.hx[k]));
            d.maxAbsHy = std::max(d.maxAbsHy, std::abs(s.hy[k]));
            const double div = divergenceAt(s, i, j);
            d.magneticDivergenceRms = Checked(std::hypot(d.magneticDivergenceRms, div / rootN));
            d.maxAbsMagneticDivergence = std::max(d.maxAbsMagneticDivergence, std::abs(div));
            if (h > 0) {
                const double dx = Checked(s.ez[Index((i + 1) % nx, j, nx)] - s.ez[k]);
                const double dy = Checked(s.ez[Index(i, (j + 1) % ny, nx)] - s.ez[k]);
                nonzeroCorrection = nonzeroCorrection || dx != 0 || dy != 0;
                const double x = Checked(hx * dx), y = Checked(hy * dy);
                correctionNorm = Checked(std::hypot(correctionNorm, Checked(magneticScale_ * x)));
                correctionNorm = Checked(std::hypot(correctionNorm, Checked(magneticScale_ * y)));
            }
        }
    d.meanEz = meanEz.value(n);
    d.meanHx = meanHx.value(n);
    d.meanHy = meanHy.value(n);
    if (d.maxAbsMagneticDivergence > 0 && d.magneticDivergenceRms == 0)
        throw std::overflow_error("Maxwell magnetic-divergence RMS underflows.");
    d.electricEnergy = NormEnergy(electricNorm, nonzeroElectric);
    d.magneticEnergy = NormEnergy(magneticNorm, nonzeroMagnetic);
    const double correction = NormEnergy(correctionNorm, nonzeroCorrection);
    if (electricModifiedPart) {
        *electricModifiedPart = Checked(d.electricEnergy - correction);
        if (*electricModifiedPart < 0 || (d.electricEnergy > 0 && *electricModifiedPart == 0))
            throw std::overflow_error("Maxwell electric modified-energy part is unrepresentable.");
    }
    d.totalEnergy = Checked(d.electricEnergy + d.magneticEnergy);
    d.modifiedEnergy = Checked(d.totalEnergy - correction);
    if (d.modifiedEnergy < 0 || (d.totalEnergy > 0 && d.modifiedEnergy == 0))
        throw std::overflow_error("Maxwell modified energy is not representably nonnegative.");
    return d;
}
void MaxwellGrid::setState(const MaxwellFieldState &state) {
    const auto n = state_.ez.size();
    if (state.ez.size() != n || state.hx.size() != n || state.hy.size() != n)
        throw std::invalid_argument("Maxwell field array size mismatch.");
    for (const auto *values : {&state.ez, &state.hx, &state.hy})
        for (double x : *values)
            if (!std::isfinite(x))
                throw std::invalid_argument("Maxwell fields must be finite.");
    auto next = state;
    auto d = measure(next, 0);
    d.time = diagnostics_.time;
    state_.ez.swap(next.ez);
    state_.hx.swap(next.hx);
    state_.hy.swap(next.hy);
    diagnostics_ = d;
}
std::vector<double> MaxwellGrid::getMagneticDivergence() const {
    std::vector<double> result(state_.ez.size());
    for (std::size_t j = 0; j < config_.rows; ++j)
        for (std::size_t i = 0; i < config_.columns; ++i)
            result[Index(i, j, config_.columns)] = divergenceAt(state_, i, j);
    return result;
}
double MaxwellGrid::getModifiedEnergy(double h) const {
    if (!std::isfinite(h) || h < 0 || (h > 0 && !(rate_ * h < 1)))
        throw std::invalid_argument("Maxwell reference step must satisfy strict CFL.");
    if (h > 0)
        for (double value : {0.5 * h * magneticX_, 0.5 * h * magneticY_})
            if (!std::isfinite(value) || value <= 0)
                throw std::overflow_error("Maxwell reference-step coefficient is unrepresentable.");
    return measure(state_, h).modifiedEnergy;
}
MaxwellGrid::StepPlan MaxwellGrid::plan(double dt, std::size_t passesPerSubstep) const {
    const double requested = std::ceil(dt / limit_);
    // Compare against the hard/user count bounds before the only count cast.
    if (!std::isfinite(requested) || requested > double(config_.maximumSubsteps))
        throw std::length_error("Maxwell substep limit exceeded.");
    auto count = static_cast<std::size_t>(std::max(1.0, requested));
    const auto n = state_.ez.size(), passes = config_.maximumCellVisits / n;
    if (passes < passesPerSubstep + 1 || count > (passes - 1) / passesPerSubstep)
        throw std::length_error("Maxwell cell-visit budget exceeded.");
    const auto budget = std::min(config_.maximumSubsteps, (passes - 1) / passesPerSubstep);
    double h = dt / count;
    if (h > limit_) {
        if (count >= budget)
            throw std::length_error("Maxwell substep budget exceeded after rounding.");
        h = dt / ++count;
    }
    const double half = 0.5 * h, mx = half * magneticX_, my = half * magneticY_;
    const double ex = h * electricX_, ey = h * electricY_;
    for (double value : {h, half, mx, my, ex, ey})
        if (!std::isfinite(value) || value <= 0)
            throw std::overflow_error("Maxwell substep coefficient is unrepresentable.");
    if (h > limit_ || !(rate_ * h < 1))
        throw std::overflow_error("Maxwell substep violates strict CFL.");
    const double time = Checked(diagnostics_.time + dt);
    if (time == diagnostics_.time)
        throw std::overflow_error("Maxwell clock increment is unrepresentable.");
    return {h, time, count};
}
void MaxwellGrid::waveStep(MaxwellFieldState &next, double h) const {
    const double half = 0.5 * h, mx = half * magneticX_, my = half * magneticY_;
    const double ex = h * electricX_, ey = h * electricY_;
    const auto nx = config_.columns, ny = config_.rows;
    auto kick = [&]() {
        for (std::size_t j = 0; j < ny; ++j)
            for (std::size_t i = 0; i < nx; ++i) {
                const auto k = Index(i, j, nx);
                next.hx[k] = Checked(
                    next.hx[k] -
                    Checked(my * Checked(next.ez[Index(i, (j + 1) % ny, nx)] - next.ez[k])));
                next.hy[k] = Checked(
                    next.hy[k] +
                    Checked(mx * Checked(next.ez[Index((i + 1) % nx, j, nx)] - next.ez[k])));
            }
    };
    {
        kick();
        for (std::size_t j = 0; j < ny; ++j)
            for (std::size_t i = 0; i < nx; ++i) {
                const auto k = Index(i, j, nx);
                const double x =
                    Checked(ex * Checked(next.hy[k] - next.hy[Index((i + nx - 1) % nx, j, nx)]));
                const double y =
                    Checked(ey * Checked(next.hx[k] - next.hx[Index(i, (j + ny - 1) % ny, nx)]));
                next.ez[k] = Checked(next.ez[k] + Checked(x - y));
            }
        kick();
    }
}
void MaxwellGrid::step(double dt) {
    if (!std::isfinite(dt) || dt < 0)
        throw std::invalid_argument("Maxwell timestep must be finite and nonnegative.");
    if (dt == 0)
        return;
    const auto p = plan(dt, 3);
    const auto h = p.h;
    const auto count = p.count, n = state_.ez.size();
    auto next = state_;
    for (std::size_t s = 0; s < count; ++s)
        waveStep(next, h);
    auto d = measure(next, h);
    d.time = p.time;
    d.lastSubstep = h;
    d.lastSubsteps = count;
    d.lastCellVisits = n * (3 * count + 1);
    state_.ez.swap(next.ez);
    state_.hx.swap(next.hx);
    state_.hy.swap(next.hy);
    diagnostics_ = d;
}
MaxwellOhmicStepDiagnostics MaxwellGrid::stepOhmic(double dt, double conductivity) {
    if (!std::isfinite(dt) || dt < 0 || !std::isfinite(conductivity) || conductivity < 0)
        throw std::invalid_argument(
            "Ohmic duration and conductivity must be finite and nonnegative.");
    MaxwellOhmicStepDiagnostics ledger;
    ledger.conductivity = conductivity;
    ledger.duration = dt;
    ledger.startTime = ledger.endTime = diagnostics_.time;
    ledger.initialPhysicalEnergy = ledger.finalPhysicalEnergy = diagnostics_.totalEnergy;
    if (dt == 0)
        return ledger;
    if (conductivity == 0) {
        step(dt); // No additional throwing observation after existing publication.
        ledger.endTime = diagnostics_.time;
        ledger.finalPhysicalEnergy = diagnostics_.totalEnergy;
        ledger.wavePhysicalEnergyChange = ledger.finalPhysicalEnergy - ledger.initialPhysicalEnergy;
        ledger.substep = diagnostics_.lastSubstep;
        ledger.substeps = diagnostics_.lastSubsteps;
        ledger.cellVisits = diagnostics_.lastCellVisits;
        return ledger;
    }
    const auto p = plan(dt, 8);
    const double chi = DecayExponent(p.h, conductivity, config_.permittivity);
    const double factor = Checked(std::exp(-chi)), decrement = Checked(-std::expm1(-chi)),
                 energyFraction = Checked(-std::expm1(-2 * chi));
    if (factor == 0 || decrement <= 0 || energyFraction <= 0)
        throw std::overflow_error("Maxwell Ohmic decay factor is unrepresentable.");
    auto next = state_;
    double electricQ = 0;
    auto observed = measure(next, p.h, &electricQ);
    Sum joule, represented, wave, modified;
    auto decay = [&]() {
        // Exact split-subflow work, independently of rounded stored-field loss.
        joule.add(Loss(energyFraction, observed.electricEnergy));
        modified.add(Loss(energyFraction, electricQ));
        const double before = observed.electricEnergy;
        for (double &e : next.ez) {
            const double value = Checked(chi < .5 ? e - Checked(e * decrement) : e * factor);
            if (e != 0 && value == 0)
                throw std::overflow_error("Maxwell Ohmic field update underflows float64 range.");
            e = value;
        }
        observed = measure(next, p.h, &electricQ);
        represented.add(Checked(before - observed.electricEnergy));
    };
    for (std::size_t s = 0; s < p.count; ++s) {
        decay();
        const double beforeWave = observed.totalEnergy;
        waveStep(next, p.h);
        observed = measure(next, p.h, &electricQ);
        wave.add(Checked(observed.totalEnergy - beforeWave));
        decay();
    }
    ledger.endTime = p.time;
    ledger.finalPhysicalEnergy = observed.totalEnergy;
    ledger.exactJouleEnergy = joule.value();
    ledger.representedElectricEnergyLoss = represented.value();
    ledger.wavePhysicalEnergyChange = wave.value();
    ledger.modifiedEnergyDissipation = modified.value();
    ledger.decayStorageEnergyChange =
        Checked(ledger.exactJouleEnergy - ledger.representedElectricEnergyLoss);
    ledger.physicalBalanceResidual =
        Balance(ledger.finalPhysicalEnergy, ledger.initialPhysicalEnergy, ledger.exactJouleEnergy,
                ledger.wavePhysicalEnergyChange);
    ledger.substep = p.h;
    ledger.substeps = p.count;
    ledger.cellVisits = state_.ez.size() * (8 * p.count + 1);
    observed.time = p.time;
    observed.lastSubstep = p.h;
    observed.lastSubsteps = p.count;
    observed.lastCellVisits = ledger.cellVisits;
    state_.ez.swap(next.ez);
    state_.hx.swap(next.hx);
    state_.hy.swap(next.hy);
    diagnostics_ = observed;
    return ledger;
}
} // namespace PhysicsEngine

#include "physics/core/elastic_wave_grid.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace PhysicsEngine {
namespace {
double Checked(double x) {
    if (!std::isfinite(x))
        throw std::overflow_error("Elastic wave arithmetic exceeds float64 range.");
    return x;
}
double Positive(double x) {
    if (!std::isfinite(x) || x <= 0)
        throw std::invalid_argument("Elastic wave coefficient is not positively representable.");
    return x;
}
// Scale before accumulating, rather than dividing each sample by the cell count.
// Neumaier compensation retains small terms exposed by signed cancellation.
class ScaledMean {
    double scale_ = 0, sum_ = 0, correction_ = 0;
    static double RetainedProduct(double a, double b) {
        const double result = Checked(a * b);
        if (a != 0 && b != 0 && result == 0)
            throw std::overflow_error("Elastic mean normalization underflows.");
        return result;
    }

  public:
    void add(double x) {
        if (x == 0)
            return;
        const double magnitude = std::abs(x);
        if (magnitude > scale_) {
            if (scale_ != 0) {
                const double factor = scale_ / magnitude;
                if (factor == 0 && (sum_ != 0 || correction_ != 0))
                    throw std::overflow_error("Elastic mean rescaling erases a contribution.");
                sum_ = RetainedProduct(sum_, factor);
                correction_ = RetainedProduct(correction_, factor);
            }
            scale_ = magnitude;
        }
        const double term = x / scale_;
        if (term == 0)
            throw std::overflow_error("Elastic mean normalization erases a contribution.");
        const double next = sum_ + term;
        correction_ +=
            std::abs(sum_) >= std::abs(term) ? (sum_ - next) + term : (term - next) + sum_;
        sum_ = next;
    }
    double value(std::size_t count) const {
        if (scale_ == 0)
            return 0;
        const double normalized = Checked(sum_ + correction_);
        if (normalized == 0)
            return 0;
        const double quotient = normalized / double(count);
        const double result = quotient != 0 ? Checked(scale_ * quotient)
                                            : Checked((scale_ / double(count)) * normalized);
        if (result == 0)
            throw std::overflow_error("Elastic mean underflows float64 range.");
        return result;
    }
};
struct Coefficients {
    double ix, iy, ir, b, ib, im, kinetic, trace, deviator, shear, rateTrace, rateShear, cp, cs,
        rate, limit, zz;
};
Coefficients Validate(const ElasticWaveGridConfig &c) {
    if (c.columns < 2 || c.rows < 2 || c.columns > ElasticWaveGridConfig::MaximumCells / c.rows)
        throw std::length_error("Elastic wave dimensions exceed the hard cell budget.");
    if (!std::isfinite(c.lambda) || !std::isfinite(c.cflSafety) || c.cflSafety <= 0 ||
        c.cflSafety >= 1 || !c.maximumSubsteps ||
        c.maximumSubsteps > ElasticWaveGridConfig::MaximumSubsteps || !c.maximumCellVisits ||
        c.maximumCellVisits > ElasticWaveGridConfig::MaximumCellVisits)
        throw std::invalid_argument("Invalid elastic wave material, CFL or work budget.");
    Positive(c.spacingX);
    Positive(c.spacingY);
    Positive(c.density);
    Positive(c.shearModulus);
    Positive(c.maxSubstep);
    // Stable underlying 3D isotropic material, including lambda<0 auxetics.
    Positive(c.lambda + c.shearModulus * (2. / 3));
    const double b = Positive(c.lambda + c.shearModulus), p = Positive(b + c.shearModulus);
    const double ix = Positive(1 / c.spacingX), iy = Positive(1 / c.spacingY),
                 ir = Positive(1 / c.density);
    const double ib = Positive(1 / b), im = Positive(1 / c.shearModulus);
    const double area = Positive(c.spacingX * c.spacingY);
    Positive(c.columns * c.spacingX);
    Positive(c.rows * c.spacingY);
    Positive(ix * ix);
    Positive(iy * iy);
    Positive(ix * iy);
    // Derivative coefficients required by the represented update/diagnostics.
    for (double d : {ix, iy}) {
        Positive(ir * d);
        Positive(b * d);
        Positive(c.shearModulus * d);
    }
    const double kinetic = std::sqrt(Positive(.5 * Positive(c.density * area)));
    const double trace = std::sqrt(Positive((area * ib) * .125));
    const double deviator = std::sqrt(Positive((area * im) * .125));
    const double shear = std::sqrt(Positive((area * im) * .5));
    const double rt = std::sqrt(Positive(.5 * Positive(area * b)));
    const double rs = std::sqrt(Positive(.5 * Positive(area * c.shearModulus)));
    const double cp = std::sqrt(Positive(p / c.density)),
                 cs = std::sqrt(Positive(c.shearModulus / c.density));
    const double rate = Positive(cp * Positive(std::hypot(ix, iy)));
    const double limit =
        Positive(std::min(c.maxSubstep, std::nextafter(Positive(c.cflSafety / rate), 0.)));
    return {ix, iy, ir, b,  ib,   im,    kinetic,          trace, deviator, shear,
            rt, rs, cp, cs, rate, limit, c.lambda / b * .5};
}
std::size_t Index(std::size_t i, std::size_t j, std::size_t nx) { return i + nx * j; }
double Energy(double norm, bool nonzero) {
    const double e = Checked(norm * norm);
    if (nonzero && e == 0)
        throw std::overflow_error("Elastic wave energy underflows float64 range.");
    return e;
}
} // namespace
void ElasticWaveGridConfig::Validate() const { PhysicsEngine::Validate(*this); }
ElasticWaveGrid::ElasticWaveGrid(const ElasticWaveGridConfig &c) : config_(c) {
    const auto a = Validate(c);
    ix_ = a.ix;
    iy_ = a.iy;
    inverseDensity_ = a.ir;
    bulk2_ = a.b;
    inverseBulk2_ = a.ib;
    inverseShear_ = a.im;
    kineticScale_ = a.kinetic;
    traceScale_ = a.trace;
    deviatorScale_ = a.deviator;
    shearScale_ = a.shear;
    rateTraceScale_ = a.rateTrace;
    rateShearScale_ = a.rateShear;
    cp_ = a.cp;
    cs_ = a.cs;
    rate_ = a.rate;
    limit_ = a.limit;
    zzFactor_ = a.zz;
    const auto n = c.columns * c.rows;
    for (auto *v : {&state_.vx, &state_.vy, &state_.sigmaXX, &state_.sigmaYY, &state_.sigmaXY})
        v->resize(n);
    diagnostics_ = measure(state_, 0);
}
ElasticWaveGrid::Strain ElasticWaveGrid::strainRate(const ElasticWaveState &s, std::size_t i,
                                                    std::size_t j) const {
    const auto nx = config_.columns, ny = config_.rows, k = Index(i, j, nx);
    const double xx = Checked(Checked(s.vx[Index((i + 1) % nx, j, nx)] - s.vx[k]) * ix_);
    const double yy = Checked(Checked(s.vy[Index(i, (j + 1) % ny, nx)] - s.vy[k]) * iy_);
    const double xy =
        Checked(Checked(Checked(s.vx[k] - s.vx[Index(i, (j + ny - 1) % ny, nx)]) * iy_) +
                Checked(Checked(s.vy[k] - s.vy[Index((i + nx - 1) % nx, j, nx)]) * ix_));
    return {xx, yy, xy};
}
ElasticWaveGrid::Strain ElasticWaveGrid::strain(const ElasticWaveState &s, std::size_t k) const {
    const double trace = Checked(Checked(s.sigmaXX[k] * (.25 * inverseBulk2_)) +
                                 Checked(s.sigmaYY[k] * (.25 * inverseBulk2_)));
    const double deviator = Checked(Checked(s.sigmaXX[k] * (.25 * inverseShear_)) -
                                    Checked(s.sigmaYY[k] * (.25 * inverseShear_)));
    return {Checked(trace + deviator), Checked(trace - deviator),
            Checked(s.sigmaXY[k] * inverseShear_)};
}
double ElasticWaveGrid::compatibilityAt(const ElasticWaveState &s, std::size_t i,
                                        std::size_t j) const {
    const auto nx = config_.columns, ny = config_.rows, k = Index(i, j, nx);
    const auto c = strain(s, k);
    const double yy =
        Checked(Checked(Checked(strain(s, Index(i, (j + 1) % ny, nx)).xx - c.xx) +
                        Checked(strain(s, Index(i, (j + ny - 1) % ny, nx)).xx - c.xx)) *
                iy_ * iy_);
    const double xx =
        Checked(Checked(Checked(strain(s, Index((i + 1) % nx, j, nx)).yy - c.yy) +
                        Checked(strain(s, Index((i + nx - 1) % nx, j, nx)).yy - c.yy)) *
                ix_ * ix_);
    const double mixed =
        Checked(Checked(Checked(strain(s, Index((i + 1) % nx, (j + 1) % ny, nx)).xy -
                                strain(s, Index(i, (j + 1) % ny, nx)).xy) -
                        Checked(strain(s, Index((i + 1) % nx, j, nx)).xy - c.xy)) *
                ix_ * iy_);
    return Checked(Checked(xx + yy) - mixed);
}
ElasticWaveDiagnostics ElasticWaveGrid::measure(const ElasticWaveState &s, double h) const {
    ElasticWaveDiagnostics d;
    d.stableTimeStep = limit_;
    d.modifiedEnergyStep = h;
    const auto nx = config_.columns, ny = config_.rows, n = nx * ny;
    const double rootN = std::sqrt(double(n));
    double kv = 0, sv = 0, correction = 0;
    ScaledMean vxMean, vyMean, xxMean, yyMean, xyMean, zzMean;
    bool kinetic = false, stress = false, rate = false;
    for (std::size_t j = 0; j < ny; ++j)
        for (std::size_t i = 0; i < nx; ++i) {
            const auto k = Index(i, j, nx);
            for (double v : {s.vx[k], s.vy[k]})
                kv = Checked(std::hypot(kv, Checked(kineticScale_ * v)));
            const double trace =
                Checked(Checked(traceScale_ * s.sigmaXX[k]) + Checked(traceScale_ * s.sigmaYY[k]));
            const double dev = Checked(Checked(deviatorScale_ * s.sigmaXX[k]) -
                                       Checked(deviatorScale_ * s.sigmaYY[k]));
            for (double v : {trace, dev, Checked(shearScale_ * s.sigmaXY[k])})
                sv = Checked(std::hypot(sv, v));
            kinetic = kinetic || s.vx[k] != 0 || s.vy[k] != 0;
            stress = stress || s.sigmaXX[k] != 0 || s.sigmaYY[k] != 0 || s.sigmaXY[k] != 0;
            vxMean.add(s.vx[k]);
            vyMean.add(s.vy[k]);
            xxMean.add(s.sigmaXX[k]);
            yyMean.add(s.sigmaYY[k]);
            xyMean.add(s.sigmaXY[k]);
            d.maxAbsVelocity = std::max({d.maxAbsVelocity, std::abs(s.vx[k]), std::abs(s.vy[k])});
            d.maxAbsStress = std::max({d.maxAbsStress, std::abs(s.sigmaXX[k]),
                                       std::abs(s.sigmaYY[k]), std::abs(s.sigmaXY[k])});
            const double zz =
                Checked(Checked(zzFactor_ * s.sigmaXX[k]) + Checked(zzFactor_ * s.sigmaYY[k]));
            zzMean.add(zz);
            d.maxAbsSigmaZZ = std::max(d.maxAbsSigmaZZ, std::abs(zz));
            const double defect = compatibilityAt(s, i, j);
            d.compatibilityRms = Checked(std::hypot(d.compatibilityRms, defect / rootN));
            d.maxAbsCompatibility = std::max(d.maxAbsCompatibility, std::abs(defect));
            if (h > 0) {
                const auto e = strainRate(s, i, j);
                rate = rate || e.xx != 0 || e.yy != 0 || e.xy != 0;
                const double t = (.5 * h) * rateTraceScale_, u = (.5 * h) * rateShearScale_;
                const double a = Checked(Checked(t * e.xx) + Checked(t * e.yy));
                const double b = Checked(Checked(u * e.xx) - Checked(u * e.yy));
                const double c = Checked(u * e.xy);
                for (double v : {a, b, c})
                    correction = Checked(std::hypot(correction, v));
            }
        }
    d.meanVx = vxMean.value(n);
    d.meanVy = vyMean.value(n);
    d.meanSigmaXX = xxMean.value(n);
    d.meanSigmaYY = yyMean.value(n);
    d.meanSigmaXY = xyMean.value(n);
    d.meanSigmaZZ = zzMean.value(n);
    if (d.maxAbsCompatibility > 0 && d.compatibilityRms == 0)
        throw std::overflow_error("Elastic compatibility RMS underflows.");
    d.kineticEnergy = Energy(kv, kinetic);
    d.strainEnergy = Energy(sv, stress);
    d.totalEnergy = Checked(d.kineticEnergy + d.strainEnergy);
    d.modifiedEnergy = Checked(d.totalEnergy - Energy(correction, rate));
    if (d.modifiedEnergy < 0 || (d.totalEnergy > 0 && d.modifiedEnergy == 0))
        throw std::overflow_error("Elastic modified energy is not positively representable.");
    d.physicalEnergyUpperBound = Checked(d.modifiedEnergy / (1 - (h * rate_) * (h * rate_)));
    return d;
}
void ElasticWaveGrid::setState(const ElasticWaveState &s) {
    const auto n = state_.vx.size();
    for (const auto *v : {&s.vx, &s.vy, &s.sigmaXX, &s.sigmaYY, &s.sigmaXY}) {
        if (v->size() != n)
            throw std::invalid_argument("Elastic field array size mismatch.");
        for (double x : *v)
            if (!std::isfinite(x))
                throw std::invalid_argument("Elastic fields must be finite.");
    }
    auto next = s;
    auto d = measure(next, 0);
    d.time = diagnostics_.time;
    state_.vx.swap(next.vx);
    state_.vy.swap(next.vy);
    state_.sigmaXX.swap(next.sigmaXX);
    state_.sigmaYY.swap(next.sigmaYY);
    state_.sigmaXY.swap(next.sigmaXY);
    diagnostics_ = d;
}
std::vector<double> ElasticWaveGrid::getCompatibility() const {
    std::vector<double> out(state_.vx.size());
    for (std::size_t j = 0; j < config_.rows; ++j)
        for (std::size_t i = 0; i < config_.columns; ++i)
            out[Index(i, j, config_.columns)] = compatibilityAt(state_, i, j);
    return out;
}
std::vector<double> ElasticWaveGrid::getOutOfPlaneStress() const {
    std::vector<double> out(state_.vx.size());
    for (std::size_t k = 0; k < out.size(); ++k)
        out[k] = Checked(Checked(zzFactor_ * state_.sigmaXX[k]) +
                         Checked(zzFactor_ * state_.sigmaYY[k]));
    return out;
}
ElasticWaveSpatialRates ElasticWaveGrid::getSpatialRates() const {
    ElasticWaveSpatialRates out;
    const auto nx = config_.columns, ny = config_.rows, n = nx * ny;
    for (auto *v : {&out.accelerationX, &out.accelerationY, &out.strainRateXX, &out.strainRateYY,
                    &out.engineeringShearRate})
        v->resize(n);
    for (std::size_t j = 0; j < ny; ++j)
        for (std::size_t i = 0; i < nx; ++i) {
            const auto k = Index(i, j, nx);
            const auto e = strainRate(state_, i, j);
            out.strainRateXX[k] = e.xx;
            out.strainRateYY[k] = e.yy;
            out.engineeringShearRate[k] = e.xy;
            const double x = inverseDensity_ * ix_, y = inverseDensity_ * iy_;
            out.accelerationX[k] =
                Checked(Checked(x * Checked(state_.sigmaXX[k] -
                                            state_.sigmaXX[Index((i + nx - 1) % nx, j, nx)])) +
                        Checked(y * Checked(state_.sigmaXY[Index(i, (j + 1) % ny, nx)] -
                                            state_.sigmaXY[k])));
            out.accelerationY[k] =
                Checked(Checked(x * Checked(state_.sigmaXY[Index((i + 1) % nx, j, nx)] -
                                            state_.sigmaXY[k])) +
                        Checked(y * Checked(state_.sigmaYY[k] -
                                            state_.sigmaYY[Index(i, (j + ny - 1) % ny, nx)])));
        }
    return out;
}
double ElasticWaveGrid::getModifiedEnergy(double h) const {
    if (!std::isfinite(h) || h < 0 || (h > 0 && !(h * rate_ < 1)))
        throw std::invalid_argument("Elastic reference step must satisfy strict CFL.");
    if (h > 0 && !(.5 * h > 0))
        throw std::overflow_error("Elastic reference half-step underflows.");
    return measure(state_, h).modifiedEnergy;
}
void ElasticWaveGrid::step(double dt) {
    if (!std::isfinite(dt) || dt < 0)
        throw std::invalid_argument("Elastic timestep must be finite and nonnegative.");
    if (dt == 0)
        return;
    const double requested = std::ceil(dt / limit_);
    if (!std::isfinite(requested) || requested > double(config_.maximumSubsteps))
        throw std::length_error("Elastic substep budget exceeded.");
    auto count = static_cast<std::size_t>(std::max(1., requested));
    const auto n = state_.vx.size(), passes = config_.maximumCellVisits / n;
    if (passes < 4 || count > (passes - 1) / 3)
        throw std::length_error("Elastic cell-visit budget exceeded.");
    const auto budget = std::min(config_.maximumSubsteps, (passes - 1) / 3);
    double h = dt / count;
    if (h > limit_) {
        if (count >= budget)
            throw std::length_error("Elastic rounded substep budget exceeded.");
        h = dt / ++count;
    }
    const double half = .5 * h;
    for (double x :
         {h, half, half * bulk2_ * ix_, half * bulk2_ * iy_, half * config_.shearModulus * ix_,
          half * config_.shearModulus * iy_, h * inverseDensity_ * ix_, h * inverseDensity_ * iy_})
        if (!std::isfinite(x) || x <= 0)
            throw std::overflow_error("Elastic substep coefficient is unrepresentable.");
    if (h > limit_ || !(h * rate_ < 1))
        throw std::overflow_error("Elastic substep violates strict CFL.");
    const double time = Checked(diagnostics_.time + dt);
    if (time == diagnostics_.time)
        throw std::overflow_error("Elastic clock increment is unrepresentable.");
    auto next = state_;
    const auto nx = config_.columns, ny = config_.rows;
    auto kick = [&]() {
        for (std::size_t j = 0; j < ny; ++j)
            for (std::size_t i = 0; i < nx; ++i) {
                const auto k = Index(i, j, nx);
                const auto e = strainRate(next, i, j);
                const double t = half * bulk2_, u = half * config_.shearModulus;
                const double trace = Checked(Checked(t * e.xx) + Checked(t * e.yy)),
                             dev = Checked(Checked(u * e.xx) - Checked(u * e.yy));
                next.sigmaXX[k] = Checked(next.sigmaXX[k] + Checked(trace + dev));
                next.sigmaYY[k] = Checked(next.sigmaYY[k] + Checked(trace - dev));
                next.sigmaXY[k] = Checked(next.sigmaXY[k] + Checked(u * e.xy));
            }
    };
    for (std::size_t k = 0; k < count; ++k) {
        kick();
        for (std::size_t j = 0; j < ny; ++j)
            for (std::size_t i = 0; i < nx; ++i) {
                const auto a = Index(i, j, nx);
                const double cx = h * inverseDensity_ * ix_, cy = h * inverseDensity_ * iy_;
                const double x = Checked(
                    Checked(
                        Checked(next.sigmaXX[a] - next.sigmaXX[Index((i + nx - 1) % nx, j, nx)]) *
                        cx) +
                    Checked(Checked(next.sigmaXY[Index(i, (j + 1) % ny, nx)] - next.sigmaXY[a]) *
                            cy));
                const double y = Checked(
                    Checked(Checked(next.sigmaXY[Index((i + 1) % nx, j, nx)] - next.sigmaXY[a]) *
                            cx) +
                    Checked(
                        Checked(next.sigmaYY[a] - next.sigmaYY[Index(i, (j + ny - 1) % ny, nx)]) *
                        cy));
                next.vx[a] = Checked(next.vx[a] + x);
                next.vy[a] = Checked(next.vy[a] + y);
            }
        kick();
    }
    auto d = measure(next, h);
    d.time = time;
    d.lastSubstep = h;
    d.lastSubsteps = count;
    d.lastCellVisits = n * (3 * count + 1);
    state_.vx.swap(next.vx);
    state_.vy.swap(next.vy);
    state_.sigmaXX.swap(next.sigmaXX);
    state_.sigmaYY.swap(next.sigmaYY);
    state_.sigmaXY.swap(next.sigmaXY);
    diagnostics_ = d;
}
} // namespace PhysicsEngine

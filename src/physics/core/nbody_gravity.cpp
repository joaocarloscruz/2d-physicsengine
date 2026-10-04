#include "physics/core/nbody_gravity.h"

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <limits>
#include <stdexcept>
#include <utility>

namespace PhysicsEngine {
namespace {
void validate(Vector2d v) {
    if (!std::isfinite(v.x) || !std::isfinite(v.y))
        throw std::invalid_argument("N-body coordinates and impulses must be finite");
}
void validateConfig(const NBodyGravityConfig& c) {
    if (!std::isfinite(c.gravitationalStrength) || c.gravitationalStrength < 0 ||
        !std::isfinite(c.softening) || c.softening < 0 ||
        !std::isfinite(c.maxSubstep) || c.maxSubstep <= 0 ||
        !std::isfinite(c.frequencySafety) || c.frequencySafety <= 0 || c.frequencySafety > 1 ||
        c.maxParticles == 0 || c.maxSubsteps == 0 || c.maxPairWork == 0)
        throw std::invalid_argument("Invalid N-body gravity configuration");
}
double checked(double v) {
    if (!std::isfinite(v)) throw std::overflow_error("N-body arithmetic overflow");
    return v;
}
struct Scaled { double mantissa; int exponent; };
Scaled scaledProduct(std::initializer_list<double> factors, std::initializer_list<double> divisors = {}) {
    double mantissa = 1;
    int exponent = 0;
    for (const double factor : factors) {
        if (factor == 0) return {0, 0};
        int e;
        mantissa *= std::frexp(factor, &e);
        exponent += e;
    }
    for (const double divisor : divisors) {
        int e;
        mantissa /= std::frexp(divisor, &e);
        exponent -= e;
    }
    return {mantissa, exponent};
}
double product(std::initializer_list<double> factors, std::initializer_list<double> divisors = {}) {
    const auto value = scaledProduct(factors, divisors);
    return checked(std::scalbn(value.mantissa, value.exponent));
}
// Accumulate nonnegative energy terms before rounding subnormal components to
// double. Compensation also reduces loss when many small terms follow large ones.
struct PositiveSum {
    double mantissa = 0, correction = 0;
    int exponent = 0;
    void add(Scaled term) {
        if (term.mantissa == 0) return;
        int shift;
        term.mantissa = std::frexp(term.mantissa, &shift);
        term.exponent += shift;
        if (mantissa == 0) { mantissa = term.mantissa; exponent = term.exponent; return; }
        if (term.exponent > exponent) {
            mantissa = std::scalbn(mantissa, exponent - term.exponent);
            correction = std::scalbn(correction, exponent - term.exponent);
            exponent = term.exponent;
        }
        const double value = std::scalbn(term.mantissa, term.exponent - exponent) - correction;
        const double sum = mantissa + value;
        correction = (sum - mantissa) - value;
        mantissa = sum;
    }
    double value() const { return checked(std::scalbn(mantissa, exponent)); }
};
struct WorkBudget {
    std::size_t limit;
    std::size_t used = 0;
    void visit() {
        if (used == limit) throw std::runtime_error("N-body total pair-work budget exceeded");
        ++used;
    }
};
struct Separation { Vector2d delta; double radius; };
Separation separation(const GravityParticle& a, const GravityParticle& b, double softening) {
    const Vector2d delta{checked(b.position.x - a.position.x), checked(b.position.y - a.position.y)};
    const double radius = checked(std::hypot(std::hypot(delta.x, delta.y), softening));
    if (radius == 0) throw std::runtime_error("N-body unsoftened coincidence is singular");
    return {delta, radius};
}
struct Evaluation { std::vector<Vector2d> acceleration; double frequencySquared = 0; };
Evaluation evaluate(const std::vector<GravityParticle>& state, const NBodyGravityConfig& c, WorkBudget& budget) {
    Evaluation result{std::vector<Vector2d>(state.size()), 0};
    if (c.gravitationalStrength == 0) return result;
    std::vector<double> rowBound(state.size(), 0);
    for (std::size_t i = 0; i < state.size(); ++i) {
        for (std::size_t j = i + 1; j < state.size(); ++j) {
            budget.visit();
            const auto s = separation(state[i], state[j], c.softening);
            // Apply the same central pair force divided by each endpoint mass,
            // without first rounding a possibly subnormal force to zero.
            const auto acceleration = [&](double component, double mass) {
                return product({c.gravitationalStrength, state[i].mass, state[j].mass, component},
                               {s.radius, s.radius, s.radius, mass});
            };
            result.acceleration[i].x = checked(result.acceleration[i].x + acceleration(s.delta.x, state[i].mass));
            result.acceleration[i].y = checked(result.acceleration[i].y + acceleration(s.delta.y, state[i].mass));
            result.acceleration[j].x = checked(result.acceleration[j].x - acceleration(s.delta.x, state[j].mass));
            result.acceleration[j].y = checked(result.acceleration[j].y - acceleration(s.delta.y, state[j].mass));
            // The softened force Jacobian norm is <= 2*G*m/rho^3; diagonal
            // and off-diagonal blocks give the conservative factor four.
            rowBound[i] = checked(rowBound[i] + product({4, c.gravitationalStrength, state[j].mass}, {s.radius, s.radius, s.radius}));
            rowBound[j] = checked(rowBound[j] + product({4, c.gravitationalStrength, state[i].mass}, {s.radius, s.radius, s.radius}));
        }
    }
    if (!rowBound.empty()) result.frequencySquared = *std::max_element(rowBound.begin(), rowBound.end());
    return result;
}
bool safeDrift(const std::vector<GravityParticle>& before, const std::vector<GravityParticle>& after,
               const NBodyGravityConfig& c, WorkBudget& budget) {
    if (c.gravitationalStrength == 0) return true;
    for (std::size_t i = 0; i < before.size(); ++i) {
        for (std::size_t j = i + 1; j < before.size(); ++j) {
            budget.visit();
            const auto old = separation(before[i], before[j], c.softening);
            const Vector2d next{checked(after[j].position.x - after[i].position.x), checked(after[j].position.y - after[i].position.y)};
            const double change = checked(std::hypot(checked(next.x - old.delta.x), checked(next.y - old.delta.y)));
            if (change > 0.25 * old.radius) return false;
        }
    }
    return true;
}
GravityDiagnostics diagnostics(const std::vector<GravityParticle>& state,
                               const NBodyGravityConfig& c, WorkBudget& budget) {
    GravityDiagnostics result;
    PositiveSum kinetic, potential;
    for (const auto& p : state) result.totalMass = checked(result.totalMass + p.mass);
    for (const auto& p : state) {
        result.centerOfMass.x = checked(result.centerOfMass.x + product({p.mass, p.position.x}, {result.totalMass}));
        result.centerOfMass.y = checked(result.centerOfMass.y + product({p.mass, p.position.y}, {result.totalMass}));
        result.momentum.x = checked(result.momentum.x + product({p.mass, p.velocity.x}));
        result.momentum.y = checked(result.momentum.y + product({p.mass, p.velocity.y}));
        result.angularMomentum = checked(result.angularMomentum + checked(
            product({p.mass, p.position.x, p.velocity.y}) - product({p.mass, p.position.y, p.velocity.x})));
        const double speedScale = std::max(std::abs(p.velocity.x), std::abs(p.velocity.y));
        if (speedScale > 0) {
            const double x = p.velocity.x / speedScale, y = p.velocity.y / speedScale;
            kinetic.add(scaledProduct({p.mass, speedScale, speedScale, x*x + y*y}, {2}));
        }
    }
    if (c.gravitationalStrength != 0) {
        for (std::size_t i = 0; i < state.size(); ++i) {
            for (std::size_t j = i + 1; j < state.size(); ++j) {
                budget.visit();
                const auto s = separation(state[i], state[j], c.softening);
                potential.add(scaledProduct({c.gravitationalStrength, state[i].mass, state[j].mass}, {s.radius}));
            }
        }
    }
    result.kineticEnergy = kinetic.value();
    result.potentialEnergy = -potential.value();
    result.totalEnergy = checked(result.kineticEnergy + result.potentialEnergy);
    return result;
}
} // namespace

NBodyGravity::NBodyGravity(const NBodyGravityConfig& config) : config_(config) { validateConfig(config); }
std::size_t NBodyGravity::addParticle(Vector2d position, Vector2d velocity, double mass) {
    validate(position); validate(velocity);
    if (!std::isfinite(mass) || mass <= 0) throw std::invalid_argument("N-body mass must be finite and positive");
    if (particles_.size() >= config_.maxParticles) throw std::length_error("N-body particle budget exceeded");
    particles_.push_back({position, velocity, mass});
    return particles_.size() - 1;
}
void NBodyGravity::setState(std::size_t index, Vector2d position, Vector2d velocity) {
    auto& p = particles_.at(index);
    validate(position); validate(velocity);
    p.position = position;
    p.velocity = velocity;
}
void NBodyGravity::applyImpulse(std::size_t index, Vector2d impulse) {
    auto& p = particles_.at(index);
    validate(impulse);
    const Vector2d velocity{checked(p.velocity.x + product({impulse.x}, {p.mass})),
                           checked(p.velocity.y + product({impulse.y}, {p.mass}))};
    p.velocity = velocity;
}
void NBodyGravity::setConfig(const NBodyGravityConfig& config) {
    validateConfig(config);
    if (particles_.size() > config.maxParticles) throw std::length_error("N-body configuration excludes existing particles");
    config_ = config;
}
GravityDiagnostics NBodyGravity::getDiagnostics() const {
    WorkBudget budget{config_.maxPairWork};
    auto result = diagnostics(particles_, config_, budget);
    result.lastSubsteps = lastSubsteps_;
    result.lastPairWork = lastPairWork_;
    return result;
}

void NBodyGravity::step(double dt) {
    if (!std::isfinite(dt) || dt < 0) throw std::invalid_argument("N-body timestep must be finite and nonnegative");
    if (dt == 0 || particles_.empty()) { lastSubsteps_ = lastPairWork_ = 0; return; }
    const double requested = dt / config_.maxSubstep;
    const double tolerance = 64 * std::numeric_limits<double>::epsilon() * std::max(1.0, requested);
    if (!std::isfinite(requested) || requested > static_cast<double>(config_.maxSubsteps) + tolerance)
        throw std::runtime_error("N-body substep budget exceeded");
    WorkBudget budget{config_.maxPairWork};
    auto state = particles_;
    auto current = evaluate(state, config_, budget);
    double elapsed = 0;
    double preferredH = 0;
    std::size_t substeps = 0;
    while (elapsed < dt) {
        const double remaining = dt - elapsed;
        const double limit = current.frequencySquared > 0
            ? std::min(config_.maxSubstep, std::nextafter(
                config_.frequencySafety / std::sqrt(current.frequencySquared), 0.0)) : config_.maxSubstep;
        if (!(limit > 0)) throw std::runtime_error("N-body local timestep is below representable precision");
        if (preferredH == 0 || preferredH > limit) {
            const double ratio = remaining / limit;
            const double roundoff = 64 * std::numeric_limits<double>::epsilon() * std::max(1.0, ratio);
            if (!std::isfinite(ratio) ||
                ratio > static_cast<double>(config_.maxSubsteps - substeps) + roundoff)
                throw std::runtime_error("N-body local encounter exceeds substep budget");
            double count = std::max(1.0, std::ceil(ratio - roundoff));
            double candidate = remaining / count;
            // Tolerance chooses an integer partition only. The actual physical
            // duration must still satisfy the representable limit exactly.
            if (candidate > limit) {
                count = std::max(count + 1, std::ceil(ratio));
                candidate = remaining / count;
            }
            if (count > static_cast<double>(config_.maxSubsteps - substeps) || candidate > limit)
                throw std::runtime_error("N-body representable partition exceeds substep budget");
            preferredH = candidate;
        }
        double h = std::min(remaining, preferredH);
        if (substeps >= config_.maxSubsteps || !(h > 0) || elapsed + h == elapsed)
            throw std::runtime_error("N-body timestep precision or substep budget exhausted");
        std::vector<GravityParticle> trial;
        bool accepted = false;
        for (unsigned attempt = 0; attempt < 32; ++attempt) {
            trial = state;
            for (std::size_t i = 0; i < state.size(); ++i) {
                trial[i].velocity = {checked(state[i].velocity.x + product({current.acceleration[i].x, h}, {2})),
                                     checked(state[i].velocity.y + product({current.acceleration[i].y, h}, {2}))};
                trial[i].position = {checked(state[i].position.x + product({trial[i].velocity.x, h})),
                                     checked(state[i].position.y + product({trial[i].velocity.y, h}))};
            }
            if (safeDrift(state, trial, config_, budget)) { accepted = true; break; }
            h *= 0.5;
            if (!(h > 0) || elapsed + h == elapsed ||
                remaining / h > static_cast<double>(config_.maxSubsteps - substeps))
                throw std::runtime_error("N-body unresolved close encounter exceeds substep budget");
        }
        if (!accepted) throw std::runtime_error("N-body close-encounter trial budget exceeded");
        auto next = evaluate(trial, config_, budget);
        for (std::size_t i = 0; i < trial.size(); ++i) {
            trial[i].velocity.x = checked(trial[i].velocity.x + product({next.acceleration[i].x, h}, {2}));
            trial[i].velocity.y = checked(trial[i].velocity.y + product({next.acceleration[i].y, h}, {2}));
        }
        state.swap(trial);
        current = std::move(next);
        // Retain a uniform duration until a tighter local limit or trial guard
        // requires a smaller one. Summing elapsed time avoids turning repeated
        // decimal remainder subtraction into a spurious extra partition.
        preferredH = h;
        elapsed = h == remaining ? dt : elapsed + h;
        ++substeps;
    }
    diagnostics(state, config_, budget); // Also validates aggregate representability.
    particles_.swap(state);
    lastSubsteps_ = substeps;
    lastPairWork_ = budget.used;
}
} // namespace PhysicsEngine

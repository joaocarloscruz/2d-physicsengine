#include "physics/core/softbody.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace PhysicsEngine {
namespace {
struct Vec {
    double x, y;
    Vec operator+(Vec v) const { return {x + v.x, y + v.y}; }
    Vec operator-(Vec v) const { return {x - v.x, y - v.y}; }
    Vec operator*(double s) const { return {x * s, y * s}; }
    double dot(Vec v) const { return x * v.x + y * v.y; }
};
struct State { Vec position, velocity; double inverseMass; };
bool finite(const Vector2& v) { return std::isfinite(v.x) && std::isfinite(v.y); }
bool finite(Vec v) { return std::isfinite(v.x) && std::isfinite(v.y); }
void validateConfig(const SoftBodyConfig& c) {
    if (!std::isfinite(c.maxSubstep) || c.maxSubstep <= 0.0 ||
        !std::isfinite(c.stabilityFactor) || c.stabilityFactor <= 0.0 ||
        c.stabilityFactor > 1.0 || c.maxSubsteps == 0 ||
        c.maxParticles == 0 || c.maxSprings == 0)
        throw std::invalid_argument("Invalid soft-body integration configuration");
}
Vector2 checkedVector(Vec v) {
    const double max = std::numeric_limits<float>::max();
    if (!finite(v) || std::abs(v.x) > max || std::abs(v.y) > max)
        throw std::runtime_error("Soft-body state overflow");
    return {static_cast<float>(v.x), static_cast<float>(v.y)};
}
double lengthAndDirection(const State& a, const State& b,
                          const SoftBodySpring& spring, Vec& direction) {
    const Vec delta = b.position - a.position;
    const double length = std::hypot(delta.x, delta.y);
    if (!std::isfinite(length)) throw std::runtime_error("Soft-body spring overflow");
    if (length == 0.0) {
        if (spring.restLength > 0.0 && (spring.stiffness > 0.0 || spring.damping > 0.0))
            throw std::runtime_error("Positive-rest soft-body spring collapsed");
        direction = {0.0, 0.0};
    } else {
        direction = delta * (1.0 / length);
    }
    return length;
}
std::vector<Vec> accelerations(const std::vector<State>& state,
                               const std::vector<SoftBodySpring>& springs, Vec uniform) {
    std::vector<Vec> result(state.size(), uniform);
    for (const auto& s : springs) {
        if (s.stiffness == 0.0) continue;
        Vec n;
        const double length = lengthAndDirection(state[s.first], state[s.second], s, n);
        const Vec force = n * (s.stiffness * (length - s.restLength));
        result[s.first] = result[s.first] + force * state[s.first].inverseMass;
        result[s.second] = result[s.second] - force * state[s.second].inverseMass;
    }
    for (const auto& a : result)
        if (!finite(a)) throw std::runtime_error("Soft-body acceleration overflow");
    return result;
}
// Gershgorin bound on the mass-scaled elastic Hessian. The transverse
// eigenvalue k*(1-rest/length) matters for strongly compressed springs.
double maximumSubstep(const std::vector<State>& state,
                      const std::vector<SoftBodySpring>& springs,
                      const SoftBodyConfig& config) {
    std::vector<double> rowSum(state.size(), 0.0);
    for (const auto& s : springs) {
        if (s.stiffness == 0.0 ||
            (state[s.first].inverseMass == 0.0 && state[s.second].inverseMass == 0.0)) continue;
        Vec n;
        const double length = lengthAndDirection(state[s.first], state[s.second], s, n);
        const double curvature = s.stiffness *
            (s.restLength == 0.0 ? 1.0 : std::max(1.0, std::abs(1.0 - s.restLength / length)));
        // This row bound also works in non mass-scaled coordinates; taking
        // twice each incident contribution conservatively includes off-diagonals.
        rowSum[s.first] += 2.0 * curvature * state[s.first].inverseMass;
        rowSum[s.second] += 2.0 * curvature * state[s.second].inverseMass;
    }
    const double bound = rowSum.empty() ? 0.0 : *std::max_element(rowSum.begin(), rowSum.end());
    if (!std::isfinite(bound)) throw std::runtime_error("Soft-body stiffness bound overflow");
    return bound > 0.0 ? std::min(config.maxSubstep, config.stabilityFactor / std::sqrt(bound))
                       : config.maxSubstep;
}
void dampSpring(std::vector<State>& state, const SoftBodySpring& s, double h) {
    if (s.damping == 0.0) return;
    auto& a = state[s.first];
    auto& b = state[s.second];
    const double w = a.inverseMass + b.inverseMass;
    if (w == 0.0) return;
    Vec n;
    lengthAndDirection(a, b, s, n);
    const double speed = (b.velocity - a.velocity).dot(n);
    // Exact frozen-position dashpot solve. expm1 avoids losing small impulses.
    const double impulse = speed * (-std::expm1(-s.damping * w * h)) / w;
    a.velocity = a.velocity + n * (impulse * a.inverseMass);
    b.velocity = b.velocity - n * (impulse * b.inverseMass);
}
bool safeDrift(const std::vector<State>& before, const std::vector<State>& after,
               const std::vector<SoftBodySpring>& springs) {
    for (const auto& s : springs) {
        if (s.restLength == 0.0 || (s.stiffness == 0.0 && s.damping == 0.0)) continue;
        const Vec separation = before[s.second].position - before[s.first].position;
        const Vec nextSeparation = after[s.second].position - after[s.first].position;
        const Vec change = nextSeparation - separation;
        if (std::hypot(change.x, change.y) > 0.25 * std::hypot(separation.x, separation.y)) return false;
    }
    return true;
}
} // namespace

SoftBody::SoftBody(const SoftBodyConfig& config) : config_(config) { validateConfig(config); }

std::size_t SoftBody::addParticle(const Vector2& position, const Vector2& velocity,
                                 double mass, bool fixed) {
    if (!finite(position) || !finite(velocity) || !std::isfinite(mass) || mass <= 0.0 ||
        !std::isfinite(1.0 / mass) || (fixed && (velocity.x != 0.0f || velocity.y != 0.0f)))
        throw std::invalid_argument("Invalid soft-body particle");
    if (particles_.size() >= config_.maxParticles) throw std::length_error("Soft-body particle budget exceeded");
    particles_.push_back({position, velocity, mass, fixed});
    return particles_.size() - 1;
}

std::size_t SoftBody::addSpring(std::size_t first, std::size_t second, double restLength,
                               double stiffness, double damping) {
    if (first >= particles_.size() || second >= particles_.size())
        throw std::out_of_range("Soft-body spring particle index");
    if (first == second || !std::isfinite(restLength) || restLength < 0.0 ||
        !std::isfinite(stiffness) || stiffness < 0.0 || !std::isfinite(damping) || damping < 0.0)
        throw std::invalid_argument("Invalid soft-body spring");
    if (restLength > 0.0 && (stiffness > 0.0 || damping > 0.0) &&
        particles_[first].position == particles_[second].position)
        throw std::invalid_argument("Positive-rest soft-body spring starts collapsed");
    const auto pair = std::minmax(first, second);
    if (springPairs_.count(pair)) throw std::invalid_argument("Duplicate soft-body spring");
    if (springs_.size() >= config_.maxSprings) throw std::length_error("Soft-body spring budget exceeded");
    const auto inserted = springPairs_.insert(pair);
    try { springs_.push_back({first, second, restLength, stiffness, damping}); }
    catch (...) { springPairs_.erase(inserted.first); throw; }
    return springs_.size() - 1;
}

void SoftBody::setParticleState(std::size_t index, const Vector2& position, const Vector2& velocity) {
    auto& particle = particles_.at(index);
    if (!finite(position) || !finite(velocity) ||
        (particle.fixed && (velocity.x != 0.0f || velocity.y != 0.0f)))
        throw std::invalid_argument("Invalid soft-body particle state");
    particle.position = position;
    particle.velocity = velocity;
}
void SoftBody::setFixed(std::size_t index, bool fixed) {
    auto& particle = particles_.at(index);
    particle.fixed = fixed;
    if (fixed) particle.velocity = {};
}
void SoftBody::applyImpulse(std::size_t index, const Vector2& impulse) {
    auto& particle = particles_.at(index);
    if (!finite(impulse)) throw std::invalid_argument("Invalid soft-body impulse");
    if (!particle.fixed) particle.velocity = checkedVector(
        Vec{particle.velocity.x, particle.velocity.y} + Vec{impulse.x, impulse.y} * (1.0 / particle.mass));
}
void SoftBody::setUniformAcceleration(const Vector2& acceleration) {
    if (!finite(acceleration)) throw std::invalid_argument("Invalid soft-body acceleration");
    acceleration_ = acceleration;
}
void SoftBody::setConfig(const SoftBodyConfig& config) {
    validateConfig(config);
    if (particles_.size() > config.maxParticles || springs_.size() > config.maxSprings)
        throw std::length_error("Soft-body configuration excludes existing topology");
    config_ = config;
}

void SoftBody::step(double dt) {
    if (!std::isfinite(dt) || dt < 0.0) throw std::invalid_argument("Invalid soft-body timestep");
    if (dt == 0.0 || particles_.empty()) { lastSubsteps_ = 0; return; }
    if (dt / config_.maxSubstep > static_cast<double>(config_.maxSubsteps))
        throw std::runtime_error("Soft-body substep budget exceeded");
    std::vector<State> state;
    state.reserve(particles_.size());
    for (const auto& p : particles_)
        state.push_back({{p.position.x, p.position.y}, {p.velocity.x, p.velocity.y}, p.fixed ? 0.0 : 1.0 / p.mass});
    double remaining = dt;
    std::size_t substeps = 0;
    while (remaining > 0.0) {
        const double limit = maximumSubstep(state, springs_, config_);
        const double ratio = remaining / limit;
        const double roundoff = 64.0 * std::numeric_limits<double>::epsilon() * std::max(1.0, ratio);
        if (!(limit > 0.0) || !std::isfinite(ratio) ||
            ratio > static_cast<double>(config_.maxSubsteps - substeps) + roundoff)
            throw std::runtime_error("Soft-body substep budget exceeded");
        // Equalize the currently required partition to avoid tiny final steps.
        const double count = std::max(1.0, std::ceil(ratio - roundoff));
        double h = remaining / count;
        if (substeps >= config_.maxSubsteps || !(h > 0.0) || remaining - h == remaining)
            throw std::runtime_error("Soft-body substep budget exceeded");
        std::vector<State> trial;
        bool accepted = false;
        // Bound retries as well as accepted work. A displacement guard prevents
        // a large imposed velocity from jumping across the central-force cusp.
        for (unsigned attempt = 0; attempt < 32; ++attempt) {
            trial = state;
            for (const auto& s : springs_) dampSpring(trial, s, h * 0.5);
            const auto a = accelerations(trial, springs_, {acceleration_.x, acceleration_.y});
            for (std::size_t i = 0; i < trial.size(); ++i) {
                if (trial[i].inverseMass == 0.0) continue;
                trial[i].velocity = trial[i].velocity + a[i] * (h * 0.5);
                trial[i].position = trial[i].position + trial[i].velocity * h;
            }
            if (safeDrift(state, trial, springs_)) { accepted = true; break; }
            h *= 0.5;
            if (!(h > 0.0) || remaining - h == remaining ||
                remaining / h > static_cast<double>(config_.maxSubsteps - substeps))
                throw std::runtime_error("Soft-body motion exceeds substep budget");
        }
        if (!accepted) throw std::runtime_error("Soft-body motion retry budget exceeded");
        state.swap(trial);
        const auto a = accelerations(state, springs_, {acceleration_.x, acceleration_.y});
        for (std::size_t i = 0; i < state.size(); ++i)
            if (state[i].inverseMass != 0.0) state[i].velocity = state[i].velocity + a[i] * (h * 0.5);
        for (auto s = springs_.rbegin(); s != springs_.rend(); ++s) dampSpring(state, *s, h * 0.5);
        for (const auto& p : state)
            if (!finite(p.position) || !finite(p.velocity)) throw std::runtime_error("Soft-body state overflow");
        remaining = h == remaining ? 0.0 : remaining - h;
        ++substeps;
    }
    // Validate every conversion before committing any changed particle.
    auto result = particles_;
    for (std::size_t i = 0; i < state.size(); ++i) {
        result[i].position = checkedVector(state[i].position);
        result[i].velocity = checkedVector(state[i].velocity);
    }
    particles_.swap(result);
    lastSubsteps_ = substeps;
}

SoftBodyDiagnostics SoftBody::getDiagnostics() const {
    SoftBodyDiagnostics result;
    result.lastSubsteps = lastSubsteps_;
    for (const auto& p : particles_) {
        result.totalMass += p.mass;
        result.momentumX += p.mass * p.velocity.x;
        result.momentumY += p.mass * p.velocity.y;
        result.kineticEnergy += 0.5 * p.mass * (double(p.velocity.x) * p.velocity.x + double(p.velocity.y) * p.velocity.y);
    }
    for (const auto& s : springs_) {
        const auto& a = particles_[s.first].position;
        const auto& b = particles_[s.second].position;
        const double extension = std::hypot(double(b.x) - a.x, double(b.y) - a.y) - s.restLength;
        result.elasticEnergy += 0.5 * s.stiffness * extension * extension;
        if (s.restLength > 0.0) result.maxStrain = std::max(result.maxStrain, std::abs(extension) / s.restLength);
    }
    if (!std::isfinite(result.totalMass) || !std::isfinite(result.momentumX) ||
        !std::isfinite(result.momentumY) || !std::isfinite(result.kineticEnergy) ||
        !std::isfinite(result.elasticEnergy) || !std::isfinite(result.maxStrain))
        throw std::runtime_error("Soft-body diagnostic overflow");
    return result;
}
} // namespace PhysicsEngine

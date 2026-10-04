#pragma once

#include "physics/math/vector2d.h"
#include <cstddef>
#include <vector>

namespace PhysicsEngine {

struct GravityParticle {
    Vector2d position;
    Vector2d velocity;
    double mass;
};

struct NBodyGravityConfig {
    double gravitationalStrength = 1.0; // G; reduced units by default.
    double softening = 0.0; // Plummer length, same units as position.
    double maxSubstep = 0.01;
    double frequencySafety = 0.1; // h * sqrt(local tidal bound) <= this value.
    std::size_t maxParticles = 1024;
    std::size_t maxSubsteps = 4096;
    std::size_t maxPairWork = 8000000; // Every pair visit, including trial guards.
};

struct GravityDiagnostics {
    double totalMass = 0.0;
    Vector2d centerOfMass;
    Vector2d momentum;
    double angularMomentum = 0.0; // About the coordinate origin, +Z positive.
    double kineticEnergy = 0.0;
    double potentialEnergy = 0.0;
    double totalEnergy = 0.0;
    std::size_t lastSubsteps = 0;
    std::size_t lastPairWork = 0;
};

// Newtonian inverse-square gravity restricted to a plane, with optional Plummer
// softening. Independent of World; no contact, accretion or relativistic physics.
class NBodyGravity {
public:
    explicit NBodyGravity(const NBodyGravityConfig& config = {});
    std::size_t addParticle(Vector2d position, Vector2d velocity = {}, double mass = 1.0);
    void setState(std::size_t index, Vector2d position, Vector2d velocity);
    void applyImpulse(std::size_t index, Vector2d impulse);
    void setConfig(const NBodyGravityConfig& config);
    const NBodyGravityConfig& getConfig() const noexcept { return config_; }
    const std::vector<GravityParticle>& getParticles() const noexcept { return particles_; }
    GravityDiagnostics getDiagnostics() const;
    // Staged velocity Verlet with bounded substeps, trials and total pair work.
    // Rejection leaves particles and successful-step work diagnostics unchanged.
    void step(double dt);

private:
    NBodyGravityConfig config_;
    std::vector<GravityParticle> particles_;
    std::size_t lastSubsteps_ = 0;
    std::size_t lastPairWork_ = 0;
};
} // namespace PhysicsEngine

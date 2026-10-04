#pragma once

#include "physics/math/vector2.h"
#include <cstddef>
#include <set>
#include <utility>
#include <vector>

namespace PhysicsEngine {

struct SoftBodyParticle {
    Vector2 position;
    Vector2 velocity;
    double mass = 1.0;
    bool fixed = false;
};

struct SoftBodySpring {
    std::size_t first;
    std::size_t second;
    double restLength;
    double stiffness;
    double damping;
};

struct SoftBodyConfig {
    double maxSubstep = 0.01;
    // h * estimated maximum angular frequency <= stabilityFactor.
    double stabilityFactor = 0.25;
    std::size_t maxSubsteps = 4096;
    std::size_t maxParticles = 100000;
    std::size_t maxSprings = 300000;
};

struct SoftBodyDiagnostics {
    double totalMass = 0.0; // Includes fixed nodes' finite physical masses.
    double momentumX = 0.0;
    double momentumY = 0.0;
    double kineticEnergy = 0.0;
    double elasticEnergy = 0.0;
    double maxStrain = 0.0; // |length-rest|/rest, for positive-rest springs.
    std::size_t lastSubsteps = 0;
};

// Independent mass-spring simulation. No self, rigid-body or fluid collisions.
// Append-only indexes are stable; state observers cannot bypass validation.
class SoftBody {
public:
    explicit SoftBody(const SoftBodyConfig& config = {});
    std::size_t addParticle(const Vector2& position, const Vector2& velocity = {},
                            double mass = 1.0, bool fixed = false);
    std::size_t addSpring(std::size_t first, std::size_t second, double restLength,
                         double stiffness, double damping = 0.0);
    void setParticleState(std::size_t index, const Vector2& position,
                          const Vector2& velocity = {});
    void setFixed(std::size_t index, bool fixed);
    void applyImpulse(std::size_t index, const Vector2& impulse);
    void setUniformAcceleration(const Vector2& acceleration);
    void setConfig(const SoftBodyConfig& config);
    const SoftBodyConfig& getConfig() const noexcept { return config_; }
    const std::vector<SoftBodyParticle>& getParticles() const noexcept { return particles_; }
    const std::vector<SoftBodySpring>& getSprings() const noexcept { return springs_; }
    const Vector2& getUniformAcceleration() const noexcept { return acceleration_; }
    SoftBodyDiagnostics getDiagnostics() const;
    // Finite dt >= 0. Budget, collapse and overflow failures leave state unchanged.
    void step(double dt);

private:
    SoftBodyConfig config_;
    std::vector<SoftBodyParticle> particles_;
    std::vector<SoftBodySpring> springs_;
    std::set<std::pair<std::size_t, std::size_t>> springPairs_;
    Vector2 acceleration_;
    std::size_t lastSubsteps_ = 0;
};

} // namespace PhysicsEngine

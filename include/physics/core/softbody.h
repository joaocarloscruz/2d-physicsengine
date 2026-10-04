#pragma once

#include "physics/math/vector2.h"
#include <cstddef>
#include <set>
#include <utility>
#include <vector>

namespace PhysicsEngine {

struct SoftBodyForce {
    double x = 0.0;
    double y = 0.0;
};

struct SoftBodyParticle {
    Vector2 position;
    Vector2 velocity;
    double mass = 1.0;
    bool fixed = false;
    SoftBodyForce force; // Pending external force, constant during the next step.
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
    void applyForce(std::size_t index, const Vector2& force);
    void applyForce(std::size_t index, double forceX, double forceY);
    void clearForces() noexcept;
    void clearForces(std::size_t index);
    const SoftBodyForce& getAccumulatedForce(std::size_t index) const { return particles_.at(index).force; }
    void setUniformAcceleration(const Vector2& acceleration);
    void setConfig(const SoftBodyConfig& config);
    const SoftBodyConfig& getConfig() const noexcept { return config_; }
    const std::vector<SoftBodyParticle>& getParticles() const noexcept { return particles_; }
    const std::vector<SoftBodySpring>& getSprings() const noexcept { return springs_; }
    const Vector2& getUniformAcceleration() const noexcept { return acceleration_; }
    SoftBodyDiagnostics getDiagnostics() const;
    // Finite dt >= 0. Failures retain state and forces. Successful positive dt
    // consumes forces; zero dt retains them without integrating.
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

#ifndef FLUID_RIGID_COUPLER_H
#define FLUID_RIGID_COUPLER_H

#include "fluid_particle.h"
#include "../rigidbody.h"

#include <cstddef>
#include <vector>

namespace PhysicsEngine {

struct FluidRigidCouplingSettings {
    float particleRadius = 0.05f;
    float pressureScale = 1.0f;
    float viscosityScale = 1.0f;
    float contactRestitution = 0.0f;

    void Validate() const;
};

struct FluidRigidCouplingStatistics {
    std::size_t interactionCount = 0;
    std::size_t contactCorrectionCount = 0;
    float pressureImpulseMagnitude = 0.0f;
    float viscosityImpulseMagnitude = 0.0f;
    float maximumPenetration = 0.0f;
};

class FluidRigidCoupler {
public:
    explicit FluidRigidCoupler(
        const FluidRigidCouplingSettings& settings = FluidRigidCouplingSettings{}
    );

    FluidRigidCouplingStatistics couple(
        std::vector<FluidParticle>& particles,
        const std::vector<RigidBody*>& bodies,
        float deltaTime
    ) const;

private:
    FluidRigidCouplingSettings settings;
};

} // namespace PhysicsEngine

#endif // FLUID_RIGID_COUPLER_H

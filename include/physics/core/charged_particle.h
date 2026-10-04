#pragma once

#include "physics/math/vector2d.h"

namespace PhysicsEngine {

struct UniformElectromagneticField {
    Vector2d electric; // V/m in the XY plane.
    double magnetic = 0.0; // Tesla, positive along +Z.
};

// Nonrelativistic test charge in prescribed fields. No inter-particle forces,
// radiation, field evolution or automatic World/ParticleSystem registration.
class ChargedParticle {
public:
    ChargedParticle(Vector2d position = {}, Vector2d velocity = {},
                    double mass = 1.0, double charge = 0.0);
    const Vector2d& getPosition() const noexcept { return position_; }
    const Vector2d& getVelocity() const noexcept { return velocity_; }
    double getMass() const noexcept { return mass_; }
    double getCharge() const noexcept { return charge_; }
    double getKineticEnergy() const;
    void setState(Vector2d position, Vector2d velocity);
    // Analytic constant-field evolution; fields remain constant for this call.
    // Finite dt >= 0. Rejection preserves the complete particle state.
    void step(double dt, const UniformElectromagneticField& field = {});

private:
    Vector2d position_, velocity_;
    double mass_, charge_;
};
}

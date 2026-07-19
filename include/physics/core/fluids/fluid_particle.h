#ifndef FLUID_PARTICLE_H
#define FLUID_PARTICLE_H

#include "../../math/vector2.h"

namespace PhysicsEngine {

struct FluidParticleProperties {
    float mass = 1.0f;
    float restDensity = 1000.0f;
    float smoothingLength = 0.5f;
    float viscosity = 0.01f;

    void Validate() const;
};

struct FluidParticle {
    Vector2 position;
    Vector2 velocity;
    Vector2 force;
    float mass;
    float inverseMass;
    float density;
    float pressure;
    float smoothingLength;
    float restDensity;
    float viscosity;
    float volume;

    FluidParticle(
        const Vector2& position = Vector2(),
        const Vector2& velocity = Vector2(),
        const FluidParticleProperties& properties = FluidParticleProperties{}
    );
};

}

#endif // FLUID_PARTICLE_H

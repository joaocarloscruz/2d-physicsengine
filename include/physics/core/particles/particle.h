#ifndef PARTICLE_H
#define PARTICLE_H

#include "../../math/vector2.h"

namespace PhysicsEngine {

struct Particle {
    Vector2 position;
    Vector2 velocity;
    Vector2 force;
    float mass;
    float inverseMass;

    Particle(
        const Vector2& position = Vector2(),
        const Vector2& velocity = Vector2(),
        float mass = 1.0f
    );

    void ApplyForce(const Vector2& appliedForce);
    void Integrate(float deltaTime);
};

}

#endif // PARTICLE_H

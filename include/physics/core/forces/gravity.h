#ifndef GRAVITY_H
#define GRAVITY_H

#include "../force_generator.h"
#include "../../math/vector2.h"

namespace PhysicsEngine {
    class Gravity : public IForceGenerator {
    private:
        Vector2 gravity;

    public:
        Gravity(const Vector2& gravity);

        void applyForce(RigidBody* body) override;

        void setGravity(const Vector2& new_gravity);
    };
}

#endif // GRAVITY_H

#ifndef DRAG_H
#define DRAG_H

#include "../force_generator.h"

namespace PhysicsEngine {
    class Drag : public IForceGenerator {
    private:
        // Drag coefficients
        float k1; // Coefficient for linear drag
        float k2; // Coefficient for quadratic drag

    public:
        Drag(float k1, float k2);

        void applyForce(RigidBody* body) override;
    };
}

#endif // DRAG_H

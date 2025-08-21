#ifndef RIGIDBODY_H
#define RIGIDBODY_H

#include "shape.h"
#include "../math/Vector2.h"

class RigidBody {
public:
    Vector2 position; // world position
    float orientation; // rotation 
    Vector2 velocity; // linear velocity
    float angularVelocity;

    Shape* shape;

    Vector2 force; // Accumulated force
    float torque; // Accumulated torque

    float mass;
    float inverseMass; // 1/mass, used for calculations
    float inertia;
    float inverseInertia; //1/inertia, used for calculations

    RigidBody(Shape* s, float density, const Vector2& pos = {0, 0});

    void ApplyForce(const Vector2& f);
    void ApplyTorque(float t);
    void Integrate(float deltaTime);
    
};

#endif // RIGIDBODY_H
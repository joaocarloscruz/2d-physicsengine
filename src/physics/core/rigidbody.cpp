#include "rigidbody.h"
#include <cmath>

RigidBody::RigidBody(Shape* s, float density, const Vector2& pos) : shape(s), position(pos), orientation(0.0f), velocity(0.0f, 0.0f), angularVelocity(0.0f), force(0.0f, 0.0f), torque(0.0f), mass(0.0f), inverseMass(0.0f), inertia(0.0f), inverseInertia(0.0f) {
    // Initialize mass and inertia based on the shape and density
    if (shape) {
        float area = shape->GetArea();
        mass = density * area;
        inverseMass = (mass != 0.0f) ? 1.0f / mass : 0.0f;

        inertia = shape->GetInertia(mass);
        inverseInertia = (inertia != 0.0f) ? 1.0f / inertia : 0.0f;
    }
}

void RigidBody::ApplyForce(const Vector2& f) {
    force = force + f; 
}

void RigidBody::ApplyTorque(float t) {
    torque += t; 
}

void RigidBody::Integrate(float deltaTime) {
    // Update linear velocity
    Vector2 acceleration = force * inverseMass;
    velocity = velocity + acceleration * deltaTime;

    // Update angular velocity
    float angularAcceleration = torque * inverseInertia;
    angularVelocity += angularAcceleration * deltaTime;

    // Update position and orientation
    position = position + velocity * deltaTime;
    orientation += angularVelocity * deltaTime;

    // Reset forces
    force = Vector2(0.0f, 0.0f);
    torque = 0.0f;
}

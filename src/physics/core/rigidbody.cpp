#include "physics/core/rigidbody.h"
#include <stdexcept>
#include <cmath>

namespace PhysicsEngine {

    RigidBody::RigidBody(Shape* s, float density, const Vector2& pos) : shape(s), position(pos), orientation(0.0f), velocity(0.0f, 0.0f), angularVelocity(0.0f), force(0.0f, 0.0f), torque(0.0f), mass(0.0f), inverseMass(0.0f), inertia(0.0f), inverseInertia(0.0f) {
        // Initialize mass and inertia based on the shape and density
        if (!shape) {
            throw std::invalid_argument("RigidBody requires a valid Shape.");
        }
        float area = shape->GetArea();
        mass = density * area;
        inverseMass = (mass != 0.0f) ? 1.0f / mass : 0.0f;

        inertia = shape->GetInertia(mass);
        inverseInertia = (inertia != 0.0f) ? 1.0f / inertia : 0.0f;

    }

    void RigidBody::ApplyForce(const Vector2& f) {
        force = force + f; 
    }

    void RigidBody::ApplyTorque(float t) {
        torque += t; 
    }

    void RigidBody::Integrate(float deltaTime) {
        //TO-DO : update to Verlet integration
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

    // ---- Setters ----

    void RigidBody::SetVelocity(const Vector2& v) {
        velocity = v;
    }

    void RigidBody::SetPosition(const Vector2& p) {
        position = p;
    }

    void RigidBody::SetMass(float m) {
        mass = m;
        inverseMass = (mass != 0.0f) ? 1.0f / mass : 0.0f;
        inertia = shape->GetInertia(mass);
        inverseInertia = (inertia != 0.0f) ? 1.0f / inertia : 0.0f;
    }

    Vector2 RigidBody::GetAcceleration() const {
        return force * inverseMass;
    }

    // ----- Getters ---

    float RigidBody::GetMass() const {
        return mass;
    }

    float RigidBody::GetInertia() const {
        return inertia;
    }

    float RigidBody::GetInverseMass() const {
        return inverseMass;
    }

    float RigidBody::GetInverseInertia() const {
        return inverseInertia;
    }

    Vector2 RigidBody::GetPosition() const {
        return position;
    }

    float RigidBody::GetOrientation() const {
        return orientation;
    }

    Vector2 RigidBody::GetVelocity() const {
        return velocity;
    }

    float RigidBody::GetAngularVelocity() const {
        return angularVelocity;
    }

    Vector2 RigidBody::GetForce() const {
        return force;
    }

    float RigidBody::GetTorque() const {
        return torque;
    }

}
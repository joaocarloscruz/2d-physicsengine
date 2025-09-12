#include "physics/core/rigidbody.h"
#include "physics/core/shape.h"
#include "physics/math/vector2.h"
#include "physics/math/matrix2x2.h"
#include <stdexcept>
#include <cmath>

namespace PhysicsEngine {

    RigidBody::RigidBody(Shape* s, const Material& mat, const Vector2& pos, bool isStatic) : shape(s), material(mat), velocity(0.0f, 0.0f), angularVelocity(0.0f), force(0.0f, 0.0f), torque(0.0f), mass(0.0f), inverseMass(0.0f), inertia(0.0f), inverseInertia(0.0f), isStatic(isStatic) {
        SetPosition(pos);
        SetOrientation(0.0f);
        // Initialize mass and inertia based on the shape and density
        if (!shape) {
            throw std::invalid_argument("RigidBody requires a valid Shape.");
        }
        if (isStatic) {
            mass = 0.0f;
            inverseMass = 0.0f;
            inertia = 0.0f;
            inverseInertia = 0.0f;
        } else {
            float area = shape->GetArea();
            mass = material.density * area;
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
        if (isStatic) return;

        // --- Velocity Verlet Integration ---

        // 1. Calculate acceleration from forces
        Vector2 linearAcceleration = force * inverseMass;
        float angularAcceleration = torque * inverseInertia;

        // 2. Update position
        // p(t + dt) = p(t) + v(t) * dt + 0.5 * a(t) * dt^2
        position = position + velocity * deltaTime + linearAcceleration * (0.5f * deltaTime * deltaTime);
        orientation = orientation + angularVelocity * deltaTime + angularAcceleration * (0.5f * deltaTime * deltaTime);

        // 3. Calculate new acceleration (if forces were dependent on the new position/orientation)
        // In this simple model, we assume forces are constant over the timestep, so a(t+dt) = a(t)
        Vector2 nextLinearAcceleration = force * inverseMass;
        float nextAngularAcceleration = torque * inverseInertia;

        // 4. Update velocity
        // v(t + dt) = v(t) + 0.5 * (a(t) + a(t+dt)) * dt
        velocity = velocity + (linearAcceleration + nextLinearAcceleration) * (0.5f * deltaTime);
        angularVelocity = angularVelocity + (angularAcceleration + nextAngularAcceleration) * (0.5f * deltaTime);

        // 5. Reset forces and torque for the next frame
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

    void RigidBody::SetOrientation(float o) {
        orientation = o;
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

    AABB RigidBody::GetAABB() const {
        if (shape->type == ShapeType::CIRCLE) {
            Circle* circle = static_cast<Circle*>(shape);
            Vector2 min = position - Vector2(circle->GetRadius(), circle->GetRadius());
            Vector2 max = position + Vector2(circle->GetRadius(), circle->GetRadius());
            return { min, max };
        } else if (shape->type == ShapeType::RECTANGLE) {
            Rectangle* rect = static_cast<Rectangle*>(shape);
            float halfWidth = rect->GetWidth() / 2.0f;
            float halfHeight = rect->GetHeight() / 2.0f;

            Matrix2x2 rot = Matrix2x2::rotation(orientation);
            Vector2 vertices[4];
            vertices[0] = position + rot * Vector2(-halfWidth, -halfHeight);
            vertices[1] = position + rot * Vector2( halfWidth, -halfHeight);
            vertices[2] = position + rot * Vector2( halfWidth,  halfHeight);
            vertices[3] = position + rot * Vector2(-halfWidth,  halfHeight);

            Vector2 min = vertices[0];
            Vector2 max = vertices[0];

            for (int i = 1; i < 4; ++i) {
                if (vertices[i].x < min.x) min.x = vertices[i].x;
                if (vertices[i].y < min.y) min.y = vertices[i].y;
                if (vertices[i].x > max.x) max.x = vertices[i].x;
                if (vertices[i].y > max.y) max.y = vertices[i].y;
            }

            return { min, max };
        }
        return { PhysicsEngine::Vector2(0, 0), PhysicsEngine::Vector2(0, 0) };
    }

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

    bool RigidBody::IsStatic() const {
        return isStatic;
    }

}
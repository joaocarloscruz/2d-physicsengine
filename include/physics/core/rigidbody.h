#ifndef RIGIDBODY_H
#define RIGIDBODY_H

#include "shape.h"
#include "../math/Vector2.h"
#include "../core/collisions/broad_phase/aabb.h"

namespace PhysicsEngine {
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

        RigidBody(Shape* s, float density, const Vector2& pos = {0, 0}, bool isStatic = false);

        void ApplyForce(const Vector2& f);
        void ApplyTorque(float t);
        void Integrate(float deltaTime);

        // getters

        float GetMass() const;
        float GetInertia() const;
        float GetInverseMass() const;
        float GetInverseInertia() const;
        Vector2 GetPosition() const;
        float GetOrientation() const;
        Vector2 GetVelocity() const;
        float GetAngularVelocity() const;
        Vector2 GetForce() const;
        float GetTorque() const;
        Vector2 GetAcceleration() const;
        AABB GetAABB() const;

        bool IsStatic() const;

        // setters

        void SetVelocity(const Vector2& v);
        void SetPosition(const Vector2& p);
        void SetMass(float m);
        

    private:
        bool isStatic;
    };
}

#endif // RIGIDBODY_H
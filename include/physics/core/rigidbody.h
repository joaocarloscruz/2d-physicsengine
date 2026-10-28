#ifndef RIGIDBODY_H
#define RIGIDBODY_H

#include <atomic>
#include <cstdint>

#include "shape.h"
#include "material.h"
#include "simulation_config.h"
#include "../math/vector2.h"
#include "../core/collisions/broad_phase/aabb.h"

namespace PhysicsEngine {
    class RigidBody {
    public:
        Vector2 position; // world position
        float orientation; // rotation 
        Vector2 velocity; // linear velocity
        float angularVelocity;

        Vector2 previousPosition;
        float previousOrientation;

        // Owned immutable copy; source shapes may be destroyed immediately.
        const std::unique_ptr<const Shape> shape;
        Material material;

        Vector2 force; // Accumulated force
        float torque; // Accumulated torque

        float mass;
        float inverseMass; // 1/mass, used for calculations
        float inertia;
        float inverseInertia; //1/inertia, used for calculations

        RigidBody(const Shape* s, const Material& mat, const Vector2& pos = {0, 0}, bool isStatic = false);
        RigidBody(const Shape& s, const Material& mat, const Vector2& pos = {0, 0}, bool isStatic = false)
            : RigidBody(&s, mat, pos, isStatic) {}

        void ApplyForce(const Vector2& f);
        void ApplyTorque(float t);
        void ApplyImpulse(const Vector2& impulse, const Vector2& contactVector);
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
        Vector2 GetVelocityAtPoint(const Vector2& worldPoint) const;
        Vector2 GetForce() const;
        float GetTorque() const;
        Vector2 GetAcceleration() const;
        AABB GetAABB() const;
        std::uint64_t GetId() const;
        std::uint32_t GetCollisionCategoryBits() const;
        std::uint32_t GetCollisionMaskBits() const;
        bool CanCollideWith(const RigidBody& other) const;

        bool IsStatic() const;
        bool IsAwake() const { return !isStatic && awake; }
        void Wake() { if (!isStatic) { awake = true; sleepTime = 0; } }
        bool IsCcdEnabled() const { return ccdEnabled; }
        void SetCcdEnabled(bool enabled) { ccdEnabled = enabled; }
        // A sensor reports overlaps through the collision events (with
        // CollisionEvent::sensor set) but never gets contact impulses, CCD
        // sweeps or island links. Two sensors never report each other.
        // See docs/sensors.md.
        bool IsSensor() const { return sensor; }
        void SetSensor(bool enabled);
        // Opaque application value (an index, a handle or a pointer cast to
        // an integer). The engine never reads it. Defaults to 0.
        std::uintptr_t GetUserData() const { return userData; }
        void SetUserData(std::uintptr_t data) { userData = data; }

        // setters

        void SetVelocity(const Vector2& v);
        void SetAngularVelocity(float w);
        void SetPosition(const Vector2& p);
        void SetMass(float m);
        void SetOrientation(float o);
        void SetCollisionCategoryBits(std::uint32_t bits);
        void SetCollisionMaskBits(std::uint32_t bits);
        

    private:
        friend class World;

        void Integrate(float deltaTime, const SimulationConfig& config);

        // Split form of Integrate for the speculative-contact step, which solves
        // contacts between the velocity and position updates.
        struct StepStart {
            Vector2 velocity; float angularVelocity = 0;
            double ax = 0, ay = 0, alpha = 0;   // applied-load accelerations
            Vector2 predictedVelocity; float predictedAngularVelocity = 0;
        };
        StepStart IntegrateVelocity(float deltaTime, const SimulationConfig& config);
        // Moves along the constant-load trajectory of `start`, plus
        // contactVelocity * deltaTime for the solver's velocity change.
        void IntegratePosition(float deltaTime, const StepStart& start,
                               double contactVx, double contactVy, double contactOmega);

        static std::atomic<std::uint64_t> nextId;
        const std::uint64_t id;
        bool isStatic;
        bool ccdEnabled = false;
        bool sensor = false;
        std::uintptr_t userData = 0;
        bool awake = true;
        float sleepTime = 0;
        bool applyingAutomaticForces = false;
        bool contactWakeRequested = false;
        // Scratch for World::step: this body's index in the stepping world's
        // body list. Only a hint; World verifies it before use.
        std::size_t worldIndex = 0;
        std::uint32_t collisionCategoryBits = 0x00000001u;
        std::uint32_t collisionMaskBits = 0xFFFFFFFFu;
    };
}

#endif // RIGIDBODY_H

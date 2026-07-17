#include "physics/core/rigidbody.h"
#include "physics/core/shape.h"
#include "physics/math/vector2.h"
#include "physics/math/matrix2x2.h"
#include <stdexcept>
#include <cmath>
#include <limits> // Required for std::numeric_limits

namespace PhysicsEngine {

    namespace {
        void ValidateMaterial(const Material& material) {
            if (!std::isfinite(material.density) || material.density <= 0.0f) {
                throw std::invalid_argument("Material density must be positive and finite.");
            }
            if (!std::isfinite(material.restitution)
                || material.restitution < 0.0f
                || material.restitution > 1.0f) {
                throw std::invalid_argument("Material restitution must be finite and between zero and one.");
            }
            if (!std::isfinite(material.staticFriction)
                || !std::isfinite(material.dynamicFriction)
                || material.staticFriction < 0.0f
                || material.dynamicFriction < 0.0f) {
                throw std::invalid_argument("Material friction coefficients must be finite and non-negative.");
            }
        }
    }

    std::atomic<std::uint64_t> RigidBody::nextId{1};

    RigidBody::RigidBody(Shape* s, const Material& mat, const Vector2& pos, bool isStatic)
        : position(pos),
          orientation(0.0f),
          velocity(0.0f, 0.0f),
          angularVelocity(0.0f),
          previousPosition(pos),
          previousOrientation(0.0f),
          shape(s),
          material(mat),
          force(0.0f, 0.0f),
          torque(0.0f),
          mass(0.0f),
          inverseMass(0.0f),
          inertia(0.0f),
          inverseInertia(0.0f),
          id(nextId.fetch_add(1, std::memory_order_relaxed)),
          isStatic(isStatic) {
        // Initialize mass and inertia based on the shape and density
        if (!shape) {
            throw std::invalid_argument("RigidBody requires a valid Shape.");
        }
        ValidateMaterial(material);
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

    void RigidBody::ApplyImpulse(const Vector2& impulse, const Vector2& contactVector) {
        if (isStatic) return;

        velocity = velocity + impulse * inverseMass;
        angularVelocity = angularVelocity + inverseInertia * contactVector.cross(impulse);
    }

    void RigidBody::Integrate(float deltaTime) {
        Integrate(deltaTime, SimulationConfig{});
    }

    void RigidBody::Integrate(float deltaTime, const SimulationConfig& config) {
        if (!std::isfinite(deltaTime) || deltaTime < 0.0f) {
            throw std::invalid_argument("RigidBody delta time must be finite and non-negative.");
        }
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

        // 5. Apply optional safety limits. Continuous collision detection is
        // responsible for tunneling prevention when it is implemented.
        if (config.enableLinearVelocityLimit) {
            const float speedSq = velocity.magnitudeSquared();
            if (speedSq > config.maxLinearSpeed * config.maxLinearSpeed) {
                velocity = velocity.normalized() * config.maxLinearSpeed;
            }
        }
        if (config.enableAngularVelocityLimit) {
            if (angularVelocity > config.maxAngularSpeed) {
                angularVelocity = config.maxAngularSpeed;
            }
            if (angularVelocity < -config.maxAngularSpeed) {
                angularVelocity = -config.maxAngularSpeed;
            }
        }

        // 6. Reset forces and torque for the next frame
        force = Vector2(0.0f, 0.0f);
        torque = 0.0f;
    }
    // ---- Setters ----

    void RigidBody::SetVelocity(const Vector2& v) {
        velocity = v;
    }

    void RigidBody::SetAngularVelocity(float w) {
        angularVelocity = w;
    }

    void RigidBody::SetPosition(const Vector2& p) {
        position = p;
    }

    void RigidBody::SetOrientation(float o) {
        orientation = o;
    }

    void RigidBody::SetMass(float m) {
        if (isStatic) {
            return;
        }
        if (!std::isfinite(m) || m <= 0.0f) {
            throw std::invalid_argument("RigidBody mass must be positive and finite.");
        }
        mass = m;
        inverseMass = (mass != 0.0f) ? 1.0f / mass : 0.0f;
        inertia = shape->GetInertia(mass);
        inverseInertia = (inertia != 0.0f) ? 1.0f / inertia : 0.0f;
    }

    void RigidBody::SetCollisionCategoryBits(std::uint32_t bits) {
        collisionCategoryBits = bits;
    }

    void RigidBody::SetCollisionMaskBits(std::uint32_t bits) {
        collisionMaskBits = bits;
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
        } else if (shape->type == ShapeType::POLYGON) {
            Polygon* poly = static_cast<Polygon*>(shape);

            Matrix2x2 rot = Matrix2x2::rotation(orientation);
            Vector2 min(std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
            Vector2 max(std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest());

            for (const auto& v : poly->getVertices()) {
                Vector2 worldVertex = position + rot * v;
                min.x = std::min(min.x, worldVertex.x);
                min.y = std::min(min.y, worldVertex.y);
                max.x = std::max(max.x, worldVertex.x);
                max.y = std::max(max.y, worldVertex.y);
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

    Vector2 RigidBody::GetVelocityAtPoint(const Vector2& worldPoint) const {
        Vector2 r = worldPoint - position;
        return velocity + Vector2::cross(angularVelocity, r);
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

    std::uint64_t RigidBody::GetId() const {
        return id;
    }

    std::uint32_t RigidBody::GetCollisionCategoryBits() const {
        return collisionCategoryBits;
    }

    std::uint32_t RigidBody::GetCollisionMaskBits() const {
        return collisionMaskBits;
    }

    bool RigidBody::CanCollideWith(const RigidBody& other) const {
        return (collisionCategoryBits & other.collisionMaskBits) != 0u
            && (other.collisionCategoryBits & collisionMaskBits) != 0u;
    }

}

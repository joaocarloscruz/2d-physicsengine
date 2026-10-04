#include "physics/core/rigidbody.h"
#include "physics/core/shape.h"
#include "physics/math/vector2.h"
#include <stdexcept>
#include <cmath>
#include <limits> // Required for std::numeric_limits

namespace PhysicsEngine {

    namespace {
        void ValidateFinite(float value) {
            if (!std::isfinite(value)) throw std::invalid_argument("RigidBody state must be finite.");
        }
        void ValidateFinite(Vector2 value) { ValidateFinite(value.x); ValidateFinite(value.y); }
        float CheckedFloat(double value) {
            if (!std::isfinite(value) || std::abs(value) > std::numeric_limits<float>::max())
                throw std::overflow_error("RigidBody result exceeds finite float range.");
            return static_cast<float>(value);
        }
        Vector2 CheckedVector(double x, double y) { return {CheckedFloat(x), CheckedFloat(y)}; }
        struct BoundsInterval { double lower, upper; };
        double DirectedSum(double a, double b, bool upper) {
            const double sum = a + b;
            // TwoSum retains an offset even when it is smaller than a double
            // ulp at the body's position. Simply doing the addition in double
            // would still collapse a unit shape at x=1e30.
            const double recoveredB = sum - a;
            const double error = (a - (sum - recoveredB)) + (b - recoveredB);
            if ((upper && error > 0) || (!upper && error < 0))
                return std::nextafter(sum, upper ? std::numeric_limits<double>::infinity()
                                                : -std::numeric_limits<double>::infinity());
            return sum;
        }
        BoundsInterval AddBounds(BoundsInterval a, BoundsInterval b) {
            return {DirectedSum(a.lower, b.lower, false), DirectedSum(a.upper, b.upper, true)};
        }
        BoundsInterval ProductBounds(double a, double b) {
            const double product = a * b;
            if (a != 0 && b != 0 && std::abs(product) < std::numeric_limits<double>::min())
                return {std::nextafter(product, -std::numeric_limits<double>::infinity()),
                        std::nextafter(product, std::numeric_limits<double>::infinity())};
            const double error = std::fma(a, b, -product);
            return {error < 0 ? std::nextafter(product, -std::numeric_limits<double>::infinity())
                             : product,
                    error > 0 ? std::nextafter(product, std::numeric_limits<double>::infinity())
                             : product};
        }
        float StoreBound(double value, bool upper) {
            float result = CheckedFloat(value);
            if ((upper && result < value) || (!upper && result > value))
                result = std::nextafter(result, upper ? std::numeric_limits<float>::infinity()
                                                      : -std::numeric_limits<float>::infinity());
            if (!std::isfinite(result))
                throw std::overflow_error("RigidBody bounds exceed finite float range.");
            return result;
        }
        void ValidateInverseProperties(const RigidBody& body) {
            if (!std::isfinite(body.inverseMass) || body.inverseMass <= 0 ||
                !std::isfinite(body.inverseInertia) || body.inverseInertia <= 0)
                throw std::invalid_argument("Dynamic RigidBody inverse mass and inertia must be positive and finite.");
        }
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

    RigidBody::RigidBody(const Shape* s, const Material& mat, const Vector2& pos, bool isStatic)
        : position(pos),
          orientation(0.0f),
          velocity(0.0f, 0.0f),
          angularVelocity(0.0f),
          previousPosition(pos),
          previousOrientation(0.0f),
          shape(s ? s->Clone() : nullptr),
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
        ValidateFinite(position);
        if (isStatic) {
            mass = 0.0f;
            inverseMass = 0.0f;
            inertia = 0.0f;
            inverseInertia = 0.0f;
        } else {
            SetMass(material.density * shape->GetArea());
        }

    }

    void RigidBody::ApplyForce(const Vector2& f) {
        ValidateFinite(f); ValidateFinite(force);
        const Vector2 next = CheckedVector(static_cast<double>(force.x) + f.x,
                                          static_cast<double>(force.y) + f.y);
        if (!applyingAutomaticForces && (f.x != 0 || f.y != 0)) Wake();
        force = next;
    }

    void RigidBody::ApplyTorque(float t) {
        ValidateFinite(t); ValidateFinite(torque);
        const float next = CheckedFloat(static_cast<double>(torque) + t);
        if (!applyingAutomaticForces && t != 0) Wake();
        torque = next;
    }

    void RigidBody::ApplyImpulse(const Vector2& impulse, const Vector2& contactVector) {
        ValidateFinite(impulse); ValidateFinite(contactVector);
        if (isStatic) return;
        ValidateFinite(velocity); ValidateFinite(angularVelocity);
        ValidateInverseProperties(*this);
        const Vector2 nextVelocity = CheckedVector(
            velocity.x + static_cast<double>(impulse.x) * inverseMass,
            velocity.y + static_cast<double>(impulse.y) * inverseMass);
        const double angularImpulse = static_cast<double>(contactVector.x) * impulse.y
            - static_cast<double>(contactVector.y) * impulse.x;
        const float nextAngularVelocity = CheckedFloat(angularVelocity + inverseInertia * angularImpulse);
        if (impulse.x != 0 || impulse.y != 0) Wake();
        velocity = nextVelocity;
        angularVelocity = nextAngularVelocity;
    }

    void RigidBody::Integrate(float deltaTime) {
        Integrate(deltaTime, SimulationConfig{});
    }

    void RigidBody::Integrate(float deltaTime, const SimulationConfig& config) {
        if (!std::isfinite(deltaTime) || deltaTime < 0.0f) {
            throw std::invalid_argument("RigidBody delta time must be finite and non-negative.");
        }
        if (isStatic) return;
        ValidateFinite(position); ValidateFinite(orientation);
        ValidateFinite(velocity); ValidateFinite(angularVelocity);
        ValidateFinite(force); ValidateFinite(torque);
        ValidateInverseProperties(*this);

        // Constant-force integration. Compute in double and publish the complete
        // body update only after every result is representable. World validates
        // its private config; the public overload supplies the default config.
        const double dt = deltaTime;
        const double ax = static_cast<double>(force.x) * inverseMass;
        const double ay = static_cast<double>(force.y) * inverseMass;
        const double alpha = static_cast<double>(torque) * inverseInertia;
        const Vector2 nextPosition = CheckedVector(
            position.x + velocity.x * dt + 0.5 * ax * dt * dt,
            position.y + velocity.y * dt + 0.5 * ay * dt * dt);
        const float nextOrientation = CheckedFloat(orientation + angularVelocity * dt + 0.5 * alpha * dt * dt);
        double vx = velocity.x + ax * dt;
        double vy = velocity.y + ay * dt;
        double omega = angularVelocity + alpha * dt;

        // Cap the speed without overflowing a float squared norm. Position still
        // follows the original constant-force trajectory; caps affect final speed.
        if (config.enableLinearVelocityLimit) {
            const double speed = std::hypot(vx, vy);
            if (speed > config.maxLinearSpeed) {
                const double scale = config.maxLinearSpeed / speed;
                vx *= scale;
                vy *= scale;
            }
        }
        if (config.enableAngularVelocityLimit) {
            omega = std::clamp(omega, -static_cast<double>(config.maxAngularSpeed),
                               static_cast<double>(config.maxAngularSpeed));
        }
        const Vector2 nextVelocity = CheckedVector(vx, vy);
        const float nextAngularVelocity = CheckedFloat(omega);
        position = nextPosition;
        orientation = nextOrientation;
        velocity = nextVelocity;
        angularVelocity = nextAngularVelocity;
        force = Vector2(0.0f, 0.0f);
        torque = 0.0f;
    }
    // ---- Setters ----

    void RigidBody::SetVelocity(const Vector2& v) {
        ValidateFinite(v);
        Wake();
        velocity = v;
    }

    void RigidBody::SetAngularVelocity(float w) {
        ValidateFinite(w);
        Wake();
        angularVelocity = w;
    }

    void RigidBody::SetPosition(const Vector2& p) {
        ValidateFinite(p);
        contactWakeRequested = true;
        Wake();
        position = p;
    }

    void RigidBody::SetOrientation(float o) {
        ValidateFinite(o);
        contactWakeRequested = true;
        Wake();
        orientation = o;
    }

    void RigidBody::SetMass(float m) {
        if (!std::isfinite(m) || m <= 0.0f) {
            throw std::invalid_argument("RigidBody mass must be positive and finite.");
        }
        if (isStatic) return;
        const float nextInverseMass = 1.0f / m;
        const float nextInertia = shape->GetInertia(m);
        const float nextInverseInertia = 1.0f / nextInertia;
        if (!std::isfinite(nextInverseMass) || nextInverseMass <= 0 ||
            !std::isfinite(nextInertia) || nextInertia <= 0 ||
            !std::isfinite(nextInverseInertia) || nextInverseInertia <= 0) {
            throw std::invalid_argument("RigidBody mass and inertia must have finite positive reciprocals.");
        }
        mass = m;
        Wake();
        inverseMass = nextInverseMass;
        inertia = nextInertia;
        inverseInertia = nextInverseInertia;
    }

    void RigidBody::SetCollisionCategoryBits(std::uint32_t bits) {
        Wake(); contactWakeRequested = true;
        collisionCategoryBits = bits;
    }

    void RigidBody::SetCollisionMaskBits(std::uint32_t bits) {
        Wake(); contactWakeRequested = true;
        collisionMaskBits = bits;
    }

    Vector2 RigidBody::GetAcceleration() const {
        return force * inverseMass;
    }

    // ----- Getters ---

    AABB RigidBody::GetAABB() const {
        ValidateFinite(position);
        if (!shape) throw std::invalid_argument("RigidBody bounds require a shape.");
        if (shape->type == ShapeType::CIRCLE) {
            const auto* circle = dynamic_cast<const Circle*>(shape.get());
            const double radius = circle ? circle->GetRadius() : 0;
            if (!std::isfinite(radius) || radius <= 0)
                throw std::invalid_argument("RigidBody bounds require a valid circle.");
            return {{StoreBound(DirectedSum(position.x, -radius, false), false),
                     StoreBound(DirectedSum(position.y, -radius, false), false)},
                    {StoreBound(DirectedSum(position.x, radius, true), true),
                     StoreBound(DirectedSum(position.y, radius, true), true)}};
        } else if (shape->type == ShapeType::POLYGON) {
            const auto* polygon = dynamic_cast<const Polygon*>(shape.get());
            if (!polygon || polygon->getVertices().size() < 3)
                throw std::invalid_argument("RigidBody bounds require a valid polygon.");
            ValidateFinite(orientation);
            const double cosine = std::cos(double(orientation)), sine = std::sin(double(orientation));
            double minX = std::numeric_limits<double>::infinity(), minY = minX;
            double maxX = -minX, maxY = -minY;
            for (const auto& vertex : polygon->getVertices()) {
                ValidateFinite(vertex);
                const auto x = AddBounds({position.x, position.x},
                    AddBounds(ProductBounds(cosine, vertex.x), ProductBounds(-sine, vertex.y)));
                const auto y = AddBounds({position.y, position.y},
                    AddBounds(ProductBounds(sine, vertex.x), ProductBounds(cosine, vertex.y)));
                minX = std::min(minX, x.lower); minY = std::min(minY, y.lower);
                maxX = std::max(maxX, x.upper); maxY = std::max(maxY, y.upper);
            }
            return {{StoreBound(minX, false), StoreBound(minY, false)},
                    {StoreBound(maxX, true), StoreBound(maxY, true)}};
        }
        throw std::invalid_argument("RigidBody bounds require a supported shape.");
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

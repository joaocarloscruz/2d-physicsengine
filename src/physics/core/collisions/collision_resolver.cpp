#include "physics/core/collisions/collision_resolver.h"
#include "physics/core/rigidbody.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace PhysicsEngine {
namespace {
struct DVector {
    double x = 0, y = 0;
    DVector() = default;
    DVector(double x, double y) : x(x), y(y) {}
    DVector(Vector2 v) : x(v.x), y(v.y) {}
    DVector operator+(DVector v) const { return {x + v.x, y + v.y}; }
    DVector operator-(DVector v) const { return {x - v.x, y - v.y}; }
    DVector operator*(double s) const { return {x * s, y * s}; }
    double dot(DVector v) const { return x * v.x + y * v.y; }
    double cross(DVector v) const { return x * v.y - y * v.x; }
};
double Checked(double value) {
    if (!std::isfinite(value))
        throw std::overflow_error("Contact arithmetic exceeds finite double range");
    return value;
}
float CheckedFloat(double value) {
    if (!std::isfinite(value) || std::abs(value) > std::numeric_limits<float>::max())
        throw std::overflow_error("Contact result exceeds finite float state range");
    return static_cast<float>(value);
}
Vector2 CheckedVector(DVector v) {
    return {CheckedFloat(v.x), CheckedFloat(v.y)};
}
void ValidateFinite(double value) {
    if (!std::isfinite(value))
        throw std::invalid_argument("Contact input must be finite");
}
void ValidateFinite(Vector2 v) {
    ValidateFinite(v.x);
    ValidateFinite(v.y);
}
void ValidateImpulse(const ContactImpulse &j) {
    ValidateFinite(j.normal);
    ValidateFinite(j.tangent);
}
void ValidateBody(const RigidBody &body) {
    ValidateFinite(body.position);
    ValidateFinite(body.orientation);
    ValidateFinite(body.velocity);
    ValidateFinite(body.angularVelocity);
    ValidateFinite(body.inverseMass);
    ValidateFinite(body.inverseInertia);
    if (body.IsStatic() ? (body.inverseMass != 0 || body.inverseInertia != 0)
                        : (body.inverseMass <= 0 || body.inverseInertia <= 0))
        throw std::invalid_argument("Contact inverse mass/inertia are invalid for the body type");
    const auto &m = body.material;
    ValidateFinite(m.restitution);
    ValidateFinite(m.staticFriction);
    ValidateFinite(m.dynamicFriction);
    if (m.restitution < 0 || m.restitution > 1 || m.staticFriction < 0 || m.dynamicFriction < 0)
        throw std::invalid_argument("Contact restitution/friction are invalid");
}
Vector2 ContactTangent(Vector2 normal) {
    return {-normal.y, normal.x};
}
std::uint8_t EffectiveContactCount(const CollisionManifold &m) {
    return m.contactCount > 0 ? std::min<std::uint8_t>(m.contactCount, 2)
                              : std::uint8_t(m.hasCollision ? 1 : 0);
}
ContactPoint EffectiveContact(const CollisionManifold &m, std::uint8_t i) {
    return m.contactCount > 0 ? m.contacts[i] : ContactPoint{m.contactPoint, m.penetration, 0};
}
void ValidateManifold(const CollisionManifold &m) {
    ValidateBody(*m.A);
    ValidateBody(*m.B);
    if (m.A == m.B)
        throw std::invalid_argument("Contact requires distinct endpoints");
    ValidateFinite(m.normal);
    ValidateFinite(m.penetration);
    if (m.normal.x == 0 && m.normal.y == 0)
        throw std::invalid_argument("Contact normal must be nonzero");
    // Generated normals are unit vectors subject to float rounding. Preserve
    // that contract without introducing an arbitrary additional norm tolerance.
    for (std::uint8_t i = 0; i < EffectiveContactCount(m); ++i) {
        const auto p = EffectiveContact(m, i);
        ValidateFinite(p.position);
        ValidateFinite(p.penetration);
    }
}
DVector Rotate(DVector v, double angle) {
    const double c = std::cos(angle), s = std::sin(angle);
    return {c * v.x - s * v.y, s * v.x + c * v.y};
}
Vector2 ToLocalPoint(const RigidBody *body, Vector2 point) {
    return CheckedVector(
        Rotate(DVector(point) - DVector(body->position), -double(body->orientation)));
}
DVector ToWorldPoint(const RigidBody *body, Vector2 point) {
    return DVector(body->position) + Rotate(point, body->orientation);
}
DVector ConstraintPointPosition(const ContactConstraint &c, const ContactConstraintPoint &p) {
    return (ToWorldPoint(c.bodyA, p.localAnchorA) + ToWorldPoint(c.bodyB, p.localAnchorB)) * 0.5;
}
DVector PointVelocity(const RigidBody *body, DVector point) {
    const auto r = point - DVector(body->position);
    return DVector(body->velocity) +
           DVector{-double(body->angularVelocity) * r.y, double(body->angularVelocity) * r.x};
}
double EffectiveMass(const RigidBody *a, const RigidBody *b, DVector ra, DVector rb,
                     DVector direction) {
    const double ca = ra.cross(direction), cb = rb.cross(direction);
    const double inverse = Checked(double(a->inverseMass) + b->inverseMass +
                                   ca * ca * a->inverseInertia + cb * cb * b->inverseInertia);
    return inverse > 0 ? Checked(1 / inverse) : 0;
}
void SynchronizeCache(const CollisionManifold &m, ContactImpulseCache &cache) {
    if (cache.contactCount > 2)
        throw std::invalid_argument("Contact cache count exceeds its capacity");
    for (std::uint8_t i = 0; i < cache.contactCount; ++i)
        ValidateImpulse(cache.contacts[i].impulse);
    const auto previous = cache;
    cache = ContactImpulseCache{};
    cache.contactCount = EffectiveContactCount(m);
    for (std::uint8_t i = 0; i < cache.contactCount; ++i) {
        const auto p = EffectiveContact(m, i);
        cache.contacts[i].featureId = p.featureId;
        for (std::uint8_t old = 0; old < previous.contactCount; ++old)
            if (previous.contacts[old].featureId == p.featureId) {
                cache.contacts[i].impulse = previous.contacts[old].impulse;
                break;
            }
    }
}
struct Proposal {
    Vector2 linear;
    float angular;
};
Proposal Propose(const RigidBody *body, DVector lever, DVector impulse, double sign,
                 bool position) {
    const auto linear = position ? body->position : body->velocity;
    const float angular = position ? body->orientation : body->angularVelocity;
    if (body->IsStatic())
        return {linear, angular};
    return {CheckedVector(DVector(linear) + impulse * (sign * body->inverseMass)),
            CheckedFloat(double(angular) + sign * body->inverseInertia * lever.cross(impulse))};
}
void ApplyPair(RigidBody *a, RigidBody *b, DVector ra, DVector rb, DVector impulse, bool position) {
    ValidateBody(*a);
    ValidateBody(*b);
    Checked(impulse.x);
    Checked(impulse.y);
    // Stage all six components before publishing either endpoint. These internal
    // corrections do not issue external wake requests or alter pending loads.
    const auto pa = Propose(a, ra, impulse, -1, position),
               pb = Propose(b, rb, impulse, 1, position);
    if (!a->IsStatic()) {
        if (position) {
            a->position = pa.linear;
            a->orientation = pa.angular;
        } else {
            a->velocity = pa.linear;
            a->angularVelocity = pa.angular;
        }
    }
    if (!b->IsStatic()) {
        if (position) {
            b->position = pb.linear;
            b->orientation = pb.angular;
        } else {
            b->velocity = pb.linear;
            b->angularVelocity = pb.angular;
        }
    }
}
void ApplyVelocityImpulse(RigidBody *a, RigidBody *b, DVector point, DVector impulse) {
    ApplyPair(a, b, point - DVector(a->position), point - DVector(b->position), impulse, false);
}
void PublishImpulse(ContactConstraint &c, ContactConstraintPoint &p, std::uint8_t index,
                    ContactImpulse impulse) {
    p.impulse = impulse;
    if (c.cache)
        c.cache->contacts[index].impulse = impulse;
}
} // namespace

ContactConstraint CollisionResolver::PrepareConstraint(const CollisionManifold &m,
                                                       ContactImpulseCache &cache,
                                                       const SimulationConfig &config) {
    ContactConstraint c;
    if (!m.hasCollision || !m.A || !m.B) {
        cache = ContactImpulseCache{};
        return c;
    }
    ValidateManifold(m);
    config.Validate();
    SynchronizeCache(m, cache);
    c.bodyA = m.A;
    c.bodyB = m.B;
    c.normal = m.normal;
    c.tangent = ContactTangent(m.normal);
    c.pointCount = cache.contactCount;
    c.staticFriction = CheckedFloat(
        std::sqrt(double(m.A->material.staticFriction) * m.B->material.staticFriction));
    c.dynamicFriction = CheckedFloat(
        std::sqrt(double(m.A->material.dynamicFriction) * m.B->material.dynamicFriction));
    c.positionCorrectionFactor = config.positionCorrectionFactor;
    c.penetrationSlop = config.penetrationSlop;
    c.maxPositionCorrection = config.maxPositionCorrection;
    c.velocityTolerance = config.velocityTolerance;
    c.cache = &cache;
    for (std::uint8_t i = 0; i < c.pointCount; ++i) {
        const auto contact = EffectiveContact(m, i);
        auto &p = c.points[i];
        p.localAnchorA = ToLocalPoint(m.A, contact.position);
        p.localAnchorB = ToLocalPoint(m.B, contact.position);
        p.penetration = contact.penetration > 0 ? contact.penetration : m.penetration;
        p.featureId = contact.featureId;
        p.impulse = cache.contacts[i].impulse;
        const auto ra = DVector(contact.position) - DVector(m.A->position),
                   rb = DVector(contact.position) - DVector(m.B->position);
        p.normalMass = EffectiveMass(m.A, m.B, ra, rb, c.normal);
        p.tangentMass = EffectiveMass(m.A, m.B, ra, rb, c.tangent);
        const double normalVelocity =
            Checked((PointVelocity(m.B, contact.position) - PointVelocity(m.A, contact.position))
                        .dot(c.normal));
        if (normalVelocity < -config.restitutionVelocityThreshold)
            p.velocityBias =
                Checked(-double(std::max(m.A->material.restitution, m.B->material.restitution)) *
                        normalVelocity);
    }
    return c;
}
void CollisionResolver::WarmStart(ContactConstraint &c) {
    if (!c.bodyA || !c.bodyB)
        return;
    for (std::uint8_t i = 0; i < c.pointCount; ++i) {
        const auto &p = c.points[i];
        ValidateImpulse(p.impulse);
        ApplyVelocityImpulse(c.bodyA, c.bodyB, ConstraintPointPosition(c, p),
                             DVector(c.normal) * p.impulse.normal +
                                 DVector(c.tangent) * p.impulse.tangent);
    }
}
void CollisionResolver::SolveVelocity(ContactConstraint &c) {
    if (!c.bodyA || !c.bodyB)
        return;
    for (std::uint8_t i = 0; i < c.pointCount; ++i) {
        auto &p = c.points[i];
        const auto worldPoint = ConstraintPointPosition(c, p);
        auto relative = PointVelocity(c.bodyB, worldPoint) - PointVelocity(c.bodyA, worldPoint);
        const double normalVelocity = Checked(relative.dot(c.normal));
        const double normal = std::max(
            Checked(p.impulse.normal + p.normalMass * (-normalVelocity + p.velocityBias)), 0.0);
        ApplyVelocityImpulse(c.bodyA, c.bodyB, worldPoint,
                             DVector(c.normal) * (normal - p.impulse.normal));
        PublishImpulse(c, p, i, {normal, p.impulse.tangent});
        relative = PointVelocity(c.bodyB, worldPoint) - PointVelocity(c.bodyA, worldPoint);
        const double tangentVelocity = Checked(relative.dot(c.tangent));
        if (std::abs(tangentVelocity) > c.velocityTolerance) {
            const double candidate = Checked(p.impulse.tangent - p.tangentMass * tangentVelocity);
            const double maximumStatic = Checked(p.impulse.normal * c.staticFriction);
            double tangent = candidate;
            if (std::abs(candidate) > maximumStatic) {
                const double maximumDynamic = Checked(p.impulse.normal * c.dynamicFriction);
                tangent = std::clamp(candidate, -maximumDynamic, maximumDynamic);
            }
            ApplyVelocityImpulse(c.bodyA, c.bodyB, worldPoint,
                                 DVector(c.tangent) * (tangent - p.impulse.tangent));
            PublishImpulse(c, p, i, {p.impulse.normal, tangent});
        }
    }
}
bool CollisionResolver::SolvePosition(ContactConstraint &c) {
    if (!c.bodyA || !c.bodyB)
        return true;
    double minimumSeparation = 0;
    for (std::uint8_t i = 0; i < c.pointCount; ++i) {
        const auto &p = c.points[i];
        const auto anchorA = ToWorldPoint(c.bodyA, p.localAnchorA),
                   anchorB = ToWorldPoint(c.bodyB, p.localAnchorB);
        const auto ra = anchorA - DVector(c.bodyA->position),
                   rb = anchorB - DVector(c.bodyB->position);
        const double separation = Checked((anchorB - anchorA).dot(c.normal) - p.penetration);
        minimumSeparation = std::min(minimumSeparation, separation);
        const double correction =
            std::clamp(double(c.positionCorrectionFactor) * (separation + c.penetrationSlop),
                       -double(c.maxPositionCorrection), 0.0);
        const double mass = EffectiveMass(c.bodyA, c.bodyB, ra, rb, c.normal);
        ApplyPair(c.bodyA, c.bodyB, ra, rb, DVector(c.normal) * (-mass * correction), true);
    }
    return minimumSeparation >= -3.0 * c.penetrationSlop;
}
void CollisionResolver::WarmStart(const CollisionManifold &m, const ContactImpulse &j) {
    if (!m.hasCollision || !m.A || !m.B)
        return;
    ValidateManifold(m);
    ValidateImpulse(j);
    ApplyVelocityImpulse(m.A, m.B, EffectiveContact(m, 0).position,
                         DVector(m.normal) * j.normal +
                             DVector(ContactTangent(m.normal)) * j.tangent);
}
void CollisionResolver::WarmStart(const CollisionManifold &m, ContactImpulseCache &cache) {
    auto c = PrepareConstraint(m, cache, SimulationConfig{});
    WarmStart(c);
}
void CollisionResolver::Resolve(const CollisionManifold &m) {
    ContactImpulseCache cache;
    Resolve(m, cache, SimulationConfig{});
}
void CollisionResolver::Resolve(const CollisionManifold &m, ContactImpulse &impulse,
                                bool reduceWarmStart) {
    ContactImpulseCache cache;
    cache.contactCount = 1;
    cache.contacts[0] = {EffectiveContact(m, 0).featureId, impulse};
    Resolve(m, cache, SimulationConfig{}, reduceWarmStart);
    impulse = cache.contactCount > 0 ? cache.contacts[0].impulse : ContactImpulse{};
}
void CollisionResolver::Resolve(const CollisionManifold &m, ContactImpulseCache &cache,
                                const SimulationConfig &config, bool reduceWarmStart) {
    (void)reduceWarmStart;
    auto c = PrepareConstraint(m, cache, config);
    SolveVelocity(c);
    SolvePosition(c);
}
} // namespace PhysicsEngine

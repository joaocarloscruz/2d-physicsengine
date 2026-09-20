#include "physics/core/joints.h"
#include "physics/math/matrix2x2.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace PhysicsEngine {
namespace {
void Apply(RigidBody& a, RigidBody& b, Vector2 ra, Vector2 rb, Vector2 impulse, bool position) {
    if (position) {
        a.position = a.position-impulse*a.inverseMass;
        b.position = b.position+impulse*b.inverseMass;
        a.orientation -= a.inverseInertia*ra.cross(impulse);
        b.orientation += b.inverseInertia*rb.cross(impulse);
    } else {
        a.velocity = a.velocity-impulse*a.inverseMass;
        b.velocity = b.velocity+impulse*b.inverseMass;
        a.angularVelocity -= a.inverseInertia*ra.cross(impulse);
        b.angularVelocity += b.inverseInertia*rb.cross(impulse);
    }
}
float InverseEffectiveMass(const RigidBody& a, const RigidBody& b, Vector2 ra, Vector2 rb, Vector2 n) {
    const float ca = ra.cross(n), cb = rb.cross(n);
    return a.inverseMass+b.inverseMass+a.inverseInertia*ca*ca+b.inverseInertia*cb*cb;
}
Vector2 SolvePointMass(const RigidBody& a, const RigidBody& b, Vector2 ra, Vector2 rb, Vector2 rhs) {
    const float m = a.inverseMass+b.inverseMass;
    const float k11 = m+a.inverseInertia*ra.y*ra.y+b.inverseInertia*rb.y*rb.y;
    const float k22 = m+a.inverseInertia*ra.x*ra.x+b.inverseInertia*rb.x*rb.x;
    const float k12 = -a.inverseInertia*ra.x*ra.y-b.inverseInertia*rb.x*rb.y;
    const float determinant = k11*k22-k12*k12;
    if (determinant <= 0) return {};
    return {(k22*rhs.x-k12*rhs.y)/determinant, (k11*rhs.y-k12*rhs.x)/determinant};
}
}

IJoint::IJoint(RigidBodyPtr first, RigidBodyPtr second, Vector2 anchorA, Vector2 anchorB)
    : a(std::move(first)), b(std::move(second)), localA(anchorA), localB(anchorB) {
    if (!a || !b || a == b || (a->IsStatic() && b->IsStatic()))
        throw std::invalid_argument("A joint requires distinct bodies and at least one dynamic body.");
    if (!std::isfinite(localA.x) || !std::isfinite(localA.y) ||
        !std::isfinite(localB.x) || !std::isfinite(localB.y))
        throw std::invalid_argument("Joint anchors must be finite.");
}
Vector2 IJoint::getAnchorA() const { return a->position+Matrix2x2::rotation(a->orientation)*localA; }
Vector2 IJoint::getAnchorB() const { return b->position+Matrix2x2::rotation(b->orientation)*localB; }

DistanceJoint::DistanceJoint(RigidBodyPtr a, RigidBodyPtr b, float length, Vector2 la, Vector2 lb)
    : IJoint(std::move(a), std::move(b), la, lb), length(length) {
    if (!std::isfinite(length) || length <= 0)
        throw std::invalid_argument("Distance joint length must be positive and finite.");
}
void DistanceJoint::solveVelocity() {
    const Vector2 pa = getAnchorA(), pb = getAnchorB();
    const Vector2 ra = pa-a->position, rb = pb-b->position;
    const Vector2 delta = pb-pa;
    const Vector2 n = delta.magnitudeSquared() > 1e-12f ? delta.normalized() : Vector2(1, 0);
    const float k = InverseEffectiveMass(*a, *b, ra, rb, n);
    if (k > 0) Apply(*a, *b, ra, rb, n*(-(b->GetVelocityAtPoint(pb)-a->GetVelocityAtPoint(pa)).dot(n)/k), false);
}
bool DistanceJoint::solvePosition(float tolerance, float maxCorrection) {
    const Vector2 pa = getAnchorA(), pb = getAnchorB();
    const Vector2 ra = pa-a->position, rb = pb-b->position;
    const Vector2 delta = pb-pa;
    const float distance = delta.magnitude(), error = distance-length;
    const Vector2 n = distance > 1e-6f ? delta/distance : Vector2(1, 0);
    const float k = InverseEffectiveMass(*a, *b, ra, rb, n);
    if (k > 0) Apply(*a, *b, ra, rb, n*(-std::clamp(error, -maxCorrection, maxCorrection)/k), true);
    return std::abs(error) <= tolerance;
}

RevoluteJoint::RevoluteJoint(RigidBodyPtr a, RigidBodyPtr b, Vector2 la, Vector2 lb)
    : IJoint(std::move(a), std::move(b), la, lb) {}
void RevoluteJoint::solveVelocity() {
    const Vector2 pa = getAnchorA(), pb = getAnchorB();
    const Vector2 ra = pa-a->position, rb = pb-b->position;
    const Vector2 velocity = b->GetVelocityAtPoint(pb)-a->GetVelocityAtPoint(pa);
    Apply(*a, *b, ra, rb, SolvePointMass(*a, *b, ra, rb, velocity*-1), false);
}
bool RevoluteJoint::solvePosition(float tolerance, float maxCorrection) {
    const Vector2 pa = getAnchorA(), pb = getAnchorB();
    const Vector2 ra = pa-a->position, rb = pb-b->position;
    Vector2 error = pb-pa;
    const float distance = error.magnitude();
    if (distance > maxCorrection) error = error*(maxCorrection/distance);
    Apply(*a, *b, ra, rb, SolvePointMass(*a, *b, ra, rb, error*-1), true);
    return distance <= tolerance;
}
}

#include "physics/core/joints.h"
#include "physics/math/matrix2x2.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace PhysicsEngine {
namespace {
constexpr double Pi = 3.14159265358979323846;
constexpr double AngularTolerance = 0.005;
constexpr double MaxAngularCorrection = 0.2;
void ApplyAngularImpulse(RigidBody& a, RigidBody& b, double impulse) {
    a.angularVelocity = static_cast<float>(a.angularVelocity - a.inverseInertia * impulse);
    b.angularVelocity = static_cast<float>(b.angularVelocity + b.inverseInertia * impulse);
}
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
struct HingeImpulse { Vector2 linear; double angular; };
HingeImpulse SolveHingeMass(const RigidBody& a, const RigidBody& b,
    Vector2 ra, Vector2 rb, Vector2 rhs, double angularRhs) {
    const double ia = a.inverseInertia, ib = b.inverseInertia;
    const double k33 = ia + ib;
    if (k33 <= 0) return {SolvePointMass(a, b, ra, rb, rhs), 0};
    const double m = static_cast<double>(a.inverseMass) + b.inverseMass;
    const double k13 = -ia * ra.y - ib * rb.y, k23 = ia * ra.x + ib * rb.x;
    // Schur complement of the angular row. This avoids slow alternating
    // corrections when the lever arm is much longer than the body's radius.
    const double weight = ia * ib / k33;
    const double dx = static_cast<double>(ra.x) - rb.x, dy = static_cast<double>(ra.y) - rb.y;
    const double k11 = m + weight * dy * dy;
    const double k22 = m + weight * dx * dx;
    const double k12 = -weight * dx * dy;
    const double x = rhs.x - k13 * angularRhs / k33, y = rhs.y - k23 * angularRhs / k33;
    const double determinant = m * (m + weight * (dx * dx + dy * dy));
    if (determinant <= 0) return {};
    const double px = (k22 * x - k12 * y) / determinant;
    const double py = (k11 * y - k12 * x) / determinant;
    return {{static_cast<float>(px), static_cast<float>(py)}, (angularRhs - k13 * px - k23 * py) / k33};
}
void SolveAngularStop(RigidBody& a, RigidBody& b, Vector2 pa, Vector2 pb,
    double targetSpeed, double& accumulated, bool lower) {
    const Vector2 ra = pa - a.position, rb = pb - b.position;
    const Vector2 rhs = (b.GetVelocityAtPoint(pb) - a.GetVelocityAtPoint(pa)) * -1;
    const double speed = static_cast<double>(b.angularVelocity) - a.angularVelocity;
    const auto candidate = SolveHingeMass(a, b, ra, rb, rhs, targetSpeed - speed);
    const double next = lower ? std::max(accumulated + candidate.angular, 0.0)
        : std::min(accumulated + candidate.angular, 0.0);
    const double angular = next - accumulated;
    accumulated = next;
    const double k13 = -static_cast<double>(a.inverseInertia) * ra.y - static_cast<double>(b.inverseInertia) * rb.y;
    const double k23 = static_cast<double>(a.inverseInertia) * ra.x + static_cast<double>(b.inverseInertia) * rb.x;
    const Vector2 linear = angular == candidate.angular ? candidate.linear :
        SolvePointMass(a, b, ra, rb, {static_cast<float>(rhs.x - k13 * angular), static_cast<float>(rhs.y - k23 * angular)});
    Apply(a, b, ra, rb, linear, false);
    ApplyAngularImpulse(a, b, angular);
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
    : IJoint(std::move(a), std::move(b), la, lb) {
    referenceAngle = std::remainder(static_cast<double>(this->b->orientation) - this->a->orientation, 2 * Pi);
}
void RevoluteJoint::setLimits(bool enabled, float lowerAngle, float upperAngle) {
    if (!std::isfinite(lowerAngle) || !std::isfinite(upperAngle) ||
        lowerAngle > upperAngle || lowerAngle <= -Pi || upperAngle >= Pi)
        throw std::invalid_argument("Joint limits must be ordered finite angles strictly between -pi and pi.");
    if (limitsEnabled == enabled && lowerLimit == lowerAngle && upperLimit == upperAngle) return;
    limitsEnabled = enabled;
    lowerLimit = lowerAngle;
    upperLimit = upperAngle;
    a->Wake(); b->Wake();
}
double RevoluteJoint::getAngle() const {
    return std::remainder(static_cast<double>(b->orientation) - a->orientation - referenceAngle, 2 * Pi);
}
void RevoluteJoint::setMotor(bool enabled, float speed, float maxTorque) {
    if (!std::isfinite(speed) || !std::isfinite(maxTorque) || maxTorque < 0)
        throw std::invalid_argument("Motor speed must be finite and maximum torque finite and non-negative.");
    if (motorEnabled == enabled && motorSpeed == speed && maxMotorTorque == maxTorque) return;
    motorEnabled = enabled;
    motorSpeed = speed;
    maxMotorTorque = maxTorque;
    a->Wake(); b->Wake();
}
double RevoluteJoint::getMotorTorque() const {
    return stepDuration > 0 ? motorImpulse / stepDuration : 0;
}
void RevoluteJoint::prepareStep(float deltaTime) {
    stepDuration = deltaTime;
    motorImpulse = 0;
    lowerImpulse = 0;
    upperImpulse = 0;
}
bool RevoluteJoint::preventsSleeping() const {
    return motorEnabled && maxMotorTorque > 0 && motorSpeed != 0;
}
void RevoluteJoint::solveVelocity() {
    const double angularMass = static_cast<double>(a->inverseInertia) + b->inverseInertia;
    if (motorEnabled && stepDuration > 0 && angularMass > 0) {
        const double speed = static_cast<double>(b->angularVelocity) - a->angularVelocity;
        const double cap = static_cast<double>(maxMotorTorque) * stepDuration;
        const double nextImpulse = std::clamp(motorImpulse + (motorSpeed - speed) / angularMass, -cap, cap);
        const double impulse = nextImpulse - motorImpulse;
        motorImpulse = nextImpulse;
        ApplyAngularImpulse(*a, *b, impulse);
    }
    if (limitsEnabled && angularMass > 0) {
        const Vector2 pa = getAnchorA(), pb = getAnchorB();
        if (lowerLimit == upperLimit) {
            const Vector2 ra = pa - a->position, rb = pb - b->position;
            const auto impulse = SolveHingeMass(*a, *b, ra, rb,
                (b->GetVelocityAtPoint(pb) - a->GetVelocityAtPoint(pa)) * -1,
                -(static_cast<double>(b->angularVelocity) - a->angularVelocity));
            Apply(*a, *b, ra, rb, impulse.linear, false);
            ApplyAngularImpulse(*a, *b, impulse.angular);
        } else {
            const double angle = getAngle();
            const double lowerGap = angle - lowerLimit;
            if (stepDuration > 0 || lowerGap <= 0) {
                const double bias = stepDuration > 0 ? std::max(lowerGap, 0.0) / stepDuration : 0;
                SolveAngularStop(*a, *b, pa, pb, -bias, lowerImpulse, true);
            }
            const double upperGap = upperLimit - angle;
            if (stepDuration > 0 || upperGap <= 0) {
                const double bias = stepDuration > 0 ? std::max(upperGap, 0.0) / stepDuration : 0;
                SolveAngularStop(*a, *b, pa, pb, bias, upperImpulse, false);
            }
        }
        // The block solve already enforces the point constraint. With zero dt
        // and an inactive interval neither stop is solved, so solve the point below.
        if (stepDuration > 0 || lowerLimit == upperLimit || getAngle() <= lowerLimit || getAngle() >= upperLimit) return;
    }
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
    double angularError = 0;
    if (limitsEnabled) {
        const double angle = getAngle();
        angularError = angle - std::clamp(angle, static_cast<double>(lowerLimit), static_cast<double>(upperLimit));
        if (angularError != 0 || lowerLimit == upperLimit) {
            const auto impulse = SolveHingeMass(*a, *b, ra, rb, error * -1,
                -std::clamp(angularError, -MaxAngularCorrection, MaxAngularCorrection));
            Apply(*a, *b, ra, rb, impulse.linear, true);
            a->orientation = static_cast<float>(a->orientation - a->inverseInertia * impulse.angular);
            b->orientation = static_cast<float>(b->orientation + b->inverseInertia * impulse.angular);
            return distance <= tolerance && std::abs(angularError) <= AngularTolerance;
        }
    }
    Apply(*a, *b, ra, rb, SolvePointMass(*a, *b, ra, rb, error*-1), true);
    return distance <= tolerance && std::abs(angularError) <= AngularTolerance;
}
}

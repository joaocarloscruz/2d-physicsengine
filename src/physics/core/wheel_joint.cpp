#include "physics/core/wheel_joint.h"
#include "joint_solver_math.h"
#include <algorithm>
#include <utility>

namespace PhysicsEngine {
using namespace Detail::JointMath;
namespace {
// Rows follow Box2D v3's wheel joint (wheel_joint.c): the axis and its normal
// rotate with A, so A's lever for a row along u is (d + ra) x u.
struct WheelGeometry {
    Frame f;
    D2 axis, normal;
    double axialA, axialB, normalA, normalB; // levers: (d + ra) x u, rb x u
    double axialMass, normalMass;            // inverse effective masses
    WheelGeometry(const RigidBody &a, const RigidBody &b, Vector2 la, Vector2 lb, D2 localAxis)
        : f(a, b, la, lb) {
        axis = Rotate(localAxis, a.orientation);
        normal = axis.perp();
        const D2 leverA = f.delta + f.ra;
        axialA = leverA.cross(axis);
        axialB = f.rb.cross(axis);
        normalA = leverA.cross(normal);
        normalB = f.rb.cross(normal);
        const double m = double(a.inverseMass) + b.inverseMass;
        axialMass = Checked(m + a.inverseInertia * axialA * axialA + b.inverseInertia * axialB * axialB);
        normalMass = Checked(m + a.inverseInertia * normalA * normalA + b.inverseInertia * normalB * normalB);
    }
    double translation() const { return axis.dot(f.delta); }
    double speed(const RigidBody &a, const RigidBody &b, bool axial) const {
        const D2 dv = Double(b.velocity) - Double(a.velocity);
        return axial ? axis.dot(dv) + axialB * b.angularVelocity - axialA * a.angularVelocity
                     : normal.dot(dv) + normalB * b.angularVelocity - normalA * a.angularVelocity;
    }
    // Impulse lambda along the axis (or normal) on B, -lambda on A.
    void apply(RigidBody &a, RigidBody &b, double lambda, bool axial, bool position) const {
        const D2 p = (axial ? axis : normal) * lambda;
        ApplyGeneral(a, b, p, lambda * (axial ? axialA : normalA), lambda * (axial ? axialB : normalB),
                     position);
    }
};
}
WheelJoint::WheelJoint(RigidBodyPtr a, RigidBodyPtr b, Vector2 axis, Vector2 la, Vector2 lb)
    : IJoint(std::move(a), std::move(b), la, lb) {
    const double norm = std::hypot(double(axis.x), double(axis.y));
    if (!std::isfinite(norm) || norm == 0)
        throw std::invalid_argument("Wheel axis must be finite and nonzero.");
    axisX = axis.x / norm;
    axisY = axis.y / norm;
}
Vector2 WheelJoint::getLocalAxis() const { return {float(axisX), float(axisY)}; }
Vector2 WheelJoint::getAxis() const {
    ValidateBody(*a);
    const D2 e = Rotate(D2{axisX, axisY}, a->orientation);
    return {float(e.x), float(e.y)};
}
double WheelJoint::getTranslation() const {
    return WheelGeometry(*a, *b, localA, localB, {axisX, axisY}).translation();
}
double WheelJoint::getTranslationSpeed() const {
    return WheelGeometry(*a, *b, localA, localB, {axisX, axisY}).speed(*a, *b, true);
}
double WheelJoint::getTransverseError() const {
    const WheelGeometry g(*a, *b, localA, localB, {axisX, axisY});
    return g.normal.dot(g.f.delta);
}
double WheelJoint::getAngularSpeed() const {
    ValidateBody(*a);
    ValidateBody(*b);
    return double(b->angularVelocity) - a->angularVelocity;
}
void WheelJoint::setSpring(float frequency, float dampingRatio) {
    ValidateSpring(frequency, dampingRatio, true,
        "Wheel spring frequency and damping ratio must be finite and non-negative.");
    if (frequency == springFrequency && dampingRatio == springDamping) return;
    springFrequency = frequency;
    springDamping = dampingRatio;
    a->Wake();
    b->Wake();
}
void WheelJoint::setMotor(bool enabled, float speed, float maxTorque) {
    if (!std::isfinite(speed) || !std::isfinite(maxTorque) || maxTorque < 0)
        throw std::invalid_argument(
            "Motor speed must be finite and maximum torque finite and non-negative.");
    if (motorEnabled == enabled && motorSpeed == speed && maxMotorTorque == maxTorque) return;
    motorEnabled = enabled;
    motorSpeed = speed;
    maxMotorTorque = maxTorque;
    a->Wake();
    b->Wake();
}
void WheelJoint::setLimits(bool enabled, float lower, float upper) {
    if (!std::isfinite(lower) || !std::isfinite(upper) || lower > upper)
        throw std::invalid_argument("Wheel limits must be finite and ordered.");
    if (limitsEnabled == enabled && lowerLimit == lower && upperLimit == upper) return;
    limitsEnabled = enabled;
    lowerLimit = lower;
    upperLimit = upper;
    a->Wake();
    b->Wake();
}
double WheelJoint::getMotorTorque() const { return stepDuration > 0 ? motorImpulse / stepDuration : 0; }
double WheelJoint::getSpringForce() const { return stepDuration > 0 ? springImpulse / stepDuration : 0; }
void WheelJoint::prepareStep(float deltaTime) {
    IJoint::prepareStep(deltaTime);
    stepDuration = deltaTime;
    // Accumulators persist for warm starting; drop the rows that are off.
    if (deltaTime <= 0 || !motorEnabled) motorImpulse = 0;
    if (deltaTime <= 0 || springFrequency == 0) springImpulse = 0;
    if (deltaTime <= 0 || !limitsEnabled) lowerImpulse = upperImpulse = 0;
    if (deltaTime <= 0) transverseImpulse = 0;
}
bool WheelJoint::preventsSleeping() const {
    return motorEnabled && maxMotorTorque > 0 && motorSpeed != 0;
}
void WheelJoint::warmStart(float factor) {
    springImpulse *= factor;
    lowerImpulse *= factor;
    upperImpulse *= factor;
    transverseImpulse *= factor;
    motorImpulse *= factor;
    const double axial = springImpulse + lowerImpulse - upperImpulse;
    if (axial == 0 && transverseImpulse == 0 && motorImpulse == 0) return;
    const WheelGeometry g(*a, *b, localA, localB, {axisX, axisY});
    const D2 p = g.axis * axial + g.normal * transverseImpulse;
    ApplyGeneral(*a, *b, p, axial * g.axialA + transverseImpulse * g.normalA + motorImpulse,
                 axial * g.axialB + transverseImpulse * g.normalB + motorImpulse, false);
}
void WheelJoint::solveVelocity() {
    const double h = stepDuration;
    const double angularMass = double(a->inverseInertia) + b->inverseInertia;
    // Motor: angular row only, capped by maxTorque * h.
    if (motorEnabled && h > 0 && angularMass > 0) {
        ValidateBody(*a);
        ValidateBody(*b);
        const double cap = double(maxMotorTorque) * h;
        const double dw = double(b->angularVelocity) - a->angularVelocity - motorSpeed;
        const double next = std::clamp(Checked(motorImpulse - dw / angularMass), -cap, cap);
        const double lambda = next - motorImpulse;
        ApplyGeneral(*a, *b, {}, lambda, lambda, false);
        motorImpulse = next;
    }
    // Spring along the axis toward translation 0.
    if (springFrequency > 0 && h > 0) {
        const WheelGeometry g(*a, *b, localA, localB, {axisX, axisY});
        const Softness s = MakeSoft(springFrequency, springDamping, h);
        const double cdot = g.speed(*a, *b, true);
        const double lambda = Checked(-s.massScale * (cdot + s.biasRate * g.translation()) / g.axialMass -
                                      s.impulseScale * springImpulse);
        g.apply(*a, *b, lambda, true, false);
        springImpulse = Checked(springImpulse + lambda);
    }
    // Translation stops: speculative toward an open stop, rigid at contact.
    if (limitsEnabled) {
        const double infinity = std::numeric_limits<double>::infinity();
        const auto stop = [&](double gap, double sign, double &total) {
            // gap >= 0 is free travel left before the stop; sign is +1 for the
            // lower stop (pushes B along +axis) and -1 for the upper one.
            if (h <= 0 && gap > 0) return;
            const WheelGeometry g(*a, *b, localA, localB, {axisX, axisY});
            const double bias = h > 0 ? std::max(gap, 0.0) / h : 0;
            const double cdot = sign * g.speed(*a, *b, true);
            const double next = std::clamp(Checked(total - (cdot + bias) / g.axialMass), 0.0, infinity);
            const double lambda = next - total;
            g.apply(*a, *b, sign * lambda, true, false);
            total = next;
        };
        const double x = getTranslation();
        if (lowerLimit == upperLimit) {
            // Locked: one two-sided row; keep the net in lowerImpulse.
            const WheelGeometry g(*a, *b, localA, localB, {axisX, axisY});
            const double lambda = Checked(-g.speed(*a, *b, true) / g.axialMass);
            g.apply(*a, *b, lambda, true, false);
            lowerImpulse = Checked(lowerImpulse + lambda);
        } else {
            stop(x - lowerLimit, 1, lowerImpulse);
            stop(double(upperLimit) - x, -1, upperImpulse);
        }
    }
    // Point-to-line: rigid in velocity, projected in the position pass.
    const WheelGeometry g(*a, *b, localA, localB, {axisX, axisY});
    const double lambda = Checked(-g.speed(*a, *b, false) / g.normalMass);
    g.apply(*a, *b, lambda, false, false);
    transverseImpulse = Checked(transverseImpulse + lambda);
}
bool WheelJoint::solvePosition(float tolerance, float maxCorrection) {
    ValidateCorrection(tolerance, maxCorrection);
    const double limit = maxCorrection;
    const WheelGeometry g(*a, *b, localA, localB, {axisX, axisY});
    const double error = g.normal.dot(g.f.delta);
    g.apply(*a, *b, -std::clamp(error, -limit, limit) / g.normalMass, false, true);
    double travel = 0;
    if (limitsEnabled) {
        const WheelGeometry after(*a, *b, localA, localB, {axisX, axisY});
        const double x = after.translation();
        travel = x - std::clamp(x, double(lowerLimit), double(upperLimit));
        if (travel != 0)
            after.apply(*a, *b, -std::clamp(travel, -limit, limit) / after.axialMass, true, true);
    }
    return std::abs(error) <= tolerance && std::abs(travel) <= tolerance;
}
}

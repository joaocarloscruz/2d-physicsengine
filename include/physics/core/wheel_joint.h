#pragma once
#include "joints.h"

namespace PhysicsEngine {
// B's anchor slides along an axis fixed in A (the suspension axis) and B
// rotates freely. A spring-damper along the axis pulls the translation toward
// zero (the anchors coincide), an optional motor drives B's rotation relative
// to A, and optional limits bound the translation. A is the chassis, B the
// wheel. Translation is the signed projection of B's anchor minus A's anchor
// on the axis, as for PrismaticJoint.
class WheelJoint final : public IJoint {
public:
    WheelJoint(RigidBodyPtr a, RigidBodyPtr b, Vector2 localAxisA = {0, 1},
        Vector2 localAnchorA = {}, Vector2 localAnchorB = {});
    Vector2 getAxis() const;
    Vector2 getLocalAxis() const;
    double getTranslation() const;
    double getTranslationSpeed() const;
    // Signed distance of B's anchor from the axis line; the joint holds it at 0.
    double getTransverseError() const;
    // B's angular velocity minus A's, in radians/second.
    double getAngularSpeed() const;
    // frequency >= 0 Hz, damping ratio >= 0. Frequency 0 (the default) turns
    // the spring off and leaves the axis free. The frequency applies to the
    // axial effective mass; with a static or much heavier A and the wheel
    // anchor at its center that is the wheel's mass.
    void setSpring(float frequency, float dampingRatio);
    float getSpringFrequency() const { return springFrequency; }
    float getSpringDampingRatio() const { return springDamping; }
    double getSpringForce() const;
    // Speed is B relative to A in radians/second; torque is non-negative.
    void setMotor(bool enabled, float speed, float maxTorque);
    bool isMotorEnabled() const { return motorEnabled; }
    float getMotorSpeed() const { return motorSpeed; }
    float getMaxMotorTorque() const { return maxMotorTorque; }
    double getMotorTorque() const;
    void setLimits(bool enabled, float lowerTranslation, float upperTranslation);
    bool areLimitsEnabled() const { return limitsEnabled; }
    float getLowerLimit() const { return lowerLimit; }
    float getUpperLimit() const { return upperLimit; }
protected:
    void prepareStep(float deltaTime) override;
    bool preventsSleeping() const override;
    void warmStart(float factor) override;
    void solveVelocity() override;
    bool solvePosition(float tolerance, float maxCorrection) override;
private:
    double axisX, axisY;
    float springFrequency = 0, springDamping = 0;
    bool motorEnabled = false, limitsEnabled = false;
    float motorSpeed = 0, maxMotorTorque = 0, stepDuration = 0;
    float lowerLimit = 0, upperLimit = 0;
    // Axial rows (spring, lower, upper) act along A's axis, the transverse row
    // along its normal; the motor is a pure angular impulse on B.
    double springImpulse = 0, lowerImpulse = 0, upperImpulse = 0;
    double transverseImpulse = 0, motorImpulse = 0;
};
}

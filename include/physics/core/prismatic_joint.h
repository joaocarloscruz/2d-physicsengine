#pragma once
#include "joints.h"

namespace PhysicsEngine {
// Axis belongs to A. Translation is the signed projection of B's anchor minus
// A's anchor, in world units; it is not offset by the construction translation.
class PrismaticJoint final : public IJoint {
public:
    PrismaticJoint(RigidBodyPtr a, RigidBodyPtr b, Vector2 localAxisA = {1, 0},
        Vector2 localAnchorA = {}, Vector2 localAnchorB = {});
    Vector2 getAxis() const;
    Vector2 getLocalAxis() const;
    double getTranslation() const;
    double getTranslationSpeed() const;
    double getTransverseError() const;
    double getAngle() const;
    double getReferenceAngle() const { return referenceAngle; }
    // Speed is B relative to A in world units/second; force is nonnegative.
    void setMotor(bool enabled, float speed, float maxForce);
    bool isMotorEnabled() const { return motorEnabled; }
    float getMotorSpeed() const { return motorSpeed; }
    float getMaxMotorForce() const { return maxMotorForce; }
    double getMotorForce() const;
    void setLimits(bool enabled, float lowerTranslation, float upperTranslation);
    bool areLimitsEnabled() const { return limitsEnabled; }
    float getLowerLimit() const { return lowerLimit; }
    float getUpperLimit() const { return upperLimit; }
protected:
    void prepareStep(float deltaTime) override;
    bool preventsSleeping() const override;
    void solveVelocity() override;
    bool solvePosition(float tolerance, float maxCorrection) override;
private:
    double axisX, axisY;
    double referenceAngle;
    bool motorEnabled=false, limitsEnabled=false;
    float motorSpeed=0, maxMotorForce=0, stepDuration=0;
    float lowerLimit=0, upperLimit=0;
    double motorImpulse=0, lowerImpulse=0, upperImpulse=0;
};
}

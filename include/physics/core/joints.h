#pragma once
#include "rigidbody.h"
#include "types.h"

namespace PhysicsEngine {
class IJoint {
public:
    virtual ~IJoint() = default;
    const RigidBodyPtr& getBodyA() const { return a; }
    const RigidBodyPtr& getBodyB() const { return b; }
    Vector2 getAnchorA() const;
    Vector2 getAnchorB() const;
protected:
    friend class World;
    IJoint(RigidBodyPtr a, RigidBodyPtr b, Vector2 localAnchorA, Vector2 localAnchorB);
    virtual void prepareStep(float deltaTime) {}
    virtual bool preventsSleeping() const { return false; }
    virtual void solveVelocity() = 0;
    virtual bool solvePosition(float tolerance, float maxCorrection) = 0;
    RigidBodyPtr a, b;
    Vector2 localA, localB;
};
using JointPtr = std::shared_ptr<IJoint>;

class DistanceJoint final : public IJoint {
public:
    DistanceJoint(RigidBodyPtr a, RigidBodyPtr b, float length,
        Vector2 localAnchorA = {}, Vector2 localAnchorB = {});
    float getLength() const { return length; }
protected:
    void solveVelocity() override;
    bool solvePosition(float tolerance, float maxCorrection) override;
private:
    float length;
};

class RevoluteJoint final : public IJoint {
public:
    RevoluteJoint(RigidBodyPtr a, RigidBodyPtr b,
        Vector2 localAnchorA = {}, Vector2 localAnchorB = {});
    // Speed is body B relative to A in radians/second; torque is non-negative.
    void setMotor(bool enabled, float speed, float maxTorque);
    bool isMotorEnabled() const { return motorEnabled; }
    float getMotorSpeed() const { return motorSpeed; }
    float getMaxMotorTorque() const { return maxMotorTorque; }
    double getMotorTorque() const;
    // Limits use the principal relative angle from the construction pose.
    void setLimits(bool enabled, float lowerAngle, float upperAngle);
    bool areLimitsEnabled() const { return limitsEnabled; }
    float getLowerLimit() const { return lowerLimit; }
    float getUpperLimit() const { return upperLimit; }
    double getAngle() const;
protected:
    void prepareStep(float deltaTime) override;
    bool preventsSleeping() const override;
    void solveVelocity() override;
    bool solvePosition(float tolerance, float maxCorrection) override;
private:
    bool motorEnabled = false;
    float motorSpeed = 0;
    float maxMotorTorque = 0;
    float stepDuration = 0;
    double motorImpulse = 0;
    double referenceAngle = 0;
    bool limitsEnabled = false;
    float lowerLimit = 0;
    float upperLimit = 0;
    double lowerImpulse = 0;
    double upperImpulse = 0;
};
}

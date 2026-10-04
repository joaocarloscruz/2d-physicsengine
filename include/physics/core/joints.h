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
protected:
    void solveVelocity() override;
    bool solvePosition(float tolerance, float maxCorrection) override;
};
}

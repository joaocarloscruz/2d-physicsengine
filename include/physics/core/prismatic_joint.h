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
protected:
    void solveVelocity() override;
    bool solvePosition(float tolerance, float maxCorrection) override;
private:
    double axisX, axisY;
    double referenceAngle;
};
}

#pragma once
#include "joints.h"

namespace PhysicsEngine {
// Locks B's anchor to A's anchor and the relative angle to its value at
// construction. Each part can be made soft with a spring frequency (Hz) and
// damping ratio; frequency 0 (the default) is rigid. A rigid weld is solved
// as one 3x3 point-plus-angle block in both the velocity and position passes.
class WeldJoint final : public IJoint {
public:
    WeldJoint(RigidBodyPtr a, RigidBodyPtr b, Vector2 localAnchorA = {}, Vector2 localAnchorB = {});
    // frequency >= 0 (0 = rigid), damping ratio >= 0. The frequency applies to
    // the row's own effective mass: for the linear part the anchor's point
    // mass, for the angular part 1/(invInertiaA + invInertiaB).
    void setLinearSpring(float frequency, float dampingRatio);
    void setAngularSpring(float frequency, float dampingRatio);
    float getLinearFrequency() const { return linearFrequency; }
    float getLinearDampingRatio() const { return linearDamping; }
    float getAngularFrequency() const { return angularFrequency; }
    float getAngularDampingRatio() const { return angularDamping; }
    // B's angle relative to A minus the construction angle, in (-pi, pi].
    double getAngle() const;
    double getReferenceAngle() const { return referenceAngle; }
    // Distance between the two world anchors.
    double getSeparation() const;
protected:
    void prepareStep(float deltaTime) override;
    void warmStart(float factor) override;
    void solveVelocity() override;
    bool solvePosition(float tolerance, float maxCorrection) override;
private:
    double referenceAngle;
    float linearFrequency = 0, linearDamping = 0;
    float angularFrequency = 0, angularDamping = 0;
    float stepDuration = 0;
    double linearImpulseX = 0, linearImpulseY = 0; // on B at the anchor
    double angularImpulse = 0;                     // on B
};
}

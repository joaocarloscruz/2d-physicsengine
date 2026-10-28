#pragma once
#include "joints.h"

namespace PhysicsEngine {
// Pulls an anchor on `body` toward a movable world-space target through a
// soft spring (frequency in Hz, damping ratio) whose force is capped at
// maxForce. Used for dragging bodies with a mouse or touch.
//
// IJoint joins two bodies, so the joint's body A is `ground`: any static body
// already in the World, such as the level's ground, or a small static body with
// collision mask 0 kept for dragging. Its pose is ignored; the target is in
// world coordinates. Body B is the dragged body. Unlike other joints,
// collideConnected defaults to true, so the dragged body still collides with
// the ground body it is attached to.
class MouseJoint final : public IJoint {
public:
    MouseJoint(RigidBodyPtr ground, RigidBodyPtr body, Vector2 localAnchor, Vector2 target,
        float frequency, float dampingRatio, float maxForce);
    // Changing the target wakes the body.
    void setTarget(Vector2 target);
    Vector2 getTarget() const { return target; }
    Vector2 getLocalAnchor() const { return localB; }
    // frequency > 0 Hz, damping ratio >= 0. The frequency is that of the
    // anchor's effective mass; for an anchor at the center of mass it is the
    // mass-spring frequency of the whole body.
    void setSpring(float frequency, float dampingRatio);
    float getFrequency() const { return frequency; }
    float getDampingRatio() const { return dampingRatio; }
    void setMaxForce(float maxForce);
    float getMaxForce() const { return maxForce; }
    // Last step's spring force on the body (world frame), magnitude <= maxForce.
    Vector2 getReactionForce() const;
protected:
    void prepareStep(float deltaTime) override;
    void warmStart(float factor) override;
    void solveVelocity() override;
    bool solvePosition(float tolerance, float maxCorrection) override;
private:
    Vector2 target;
    float frequency, dampingRatio, maxForce;
    float stepDuration = 0;
    double impulseX = 0, impulseY = 0; // accumulated, on the body
};
}

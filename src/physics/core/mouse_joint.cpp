#include "physics/core/mouse_joint.h"
#include "joint_solver_math.h"
#include <algorithm>
#include <utility>

namespace PhysicsEngine {
using namespace Detail::JointMath;
namespace {
void ValidateMouse(Vector2 target, float frequency, float dampingRatio, float maxForce) {
    if (!Finite(target)) throw std::invalid_argument("Mouse joint target must be finite.");
    ValidateSpring(frequency, dampingRatio, false,
        "Mouse joint frequency must be positive and finite and damping ratio finite and non-negative.");
    if (!std::isfinite(maxForce) || maxForce < 0)
        throw std::invalid_argument("Mouse joint maximum force must be finite and non-negative.");
}
}
MouseJoint::MouseJoint(RigidBodyPtr ground, RigidBodyPtr body, Vector2 localAnchor, Vector2 target,
    float frequency, float dampingRatio, float maxForce)
    : IJoint(std::move(ground), std::move(body), {}, localAnchor), target(target),
      frequency(frequency), dampingRatio(dampingRatio), maxForce(maxForce) {
    if (!a->IsStatic()) throw std::invalid_argument("Mouse joint ground body must be static.");
    ValidateMouse(target, frequency, dampingRatio, maxForce);
    collideConnected = true;
}
void MouseJoint::setTarget(Vector2 next) {
    if (!Finite(next)) throw std::invalid_argument("Mouse joint target must be finite.");
    if (next == target) return;
    target = next;
    b->Wake();
}
void MouseJoint::setSpring(float f, float zeta) {
    ValidateSpring(f, zeta, false,
        "Mouse joint frequency must be positive and finite and damping ratio finite and non-negative.");
    if (f == frequency && zeta == dampingRatio) return;
    frequency = f;
    dampingRatio = zeta;
    b->Wake();
}
void MouseJoint::setMaxForce(float force) {
    if (!std::isfinite(force) || force < 0)
        throw std::invalid_argument("Mouse joint maximum force must be finite and non-negative.");
    if (force == maxForce) return;
    maxForce = force;
    b->Wake();
}
Vector2 MouseJoint::getReactionForce() const {
    if (stepDuration <= 0) return {};
    return {CheckedFloat(impulseX / stepDuration), CheckedFloat(impulseY / stepDuration)};
}
void MouseJoint::prepareStep(float deltaTime) {
    IJoint::prepareStep(deltaTime);
    stepDuration = deltaTime;
    if (deltaTime <= 0) impulseX = impulseY = 0;
}
void MouseJoint::warmStart(float factor) {
    impulseX *= factor;
    impulseY *= factor;
    // A lowered force cap applies to the carried impulse too.
    const double cap = double(maxForce) * stepDuration, length = std::hypot(impulseX, impulseY);
    if (length > cap) {
        const double scale = length > 0 ? cap / length : 0;
        impulseX *= scale;
        impulseY *= scale;
    }
    if (impulseX == 0 && impulseY == 0) return;
    const Frame f(*a, *b, localA, localB);
    ApplyPoint(*a, *b, f, {impulseX, impulseY}, 0, false);
}
void MouseJoint::solveVelocity() {
    // A soft constraint needs elapsed time; a zero step leaves the body alone.
    if (stepDuration <= 0) return;
    const Frame f(*a, *b, localA, localB);
    const double h = stepDuration;
    const Softness soft = MakeSoft(frequency, dampingRatio, h);
    // Ground pose is ignored: the error is the body's anchor minus the target.
    const D2 anchor = Double(b->position) + f.rb;
    const D2 error = anchor - Double(target);
    const D2 velocity = PointVelocity(*b, f.rb);
    const D2 rhs = (velocity + error * soft.biasRate) * -1;
    const D2 response = SolvePointMass(*a, *b, f.ra, f.rb, rhs);
    const D2 accumulated{impulseX, impulseY};
    D2 next = accumulated + response * soft.massScale - accumulated * soft.impulseScale;
    const double cap = double(maxForce) * h, length = std::hypot(next.x, next.y);
    if (length > cap) next = length > 0 ? next * (cap / length) : D2{};
    const D2 change{Checked(next.x - impulseX), Checked(next.y - impulseY)};
    // Only B moves: A is static.
    ApplyGeneral(*a, *b, change, 0, f.rb.cross(change), false);
    impulseX = next.x;
    impulseY = next.y;
}
bool MouseJoint::solvePosition(float tolerance, float maxCorrection) {
    // Soft: the spring is the position correction.
    ValidateCorrection(tolerance, maxCorrection);
    return true;
}
}

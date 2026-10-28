#include "physics/core/weld_joint.h"
#include "joint_solver_math.h"
#include <algorithm>
#include <utility>

namespace PhysicsEngine {
using namespace Detail::JointMath;
namespace {
constexpr const char *SpringMessage =
    "Weld spring frequency and damping ratio must be finite and non-negative.";
}
WeldJoint::WeldJoint(RigidBodyPtr a, RigidBodyPtr b, Vector2 la, Vector2 lb)
    : IJoint(std::move(a), std::move(b), la, lb) {
    referenceAngle = std::remainder(double(this->b->orientation) - this->a->orientation, 2 * Pi);
}
void WeldJoint::setLinearSpring(float frequency, float dampingRatio) {
    ValidateSpring(frequency, dampingRatio, true, SpringMessage);
    if (frequency == linearFrequency && dampingRatio == linearDamping) return;
    linearFrequency = frequency;
    linearDamping = dampingRatio;
    a->Wake();
    b->Wake();
}
void WeldJoint::setAngularSpring(float frequency, float dampingRatio) {
    ValidateSpring(frequency, dampingRatio, true, SpringMessage);
    if (frequency == angularFrequency && dampingRatio == angularDamping) return;
    angularFrequency = frequency;
    angularDamping = dampingRatio;
    a->Wake();
    b->Wake();
}
double WeldJoint::getAngle() const {
    ValidateBody(*a);
    ValidateBody(*b);
    return std::remainder(double(b->orientation) - a->orientation - referenceAngle, 2 * Pi);
}
double WeldJoint::getSeparation() const {
    const Frame f(*a, *b, localA, localB);
    return std::hypot(f.delta.x, f.delta.y);
}
void WeldJoint::prepareStep(float deltaTime) {
    IJoint::prepareStep(deltaTime);
    stepDuration = deltaTime;
    if (deltaTime <= 0) linearImpulseX = linearImpulseY = angularImpulse = 0;
}
void WeldJoint::warmStart(float factor) {
    linearImpulseX *= factor;
    linearImpulseY *= factor;
    angularImpulse *= factor;
    if (linearImpulseX == 0 && linearImpulseY == 0 && angularImpulse == 0) return;
    const Frame f(*a, *b, localA, localB);
    ApplyPoint(*a, *b, f, {linearImpulseX, linearImpulseY}, angularImpulse, false);
}
void WeldJoint::solveVelocity() {
    const double h = stepDuration;
    const bool softLinear = linearFrequency > 0, softAngular = angularFrequency > 0;
    if (!softLinear && !softAngular) {
        // Rigid: one block solve, exact for a lone weld in one iteration.
        const Frame f(*a, *b, localA, localB);
        const D2 dv = RelativeVelocity(*a, *b, f);
        const double dw = double(b->angularVelocity) - a->angularVelocity;
        const auto j = SolveWeldMass(*a, *b, f.ra, f.rb, dv * -1, -dw);
        ApplyPoint(*a, *b, f, j.linear, j.angular, false);
        linearImpulseX = Checked(linearImpulseX + j.linear.x);
        linearImpulseY = Checked(linearImpulseY + j.linear.y);
        angularImpulse = Checked(angularImpulse + j.angular);
        return;
    }
    // Mixed or fully soft: angular row, then the point row (Gauss-Seidel, as
    // Box2D v3's weld joint). A soft row needs elapsed time; it is skipped on
    // a zero step.
    if (!softAngular || h > 0) {
        const double k = double(a->inverseInertia) + b->inverseInertia;
        const double dw = double(b->angularVelocity) - a->angularVelocity;
        double lambda;
        if (softAngular) {
            const Softness s = MakeSoft(angularFrequency, angularDamping, h);
            lambda = -s.massScale * (dw + s.biasRate * getAngle()) / k - s.impulseScale * angularImpulse;
        } else {
            lambda = -dw / k;
        }
        ApplyGeneral(*a, *b, {}, Checked(lambda), lambda, false);
        angularImpulse = Checked(angularImpulse + lambda);
    }
    if (!softLinear || h > 0) {
        const Frame f(*a, *b, localA, localB);
        const D2 dv = RelativeVelocity(*a, *b, f);
        D2 lambda;
        if (softLinear) {
            const Softness s = MakeSoft(linearFrequency, linearDamping, h);
            const D2 response = SolvePointMass(*a, *b, f.ra, f.rb, (dv + f.delta * s.biasRate) * -1);
            lambda = response * s.massScale - D2{linearImpulseX, linearImpulseY} * s.impulseScale;
        } else {
            lambda = SolvePointMass(*a, *b, f.ra, f.rb, dv * -1);
        }
        ApplyPoint(*a, *b, f, lambda, 0, false);
        linearImpulseX = Checked(linearImpulseX + lambda.x);
        linearImpulseY = Checked(linearImpulseY + lambda.y);
    }
}
bool WeldJoint::solvePosition(float tolerance, float maxCorrection) {
    ValidateCorrection(tolerance, maxCorrection);
    const bool rigidLinear = linearFrequency == 0, rigidAngular = angularFrequency == 0;
    // Soft rows are corrected by their springs, not by projection.
    if (!rigidLinear && !rigidAngular) return true;
    const Frame f(*a, *b, localA, localB);
    D2 error = f.delta;
    const double distance = std::hypot(error.x, error.y);
    const double angle = getAngle();
    if (distance > maxCorrection) error = error * (maxCorrection / distance);
    const double turn = -std::clamp(angle, -MaxAngularCorrection, MaxAngularCorrection);
    if (rigidLinear && rigidAngular) {
        const auto j = SolveWeldMass(*a, *b, f.ra, f.rb, error * -1, turn);
        ApplyPoint(*a, *b, f, j.linear, j.angular, true);
        return distance <= tolerance && std::abs(angle) <= AngularTolerance;
    }
    if (rigidAngular) {
        const double k = double(a->inverseInertia) + b->inverseInertia;
        ApplyGeneral(*a, *b, {}, turn / k, turn / k, true);
        return std::abs(angle) <= AngularTolerance;
    }
    ApplyPoint(*a, *b, f, SolvePointMass(*a, *b, f.ra, f.rb, error * -1), 0, true);
    return distance <= tolerance;
}
}

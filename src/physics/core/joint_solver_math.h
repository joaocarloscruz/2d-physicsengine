#ifndef PHYSICS_JOINT_SOLVER_MATH_H
#define PHYSICS_JOINT_SOLVER_MATH_H
// Internal helpers shared by the mouse, weld and wheel joints. Same rules as
// joints.cpp: double-precision internals, nonfinite intermediates throw
// std::overflow_error, and both endpoints are staged before either is written.
#include "physics/core/rigidbody.h"
#include <cmath>
#include <limits>
#include <stdexcept>

namespace PhysicsEngine {
namespace Detail {
namespace JointMath {
constexpr double Pi = 3.14159265358979323846;
constexpr double AngularTolerance = 0.005;
constexpr double MaxAngularCorrection = 0.2;

struct D2 {
    double x = 0, y = 0;
    D2 operator+(D2 v) const { return {x + v.x, y + v.y}; }
    D2 operator-(D2 v) const { return {x - v.x, y - v.y}; }
    D2 operator*(double s) const { return {x * s, y * s}; }
    double dot(D2 v) const { return x * v.x + y * v.y; }
    double cross(D2 v) const { return x * v.y - y * v.x; }
    D2 perp() const { return {-y, x}; }
};
inline D2 Double(Vector2 v) { return {v.x, v.y}; }
inline double Checked(double value) {
    if (!std::isfinite(value)) throw std::overflow_error("Joint arithmetic is not finite.");
    return value;
}
inline float CheckedFloat(double value) {
    if (!std::isfinite(value) || std::abs(value) > std::numeric_limits<float>::max())
        throw std::overflow_error("Joint state is not representable.");
    return static_cast<float>(value);
}
inline D2 Rotate(D2 v, double angle) {
    const double c = std::cos(angle), s = std::sin(angle);
    return {c * v.x - s * v.y, s * v.x + c * v.y};
}
inline D2 Rotate(Vector2 v, double angle) { return Rotate(Double(v), angle); }
inline bool Finite(Vector2 v) { return std::isfinite(v.x) && std::isfinite(v.y); }

inline void ValidateBody(const RigidBody &body) {
    const double values[] = {body.position.x,  body.position.y,    body.orientation,
                             body.velocity.x,  body.velocity.y,    body.angularVelocity,
                             body.inverseMass, body.inverseInertia};
    for (double value : values)
        if (!std::isfinite(value)) throw std::invalid_argument("Joint requires finite body state.");
    if (body.IsStatic() ? body.inverseMass != 0 || body.inverseInertia != 0
                        : body.inverseMass <= 0 || body.inverseInertia <= 0)
        throw std::invalid_argument(
            "Joint requires zero static or positive dynamic inverse properties.");
}
inline void ValidateCorrection(float tolerance, float maximum) {
    if (!std::isfinite(tolerance) || tolerance < 0 || !std::isfinite(maximum) || maximum < 0)
        throw std::invalid_argument("Joint correction bounds must be finite and non-negative.");
}
// frequency == 0 means rigid; otherwise both must be finite, frequency > 0
// and damping ratio >= 0.
inline void ValidateSpring(float frequency, float dampingRatio, bool allowRigid, const char *message) {
    if (!std::isfinite(frequency) || !std::isfinite(dampingRatio) || dampingRatio < 0 ||
        (allowRigid ? frequency < 0 : frequency <= 0))
        throw std::invalid_argument(message);
}

// Soft constraint coefficients for one step h (Catto, "Solver2D" / Box2D v3
// b2MakeSoft). With effective mass M of the row, the impulse
//   lambda = -massScale * M * (Cdot + biasRate * C) - impulseScale * accumulated
// is the implicit (backward) Euler step of a spring-damper whose undamped
// frequency is `frequency` Hz and damping ratio `dampingRatio` on that mass.
struct Softness {
    double biasRate = 0, massScale = 1, impulseScale = 0;
};
inline Softness MakeSoft(double frequency, double dampingRatio, double h) {
    if (frequency == 0 || h <= 0) return {};
    const double omega = 2 * Pi * frequency;
    const double a1 = 2 * dampingRatio + h * omega;
    const double a2 = h * omega * a1;
    const double a3 = 1 / (1 + a2);
    return {Checked(omega / a1), Checked(a2 * a3), Checked(a3)};
}

// Lever arms and anchor separation (B's anchor minus A's), in world frame.
struct Frame {
    D2 ra, rb, delta;
    Frame(const RigidBody &a, const RigidBody &b, Vector2 la, Vector2 lb) {
        ValidateBody(a);
        ValidateBody(b);
        ra = Rotate(la, a.orientation);
        rb = Rotate(lb, b.orientation);
        delta = (Double(b.position) - Double(a.position)) + (rb - ra);
    }
};
inline D2 PointVelocity(const RigidBody &body, D2 r) {
    return Double(body.velocity) + r.perp() * body.angularVelocity;
}
// Relative anchor velocity, B minus A.
inline D2 RelativeVelocity(const RigidBody &a, const RigidBody &b, const Frame &f) {
    return PointVelocity(b, f.rb) - PointVelocity(a, f.ra);
}

// Applies linear impulse p on B at rb (and -p on A at ra) plus angular
// impulses: B receives torque impulse lb + rb x p and A receives -(la + ra x p).
// `position` applies the same update to poses instead of velocities.
// Both endpoints are staged and checked before either is published.
inline void ApplyGeneral(RigidBody &a, RigidBody &b, D2 p, double angularA, double angularB,
                         bool position) {
    Checked(p.x);
    Checked(p.y);
    Checked(angularA);
    Checked(angularB);
    const Vector2 la = position ? a.position : a.velocity;
    const Vector2 lb = position ? b.position : b.velocity;
    const double wa = position ? a.orientation : a.angularVelocity;
    const double wb = position ? b.orientation : b.angularVelocity;
    const Vector2 na{CheckedFloat(la.x - p.x * a.inverseMass), CheckedFloat(la.y - p.y * a.inverseMass)};
    const Vector2 nb{CheckedFloat(lb.x + p.x * b.inverseMass), CheckedFloat(lb.y + p.y * b.inverseMass)};
    const float aw = CheckedFloat(wa - a.inverseInertia * angularA);
    const float bw = CheckedFloat(wb + b.inverseInertia * angularB);
    if (!a.IsStatic()) {
        if (position) { a.position = na; a.orientation = aw; }
        else { a.velocity = na; a.angularVelocity = aw; }
    }
    if (!b.IsStatic()) {
        if (position) { b.position = nb; b.orientation = bw; }
        else { b.velocity = nb; b.angularVelocity = bw; }
    }
}
inline void ApplyPoint(RigidBody &a, RigidBody &b, const Frame &f, D2 p, double angular,
                       bool position) {
    ApplyGeneral(a, b, p, angular + f.ra.cross(p), angular + f.rb.cross(p), position);
}

// Solves K p = rhs for the 2x2 point mass of anchors ra, rb.
inline D2 SolvePointMass(const RigidBody &a, const RigidBody &b, D2 ra, D2 rb, D2 rhs) {
    const double m = double(a.inverseMass) + b.inverseMass;
    const double ia = a.inverseInertia, ib = b.inverseInertia;
    // Expanded into positive terms; k11*k22 - k12^2 cancels for long levers.
    const double cross = ra.cross(rb);
    const double determinant =
        Checked(m * (m + ia * ra.dot(ra) + ib * rb.dot(rb)) + ia * ib * cross * cross);
    if (determinant <= 0) throw std::overflow_error("Joint point mass is singular.");
    const D2 numerator = rhs * m + ra * (ia * ra.dot(rhs)) + rb * (ib * rb.dot(rhs));
    return {Checked(numerator.x / determinant), Checked(numerator.y / determinant)};
}

// Solves the 3x3 point-plus-angle system (weld) for linear impulse p and
// angular impulse l through the Schur complement of the angular row. The
// complement is m*I + w*u*u^T with u = J(ra - rb), so its determinant is the
// positive m*(m + w|ra - rb|^2) and long levers do not cancel.
struct WeldImpulse {
    D2 linear;
    double angular = 0;
};
inline WeldImpulse SolveWeldMass(const RigidBody &a, const RigidBody &b, D2 ra, D2 rb, D2 rhs,
                                 double angularRhs) {
    const double m = double(a.inverseMass) + b.inverseMass;
    const double ia = a.inverseInertia, ib = b.inverseInertia, k33 = ia + ib;
    if (k33 <= 0) throw std::overflow_error("Joint angular mass is singular.");
    const double k13 = -ia * ra.y - ib * rb.y, k23 = ia * ra.x + ib * rb.x;
    const double weight = ia * ib / k33;
    const D2 d = ra - rb;
    const double x = rhs.x - k13 * angularRhs / k33, y = rhs.y - k23 * angularRhs / k33;
    const double determinant = Checked(m * (m + weight * d.dot(d)));
    if (determinant <= 0) throw std::overflow_error("Joint weld mass is singular.");
    const double projection = d.x * x + d.y * y;
    const D2 p{Checked((m * x + weight * d.x * projection) / determinant),
               Checked((m * y + weight * d.y * projection) / determinant)};
    return {p, Checked((angularRhs - k13 * p.x - k23 * p.y) / k33)};
}
} // namespace JointMath
} // namespace Detail
} // namespace PhysicsEngine
#endif

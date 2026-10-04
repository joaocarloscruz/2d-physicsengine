#include "physics/core/joints.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace PhysicsEngine {
namespace {
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
};
D2 Double(Vector2 v) {
    return {v.x, v.y};
}
double Checked(double value) {
    if (!std::isfinite(value))
        throw std::overflow_error("Joint arithmetic is not finite.");
    return value;
}
float CheckedFloat(double value) {
    if (!std::isfinite(value) || std::abs(value) > std::numeric_limits<float>::max())
        throw std::overflow_error("Joint state is not representable.");
    return static_cast<float>(value);
}
Vector2 CheckedVector(D2 v) {
    return {CheckedFloat(v.x), CheckedFloat(v.y)};
}
void Validate(const RigidBody &body) {
    const double values[] = {body.position.x,  body.position.y,    body.orientation,
                             body.velocity.x,  body.velocity.y,    body.angularVelocity,
                             body.inverseMass, body.inverseInertia};
    for (double value : values)
        if (!std::isfinite(value))
            throw std::invalid_argument("Joint requires finite body state.");
    if (body.IsStatic() ? body.inverseMass != 0 || body.inverseInertia != 0
                        : body.inverseMass <= 0 || body.inverseInertia <= 0)
        throw std::invalid_argument(
            "Joint requires zero static or positive dynamic inverse properties.");
}
void ValidateCorrection(float tolerance, float maximum) {
    if (!std::isfinite(tolerance) || tolerance < 0 || !std::isfinite(maximum) || maximum < 0)
        throw std::invalid_argument("Joint correction bounds must be finite and non-negative.");
}
D2 Rotate(Vector2 v, double angle) {
    const double c = std::cos(angle), s = std::sin(angle);
    return {c * v.x - s * v.y, s * v.x + c * v.y};
}
struct Geometry {
    D2 ra, rb, delta;
    Geometry(const RigidBody &a, const RigidBody &b, Vector2 la, Vector2 lb) {
        Validate(a);
        Validate(b);
        ra = Rotate(la, a.orientation);
        rb = Rotate(lb, b.orientation);
        delta = (Double(b.position) - Double(a.position)) + (rb - ra);
    }
};
D2 PointVelocity(const RigidBody &b, D2 r) {
    return Double(b.velocity) + D2{-r.y, r.x} * b.angularVelocity;
}
struct Proposal {
    Vector2 linear;
    float angular;
};
struct AngularChange {
    double a = 0, b = 0;
    bool absolute = false;
};
Proposal Propose(const RigidBody &b, D2 r, D2 impulse, double angularImpulse, bool position,
                 const double *angularChange, bool absoluteAngular = false) {
    const Vector2 linear = position ? b.position : b.velocity;
    const float angular = position ? b.orientation : b.angularVelocity;
    if (b.IsStatic())
        return {linear, angular};
    return {CheckedVector(Double(linear) + impulse * b.inverseMass),
            CheckedFloat(angularChange && absoluteAngular ? *angularChange : double(angular) +
                         (angularChange ? *angularChange
                                        : b.inverseInertia * (r.cross(impulse) + angularImpulse)))};
}
void Apply(RigidBody &a, RigidBody &b, D2 ra, D2 rb, D2 impulse, bool position,
           double angularImpulse = 0, const AngularChange *change = nullptr) {
    Validate(a);
    Validate(b);
    Checked(impulse.x);
    Checked(impulse.y);
    Checked(angularImpulse);
    const auto pa =
        Propose(a, ra, impulse * -1, -angularImpulse, position, change ? &change->a : nullptr,
                change && change->absolute);
    const auto pb =
        Propose(b, rb, impulse, angularImpulse, position, change ? &change->b : nullptr,
                change && change->absolute);
    // Publish neither endpoint until both complete linear/angular proposals pass.
    if (!a.IsStatic()) {
        if (position) {
            a.position = pa.linear;
            a.orientation = pa.angular;
        } else {
            a.velocity = pa.linear;
            a.angularVelocity = pa.angular;
        }
    }
    if (!b.IsStatic()) {
        if (position) {
            b.position = pb.linear;
            b.orientation = pb.angular;
        } else {
            b.velocity = pb.linear;
            b.angularVelocity = pb.angular;
        }
    }
}
double InverseEffectiveMass(const RigidBody &a, const RigidBody &b, D2 ra, D2 rb, D2 n) {
    const double ca = ra.cross(n), cb = rb.cross(n);
    return Checked(double(a.inverseMass) + b.inverseMass + a.inverseInertia * ca * ca +
                   b.inverseInertia * cb * cb);
}
D2 SolvePointMass(const RigidBody &a, const RigidBody &b, D2 ra, D2 rb, D2 rhs) {
    const double m = double(a.inverseMass) + b.inverseMass;
    const double ia = a.inverseInertia, ib = b.inverseInertia;
    // Expand the determinant into positive mass/lever terms. Subtracting k12²
    // from k11*k22 loses the translational term for nearly parallel long levers.
    const double cross = ra.cross(rb);
    const double determinant =
        Checked(m * (m + ia * ra.dot(ra) + ib * rb.dot(rb)) + ia * ib * cross * cross);
    if (determinant <= 0)
        throw std::overflow_error("Joint point mass is singular.");
    const D2 numerator = rhs * m + ra * (ia * ra.dot(rhs)) + rb * (ib * rb.dot(rhs));
    return {Checked(numerator.x / determinant), Checked(numerator.y / determinant)};
}
struct HingeImpulse {
    D2 linear;
    double angular;
    AngularChange change;
};
HingeImpulse SolveHingeMass(const RigidBody &a, const RigidBody &b, D2 ra, D2 rb, D2 rhs,
                            double angularRhs) {
    const double ia = a.inverseInertia, ib = b.inverseInertia;
    const double k33 = ia + ib;
    if (k33 <= 0)
        return {SolvePointMass(a, b, ra, rb, rhs), 0};
    const double m = static_cast<double>(a.inverseMass) + b.inverseMass;
    const double k13 = -ia * ra.y - ib * rb.y, k23 = ia * ra.x + ib * rb.x;
    // Schur complement of the angular row. This avoids slow alternating
    // corrections when the lever arm is much longer than the body's radius.
    const double weight = ia * ib / k33;
    const double dx = static_cast<double>(ra.x) - rb.x, dy = static_cast<double>(ra.y) - rb.y;
    const double x = rhs.x - k13 * angularRhs / k33, y = rhs.y - k23 * angularRhs / k33;
    const double determinant = Checked(m * (m + weight * (dx * dx + dy * dy)));
    if (determinant <= 0)
        throw std::overflow_error("Joint hinge mass is singular.");
    const double projection = dx * x + dy * y;
    const double px = Checked((m * x + weight * dx * projection) / determinant);
    const double py = Checked((m * y + weight * dy * projection) / determinant);
    const double difference = D2{dx, dy}.cross({px, py});
    // Substitute the angular row before publication. Adding r×p to the huge
    // opposing angular impulse can otherwise erase a small net angular change.
    return {{px, py},
            Checked((angularRhs - k13 * px - k23 * py) / k33),
            {-ia * angularRhs / k33 - weight * difference,
             ib * angularRhs / k33 - weight * difference}};
}
HingeImpulse SolveHingeVelocity(const RigidBody& a,const RigidBody& b,D2 ra,D2 rb,double target) {
    const double m=double(a.inverseMass)+b.inverseMass,ia=a.inverseInertia,ib=b.inverseInertia,
        angularMass=ia+ib,weight=ia*ib/angularMass;
    const D2 d=ra-rb,dv=Double(b.velocity)-Double(a.velocity),base=dv*-1;
    const D2 lever=rb+d*(ia/angularMass);
    const double shared=(ib*a.angularVelocity+ia*b.angularVelocity)/angularMass;
    const D2 jd{-d.y,d.x},jl{-lever.y,lever.x};
    const double determinant=Checked(m*(m+weight*d.dot(d)));
    if(determinant<=0) throw std::overflow_error("Joint hinge mass is singular.");
    // Reduced RHS is -dv + J*d*shared - J*lever*target. Keep J*d
    // separate through the adjugate: d dot J*d is identically zero.
    const D2 numerator=base*m+d*(weight*d.dot(base))+jd*(m*shared)
        -jl*(m*target)+d*(weight*ra.cross(rb)*target);
    const D2 linear{Checked(numerator.x/determinant),Checked(numerator.y/determinant)};
    const double angularRhs=target-(double(b.angularVelocity)-a.angularVelocity);
    const double common=weight*d.cross(linear);
    return {linear,Checked(angularRhs/angularMass-lever.cross(linear)),
        {Checked(shared-ia/angularMass*target-common),Checked(shared+ib/angularMass*target-common),true}};
}
HingeImpulse FixedHingeVelocity(const RigidBody &a,const RigidBody &b,D2 ra,D2 rb,double angular) {
    const double m = double(a.inverseMass) + b.inverseMass;
    const double ia = a.inverseInertia, ib = b.inverseInertia, cross = ra.cross(rb);
    const double determinant =
        Checked(m * (m + ia * ra.dot(ra) + ib * rb.dot(rb)) + ia * ib * cross * cross);
    if(determinant<=0) throw std::overflow_error("Joint point mass is singular.");
    const D2 d=ra-rb,dv=Double(b.velocity)-Double(a.velocity),jrb{-rb.y,rb.x},jd{-d.y,d.x};
    const double wa=a.angularVelocity,wb=b.angularVelocity,shared=ib*wa+ia*wb;
    const D2 translation=dv*(-m)-ra*(ia*ra.dot(dv))-rb*(ib*rb.dot(dv));
    const D2 rotation=jrb*(m*(wa-wb-(ia+ib)*angular))+jd*(m*(wa-ia*angular))
        +(rb*shared+d*(ia*wb+ia*ib*angular))*cross;
    const D2 numerator=translation+rotation;
    const D2 linear{Checked(numerator.x/determinant),Checked(numerator.y/determinant)};
    // Recover final angular states directly. Forming full point speeds and
    // then adding a nearly -omega correction erases a small surviving spin.
    const double finalA=Checked((m*m*wa+m*(rb.dot(rb)*shared+ia*rb.dot(d)*wb+ia*ra.cross(dv)
        -ia*angular*(m-ib*rb.dot(d)))+ia*ib*cross*rb.dot(dv))/determinant);
    const double finalB=Checked((m*m*wb+m*(ra.dot(ra)*shared-ib*ra.dot(d)*wa-ib*rb.cross(dv)
        +ib*angular*(m+ia*ra.dot(d)))+ia*ib*cross*ra.dot(dv))/determinant);
    return {linear,angular,{finalA,finalB,true}};
}
void SolveCoupledVelocity(RigidBody &a, RigidBody &b, const Geometry &g, double targetSpeed,
                          double &accumulated, double minimum, double maximum) {
    const D2 ra = g.ra, rb = g.rb;
    const auto candidate = SolveHingeVelocity(a,b,ra,rb,targetSpeed);
    const double next = std::clamp(Checked(accumulated + candidate.angular), minimum, maximum);
    const double angular = next - accumulated;
    const auto accepted=angular==candidate.angular?candidate:FixedHingeVelocity(a,b,ra,rb,angular);
    Apply(a,b,ra,rb,accepted.linear,false,angular,&accepted.change);
    accumulated = next;
}
} // namespace

IJoint::IJoint(RigidBodyPtr first, RigidBodyPtr second, Vector2 anchorA, Vector2 anchorB)
    : a(std::move(first)), b(std::move(second)), localA(anchorA), localB(anchorB) {
    if (!a || !b || a == b || (a->IsStatic() && b->IsStatic()))
        throw std::invalid_argument(
            "A joint requires distinct bodies and at least one dynamic body.");
    if (!std::isfinite(localA.x) || !std::isfinite(localA.y) || !std::isfinite(localB.x) ||
        !std::isfinite(localB.y))
        throw std::invalid_argument("Joint anchors must be finite.");
    Validate(*a);
    Validate(*b);
}
Vector2 IJoint::getAnchorA() const {
    Validate(*a);
    return CheckedVector(Double(a->position) + Rotate(localA, a->orientation));
}
Vector2 IJoint::getAnchorB() const {
    Validate(*b);
    return CheckedVector(Double(b->position) + Rotate(localB, b->orientation));
}
void IJoint::prepareStep(float deltaTime) {
    if (!std::isfinite(deltaTime) || deltaTime < 0)
        throw std::invalid_argument("Joint step duration must be finite and non-negative.");
    Validate(*a);
    Validate(*b);
}

DistanceJoint::DistanceJoint(RigidBodyPtr a, RigidBodyPtr b, float length, Vector2 la, Vector2 lb)
    : IJoint(std::move(a), std::move(b), la, lb), length(length) {
    if (!std::isfinite(length) || length <= 0)
        throw std::invalid_argument("Distance joint length must be positive and finite.");
}
void DistanceJoint::solveVelocity() {
    const Geometry g(*a, *b, localA, localB);
    const double distance = std::hypot(g.delta.x, g.delta.y);
    const D2 n = distance != 0 ? g.delta * (1 / distance) : D2{1, 0};
    const double k = InverseEffectiveMass(*a, *b, g.ra, g.rb, n);
    Apply(*a, *b, g.ra, g.rb, n * (-(PointVelocity(*b, g.rb) - PointVelocity(*a, g.ra)).dot(n) / k),
          false);
}
bool DistanceJoint::solvePosition(float tolerance, float maxCorrection) {
    ValidateCorrection(tolerance, maxCorrection);
    const Geometry g(*a, *b, localA, localB);
    const double distance = std::hypot(g.delta.x, g.delta.y), error = distance - length;
    const D2 n = distance != 0 ? g.delta * (1 / distance) : D2{1, 0};
    const double k = InverseEffectiveMass(*a, *b, g.ra, g.rb, n);
    Apply(*a, *b, g.ra, g.rb,
          n * (-std::clamp(error, -double(maxCorrection), double(maxCorrection)) / k), true);
    return std::abs(error) <= tolerance;
}

RevoluteJoint::RevoluteJoint(RigidBodyPtr a, RigidBodyPtr b, Vector2 la, Vector2 lb)
    : IJoint(std::move(a), std::move(b), la, lb) {
    referenceAngle =
        std::remainder(static_cast<double>(this->b->orientation) - this->a->orientation, 2 * Pi);
}
void RevoluteJoint::setLimits(bool enabled, float lowerAngle, float upperAngle) {
    if (!std::isfinite(lowerAngle) || !std::isfinite(upperAngle) || lowerAngle > upperAngle ||
        lowerAngle <= -Pi || upperAngle >= Pi)
        throw std::invalid_argument(
            "Joint limits must be ordered finite angles strictly between -pi and pi.");
    if (limitsEnabled == enabled && lowerLimit == lowerAngle && upperLimit == upperAngle)
        return;
    limitsEnabled = enabled;
    lowerLimit = lowerAngle;
    upperLimit = upperAngle;
    a->Wake();
    b->Wake();
}
double RevoluteJoint::getAngle() const {
    Validate(*a);
    Validate(*b);
    return std::remainder(static_cast<double>(b->orientation) - a->orientation - referenceAngle,
                          2 * Pi);
}
void RevoluteJoint::setMotor(bool enabled, float speed, float maxTorque) {
    if (!std::isfinite(speed) || !std::isfinite(maxTorque) || maxTorque < 0)
        throw std::invalid_argument(
            "Motor speed must be finite and maximum torque finite and non-negative.");
    if (motorEnabled == enabled && motorSpeed == speed && maxMotorTorque == maxTorque)
        return;
    motorEnabled = enabled;
    motorSpeed = speed;
    maxMotorTorque = maxTorque;
    a->Wake();
    b->Wake();
}
double RevoluteJoint::getMotorTorque() const {
    return stepDuration > 0 ? motorImpulse / stepDuration : 0;
}
void RevoluteJoint::prepareStep(float deltaTime) {
    IJoint::prepareStep(deltaTime);
    stepDuration = deltaTime;
    motorImpulse = 0;
    lowerImpulse = 0;
    upperImpulse = 0;
}
bool RevoluteJoint::preventsSleeping() const {
    return motorEnabled && maxMotorTorque > 0 && motorSpeed != 0;
}
void RevoluteJoint::solveVelocity() {
    const Geometry g(*a, *b, localA, localB);
    const double angularMass = static_cast<double>(a->inverseInertia) + b->inverseInertia;
    if (motorEnabled && stepDuration > 0 && angularMass > 0) {
        const double cap = static_cast<double>(maxMotorTorque) * stepDuration;
        SolveCoupledVelocity(*a, *b, g, motorSpeed, motorImpulse, -cap, cap);
    }
    if (limitsEnabled && angularMass > 0) {
        if (lowerLimit == upperLimit) {
            const auto impulse = SolveHingeVelocity(*a,*b,g.ra,g.rb,0);
            Apply(*a, *b, g.ra, g.rb, impulse.linear, false, impulse.angular, &impulse.change);
        } else {
            const double angle = getAngle();
            const double lowerGap = angle - lowerLimit;
            if (stepDuration > 0 || lowerGap <= 0) {
                const double bias = stepDuration > 0 ? std::max(lowerGap, 0.0) / stepDuration : 0;
                SolveCoupledVelocity(*a, *b, g, -bias, lowerImpulse, 0,
                                     std::numeric_limits<double>::infinity());
            }
            const double upperGap = upperLimit - angle;
            if (stepDuration > 0 || upperGap <= 0) {
                const double bias = stepDuration > 0 ? std::max(upperGap, 0.0) / stepDuration : 0;
                SolveCoupledVelocity(*a, *b, g, bias, upperImpulse,
                                     -std::numeric_limits<double>::infinity(), 0);
            }
        }
        // The block solve already enforces the point constraint. With zero dt
        // and an inactive interval neither stop is solved, so solve the point below.
        if (stepDuration > 0 || lowerLimit == upperLimit || getAngle() <= lowerLimit ||
            getAngle() >= upperLimit)
            return;
    }
    const D2 velocity = PointVelocity(*b, g.rb) - PointVelocity(*a, g.ra);
    Apply(*a, *b, g.ra, g.rb, SolvePointMass(*a, *b, g.ra, g.rb, velocity * -1), false);
}
bool RevoluteJoint::solvePosition(float tolerance, float maxCorrection) {
    ValidateCorrection(tolerance, maxCorrection);
    const Geometry g(*a, *b, localA, localB);
    D2 error = g.delta;
    const double distance = std::hypot(error.x, error.y);
    if (distance > maxCorrection)
        error = error * (maxCorrection / distance);
    double angularError = 0;
    if (limitsEnabled) {
        const double angle = getAngle();
        angularError = angle - std::clamp(angle, static_cast<double>(lowerLimit),
                                          static_cast<double>(upperLimit));
        if (angularError != 0 || lowerLimit == upperLimit) {
            const auto impulse = SolveHingeMass(
                *a, *b, g.ra, g.rb, error * -1,
                -std::clamp(angularError, -MaxAngularCorrection, MaxAngularCorrection));
            Apply(*a, *b, g.ra, g.rb, impulse.linear, true, impulse.angular, &impulse.change);
            return distance <= tolerance && std::abs(angularError) <= AngularTolerance;
        }
    }
    Apply(*a, *b, g.ra, g.rb, SolvePointMass(*a, *b, g.ra, g.rb, error * -1), true);
    return distance <= tolerance && std::abs(angularError) <= AngularTolerance;
}
} // namespace PhysicsEngine

#include "physics/core/prismatic_joint.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace PhysicsEngine {
namespace {
constexpr double Pi = 3.14159265358979323846;
struct D2 {
    double x, y;
    D2 operator+(D2 v) const { return {x+v.x, y+v.y}; }
    D2 operator-(D2 v) const { return {x-v.x, y-v.y}; }
    D2 operator*(double s) const { return {x*s, y*s}; }
    double dot(D2 v) const { return x*v.x+y*v.y; }
    double cross(D2 v) const { return x*v.y-y*v.x; }
};
D2 Rotate(D2 v, double angle) {
    const double c = std::cos(angle), s = std::sin(angle);
    return {c*v.x-s*v.y, s*v.x+c*v.y};
}
float Checked(double value) {
    if (!std::isfinite(value) || std::abs(value) > std::numeric_limits<float>::max())
        throw std::overflow_error("Prismatic joint state is not representable");
    return static_cast<float>(value);
}
void Validate(const RigidBody& body) {
    const double values[] = {body.position.x, body.position.y, body.orientation,
        body.velocity.x, body.velocity.y, body.angularVelocity, body.inverseMass, body.inverseInertia};
    for (double value : values)
        if (!std::isfinite(value)) throw std::invalid_argument("Prismatic joint requires finite body state");
    if (body.inverseMass < 0 || body.inverseInertia < 0 ||
        (!body.IsStatic() && (body.inverseMass == 0 || body.inverseInertia == 0)))
        throw std::invalid_argument("Prismatic joint requires positive dynamic inverse mass and inertia");
}
struct Geometry {
    D2 e, n, d;
    double sa, sb, aa, ab, m, ia, ib, kpa, kaa, kpx, kax;
    Geometry(const RigidBody& a, const RigidBody& b, Vector2 la, Vector2 lb, D2 axis) {
        Validate(a); Validate(b);
        e = Rotate(axis, a.orientation); n = {-e.y,e.x};
        const D2 ra = Rotate({la.x,la.y},a.orientation), rb = Rotate({lb.x,lb.y},b.orientation);
        d = D2{b.position.x,b.position.y}+rb-(D2{a.position.x,a.position.y}+ra);
        // A's rotating axis contributes d to its lever arm. Omitting it loses
        // angular momentum when the anchors are separated along the slide.
        sa = (ra+d).cross(n); sb = rb.cross(n);
        aa = (ra+d).cross(e); ab = rb.cross(e);
        m = double(a.inverseMass)+b.inverseMass; ia=a.inverseInertia; ib=b.inverseInertia;
        kpa=ia*sa+ib*sb; kaa=ia+ib;
        kpx=ia*sa*aa+ib*sb*ab; kax=ia*aa+ib*ab;
    }
    double speed(const RigidBody& a, const RigidBody& b, bool axial) const {
        const D2 dv{double(b.velocity.x)-a.velocity.x,double(b.velocity.y)-a.velocity.y};
        return (axial ? e : n).dot(dv)+(axial ? ab : sb)*b.angularVelocity
            -(axial ? aa : sa)*a.angularVelocity;
    }
};
struct Impulse { double transverse=0, angular=0, axial=0; };
Impulse SolveBase(const Geometry& g, double p, double angle) {
    const double weight=g.ia*g.ib/g.kaa, ds=g.sa-g.sb;
    const double schur=g.m+weight*ds*ds;
    Impulse result;
    result.transverse=(p-g.kpa*angle/g.kaa)/schur;
    result.angular=(angle-g.kpa*result.transverse)/g.kaa;
    return result;
}
Impulse SolveAxial(const Geometry& g, double p, double angle, double axial) {
    const double weight=g.ia*g.ib/g.kaa, ds=g.sa-g.sb, dx=g.aa-g.ab;
    const double kp=g.m+weight*ds*ds, kx=g.m+weight*dx*dx, cross=weight*ds*dx;
    // Stable determinant after eliminating the angle row. This form avoids
    // cancellation of long lever terms in kp*kx-cross*cross.
    const double determinant=g.m*(g.m+weight*(ds*ds+dx*dx));
    const double rp=p-g.kpa*angle/g.kaa, rx=axial-g.kax*angle/g.kaa;
    Impulse result;
    result.transverse=(kx*rp-cross*rx)/determinant;
    result.axial=(kp*rx-cross*rp)/determinant;
    result.angular=(angle-g.kpa*result.transverse-g.kax*result.axial)/g.kaa;
    return result;
}
Impulse FixedAxial(const Geometry& g, double p, double angle, double axial) {
    auto result=SolveBase(g,p-g.kpx*axial,angle-g.kax*axial);
    result.axial=axial; return result;
}
void Apply(RigidBody& a, RigidBody& b, const Geometry& g, Impulse j, bool position) {
    const D2 p=g.n*j.transverse+g.e*j.axial;
    const D2 va=position ? D2{a.position.x,a.position.y} : D2{a.velocity.x,a.velocity.y};
    const D2 vb=position ? D2{b.position.x,b.position.y} : D2{b.velocity.x,b.velocity.y};
    const D2 na=va-p*a.inverseMass, nb=vb+p*b.inverseMass;
    const double wa=(position ? a.orientation : a.angularVelocity)
        -g.ia*(g.sa*j.transverse+j.angular+g.aa*j.axial);
    const double wb=(position ? b.orientation : b.angularVelocity)
        +g.ib*(g.sb*j.transverse+j.angular+g.ab*j.axial);
    // Stage both endpoints before publishing any component.
    const Vector2 fa{Checked(na.x),Checked(na.y)}, fb{Checked(nb.x),Checked(nb.y)};
    const float aw=Checked(wa), bw=Checked(wb);
    if (position) { a.position=fa; b.position=fb; a.orientation=aw; b.orientation=bw; }
    else { a.velocity=fa; b.velocity=fb; a.angularVelocity=aw; b.angularVelocity=bw; }
}
}
PrismaticJoint::PrismaticJoint(RigidBodyPtr a, RigidBodyPtr b, Vector2 axis, Vector2 la, Vector2 lb)
    : IJoint(std::move(a),std::move(b),la,lb) {
    const double norm=std::hypot(double(axis.x),double(axis.y));
    if (!std::isfinite(norm) || norm==0) throw std::invalid_argument("Prismatic axis must be finite and nonzero");
    axisX=axis.x/norm; axisY=axis.y/norm;
    Validate(*this->a); Validate(*this->b);
    referenceAngle=std::remainder(double(this->b->orientation)-this->a->orientation,2*Pi);
}
Vector2 PrismaticJoint::getLocalAxis() const { return {float(axisX),float(axisY)}; }
Vector2 PrismaticJoint::getAxis() const {
    const D2 e=Geometry(*a,*b,localA,localB,{axisX,axisY}).e;
    return {float(e.x),float(e.y)};
}
double PrismaticJoint::getTranslation() const {
    const Geometry g(*a,*b,localA,localB,{axisX,axisY}); return g.e.dot(g.d);
}
double PrismaticJoint::getTransverseError() const {
    const Geometry g(*a,*b,localA,localB,{axisX,axisY}); return g.n.dot(g.d);
}
double PrismaticJoint::getTranslationSpeed() const {
    const Geometry g(*a,*b,localA,localB,{axisX,axisY}); return g.speed(*a,*b,true);
}
double PrismaticJoint::getAngle() const {
    Validate(*a); Validate(*b);
    return std::remainder(double(b->orientation)-a->orientation-referenceAngle,2*Pi);
}
void PrismaticJoint::solveVelocity() {
    const Geometry g(*a,*b,localA,localB,{axisX,axisY});
    auto solve=[&](double target, double& total, double lower, double upper) {
        const double p=-g.speed(*a,*b,false), angle=double(a->angularVelocity)-b->angularVelocity;
        const auto candidate=SolveAxial(g,p,angle,target-g.speed(*a,*b,true));
        const double next=std::clamp(total+candidate.axial,lower,upper), actual=next-total;
        const auto j=FixedAxial(g,p,angle,actual);
        Apply(*a,*b,g,j,false); total=next;
    };
    if (motorEnabled && maxMotorForce>0 && stepDuration>0) {
        const double cap=double(maxMotorForce)*stepDuration;
        solve(motorSpeed,motorImpulse,-cap,cap);
    }
    if (limitsEnabled) {
        const double x=g.e.dot(g.d), infinity=std::numeric_limits<double>::infinity();
        if (lowerLimit==upperLimit) {
            solve(0,lowerImpulse,-infinity,infinity); return;
        }
        if (stepDuration>0 || x<=lowerLimit)
            solve(stepDuration>0 ? -std::max(x-lowerLimit,0.0)/stepDuration : 0,
                lowerImpulse,0,infinity);
        if (stepDuration>0 || x>=upperLimit)
            solve(stepDuration>0 ? std::max(double(upperLimit)-x,0.0)/stepDuration : 0,
                upperImpulse,-infinity,0);
    }
    Apply(*a,*b,g,SolveBase(g,-g.speed(*a,*b,false),double(a->angularVelocity)-b->angularVelocity),false);
}
bool PrismaticJoint::solvePosition(float tolerance, float maxCorrection) {
    const Geometry g(*a,*b,localA,localB,{axisX,axisY});
    const double error=g.n.dot(g.d), angle=getAngle();
    const double p=-std::clamp(error,-double(maxCorrection),double(maxCorrection));
    const double w=-std::clamp(angle,-0.2,0.2), x=g.e.dot(g.d);
    const double travel=limitsEnabled ? x-std::clamp(x,double(lowerLimit),double(upperLimit)) : 0;
    const bool stop=limitsEnabled && (travel!=0 || lowerLimit==upperLimit);
    Apply(*a,*b,g,stop ? SolveAxial(g,p,w,-std::clamp(travel,-double(maxCorrection),double(maxCorrection)))
        : SolveBase(g,p,w),true);
    return std::abs(error)<=tolerance && std::abs(angle)<=0.005 && std::abs(travel)<=tolerance;
}
void PrismaticJoint::setMotor(bool enabled, float speed, float maxForce) {
    if (!std::isfinite(speed) || !std::isfinite(maxForce) || maxForce<0)
        throw std::invalid_argument("Prismatic motor requires finite speed and nonnegative finite force");
    if (motorEnabled!=enabled || motorSpeed!=speed || maxMotorForce!=maxForce) { a->Wake(); b->Wake(); }
    motorEnabled=enabled; motorSpeed=speed; maxMotorForce=maxForce;
}
void PrismaticJoint::setLimits(bool enabled, float lower, float upper) {
    if (!std::isfinite(lower) || !std::isfinite(upper) || lower>upper)
        throw std::invalid_argument("Prismatic limits must be finite and ordered");
    if (limitsEnabled!=enabled || lowerLimit!=lower || upperLimit!=upper) { a->Wake(); b->Wake(); }
    limitsEnabled=enabled; lowerLimit=lower; upperLimit=upper;
}
double PrismaticJoint::getMotorForce() const { return stepDuration>0 ? motorImpulse/stepDuration : 0; }
void PrismaticJoint::prepareStep(float dt) {
    stepDuration=dt; motorImpulse=lowerImpulse=upperImpulse=0;
}
bool PrismaticJoint::preventsSleeping() const { return motorEnabled && maxMotorForce>0 && motorSpeed!=0; }
}

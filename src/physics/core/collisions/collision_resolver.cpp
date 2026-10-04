#include "physics/core/collisions/collision_resolver.h"
#include "physics/core/rigidbody.h"
#include "normal_contact_block.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace PhysicsEngine {
namespace {
struct DVector {
    double x = 0, y = 0;
    DVector() = default;
    DVector(double x, double y) : x(x), y(y) {}
    DVector(Vector2 v) : x(v.x), y(v.y) {}
    DVector operator+(DVector v) const { return {x + v.x, y + v.y}; }
    DVector operator-(DVector v) const { return {x - v.x, y - v.y}; }
    DVector operator*(double s) const { return {x * s, y * s}; }
    double dot(DVector v) const { return x * v.x + y * v.y; }
    double cross(DVector v) const { return x * v.y - y * v.x; }
};
double Checked(double value) {
    if (!std::isfinite(value))
        throw std::overflow_error("Contact arithmetic exceeds finite double range");
    return value;
}
float CheckedFloat(double value) {
    if (!std::isfinite(value) || std::abs(value) > std::numeric_limits<float>::max())
        throw std::overflow_error("Contact result exceeds finite float state range");
    return static_cast<float>(value);
}
Vector2 CheckedVector(DVector v) {
    return {CheckedFloat(v.x), CheckedFloat(v.y)};
}
void ValidateFinite(double value) {
    if (!std::isfinite(value))
        throw std::invalid_argument("Contact input must be finite");
}
void ValidateFinite(Vector2 v) {
    ValidateFinite(v.x);
    ValidateFinite(v.y);
}
void ValidateImpulse(const ContactImpulse &j) {
    ValidateFinite(j.normal);
    ValidateFinite(j.tangent);
}
void ValidateBody(const RigidBody &body) {
    ValidateFinite(body.position);
    ValidateFinite(body.orientation);
    ValidateFinite(body.velocity);
    ValidateFinite(body.angularVelocity);
    ValidateFinite(body.inverseMass);
    ValidateFinite(body.inverseInertia);
    if (body.IsStatic() ? (body.inverseMass != 0 || body.inverseInertia != 0)
                        : (body.inverseMass <= 0 || body.inverseInertia <= 0))
        throw std::invalid_argument("Contact inverse mass/inertia are invalid for the body type");
    const auto &m = body.material;
    ValidateFinite(m.restitution);
    ValidateFinite(m.staticFriction);
    ValidateFinite(m.dynamicFriction);
    if (m.restitution < 0 || m.restitution > 1 || m.staticFriction < 0 || m.dynamicFriction < 0)
        throw std::invalid_argument("Contact restitution/friction are invalid");
}
Vector2 ContactTangent(Vector2 normal) {
    return {-normal.y, normal.x};
}
std::uint8_t EffectiveContactCount(const CollisionManifold &m) {
    return m.contactCount > 0 ? std::min<std::uint8_t>(m.contactCount, 2)
                              : std::uint8_t(m.hasCollision ? 1 : 0);
}
ContactPoint EffectiveContact(const CollisionManifold &m, std::uint8_t i) {
    return m.contactCount > 0 ? m.contacts[i] : ContactPoint{m.contactPoint, m.penetration, 0};
}
void ValidateManifold(const CollisionManifold &m) {
    ValidateBody(*m.A);
    ValidateBody(*m.B);
    if (m.A == m.B)
        throw std::invalid_argument("Contact requires distinct endpoints");
    ValidateFinite(m.normal);
    ValidateFinite(m.penetration);
    if (m.normal.x == 0 && m.normal.y == 0)
        throw std::invalid_argument("Contact normal must be nonzero");
    // Generated normals are unit vectors subject to float rounding. Preserve
    // that contract without introducing an arbitrary additional norm tolerance.
    for (std::uint8_t i = 0; i < EffectiveContactCount(m); ++i) {
        const auto p = EffectiveContact(m, i);
        ValidateFinite(p.position);
        ValidateFinite(p.penetration);
    }
}
DVector Rotate(DVector v, double angle) {
    const double c = std::cos(angle), s = std::sin(angle);
    return {c * v.x - s * v.y, s * v.x + c * v.y};
}
Vector2 ToLocalPoint(const RigidBody *body, Vector2 point) {
    return CheckedVector(
        Rotate(DVector(point) - DVector(body->position), -double(body->orientation)));
}
DVector ToWorldPoint(const RigidBody *body, Vector2 point) {
    return DVector(body->position) + Rotate(point, body->orientation);
}
DVector ConstraintPointPosition(const ContactConstraint &c, const ContactConstraintPoint &p) {
    return (ToWorldPoint(c.bodyA, p.localAnchorA) + ToWorldPoint(c.bodyB, p.localAnchorB)) * 0.5;
}
DVector PointVelocity(const RigidBody *body, DVector point) {
    const auto r = point - DVector(body->position);
    return DVector(body->velocity) +
           DVector{-double(body->angularVelocity) * r.y, double(body->angularVelocity) * r.x};
}
double EffectiveMass(const RigidBody *a, const RigidBody *b, DVector ra, DVector rb,
                     DVector direction) {
    const double ca = ra.cross(direction), cb = rb.cross(direction);
    const double inverse = Checked(double(a->inverseMass) + b->inverseMass +
                                   ca * ca * a->inverseInertia + cb * cb * b->inverseInertia);
    return inverse > 0 ? Checked(1 / inverse) : 0;
}
void SynchronizeCache(const CollisionManifold &m, ContactImpulseCache &cache) {
    if (cache.contactCount > 2)
        throw std::invalid_argument("Contact cache count exceeds its capacity");
    for (std::uint8_t i = 0; i < cache.contactCount; ++i)
        ValidateImpulse(cache.contacts[i].impulse);
    const auto previous = cache;
    bool matched[2]{};
    cache = ContactImpulseCache{};
    cache.contactCount = EffectiveContactCount(m);
    for (std::uint8_t i = 0; i < cache.contactCount; ++i) {
        const auto p = EffectiveContact(m, i);
        cache.contacts[i].featureId = p.featureId;
        for (std::uint8_t old = 0; old < previous.contactCount; ++old)
            if (!matched[old] && previous.contacts[old].featureId == p.featureId) {
                cache.contacts[i].impulse = previous.contacts[old].impulse;
                matched[old] = true;
                break;
            }
    }
}
struct Proposal {
    Vector2 linear;
    float angular;
};
Proposal Propose(const RigidBody *body, DVector lever, DVector impulse, double sign,
                 bool position) {
    const auto linear = position ? body->position : body->velocity;
    const float angular = position ? body->orientation : body->angularVelocity;
    if (body->IsStatic())
        return {linear, angular};
    return {CheckedVector(DVector(linear) + impulse * (sign * body->inverseMass)),
            CheckedFloat(double(angular) + sign * body->inverseInertia * lever.cross(impulse))};
}
void ApplyPair(RigidBody *a, RigidBody *b, DVector ra, DVector rb, DVector impulse, bool position) {
    ValidateBody(*a);
    ValidateBody(*b);
    Checked(impulse.x);
    Checked(impulse.y);
    // Stage all six components before publishing either endpoint. These internal
    // corrections do not issue external wake requests or alter pending loads.
    const auto pa = Propose(a, ra, impulse, -1, position),
               pb = Propose(b, rb, impulse, 1, position);
    if (!a->IsStatic()) {
        if (position) {
            a->position = pa.linear;
            a->orientation = pa.angular;
        } else {
            a->velocity = pa.linear;
            a->angularVelocity = pa.angular;
        }
    }
    if (!b->IsStatic()) {
        if (position) {
            b->position = pb.linear;
            b->orientation = pb.angular;
        } else {
            b->velocity = pb.linear;
            b->angularVelocity = pb.angular;
        }
    }
}
void ApplyVelocityImpulse(RigidBody *a, RigidBody *b, DVector point, DVector impulse) {
    ApplyPair(a, b, point - DVector(a->position), point - DVector(b->position), impulse, false);
}
void ApplyCombinedVelocity(RigidBody *a, RigidBody *b, DVector impulse,
                           double torqueA, double torqueB) {
    ValidateBody(*a); ValidateBody(*b);
    Checked(impulse.x); Checked(impulse.y); Checked(torqueA); Checked(torqueB);
    // Both torque sums are complete before any float state conversion. No
    // intermediate contact correction or cache is visible on failure.
    const Proposal pa = a->IsStatic() ? Proposal{a->velocity,a->angularVelocity}
        : Proposal{CheckedVector(DVector(a->velocity)-impulse*a->inverseMass),
                   CheckedFloat(double(a->angularVelocity)-torqueA*a->inverseInertia)};
    const Proposal pb = b->IsStatic() ? Proposal{b->velocity,b->angularVelocity}
        : Proposal{CheckedVector(DVector(b->velocity)+impulse*b->inverseMass),
                   CheckedFloat(double(b->angularVelocity)+torqueB*b->inverseInertia)};
    if(!a->IsStatic()) { a->velocity=pa.linear; a->angularVelocity=pa.angular; }
    if(!b->IsStatic()) { b->velocity=pb.linear; b->angularVelocity=pb.angular; }
}
void PublishImpulse(ContactConstraint &c, ContactConstraintPoint &p, std::uint8_t index,
                    ContactImpulse impulse) {
    p.impulse = impulse;
    if (c.cache)
        c.cache->contacts[index].impulse = impulse;
}
} // namespace

void ContactSolverDetail::SynchronizeFeatureCache(const CollisionManifold& manifold,
                                                 ContactImpulseCache& cache) {
    SynchronizeCache(manifold,cache);
}

bool ContactSolverDetail::SolveNormalBlock(ContactConstraint &c) {
    if(c.pointCount!=2) return false;
    ValidateBody(*c.bodyA); ValidateBody(*c.bodyB);
    const DVector n=c.normal;
    DVector points[2]; double ca[2],cb[2],vn[2],old[2],b[2];
    for(int i=0;i<2;++i) {
        ValidateImpulse(c.points[i].impulse);
        points[i]=ConstraintPointPosition(c,c.points[i]);
        ca[i]=(points[i]-DVector(c.bodyA->position)).cross(n);
        cb[i]=(points[i]-DVector(c.bodyB->position)).cross(n);
        vn[i]=Checked((PointVelocity(c.bodyB,points[i])-PointVelocity(c.bodyA,points[i])).dot(n));
        old[i]=c.points[i].impulse.normal;
    }
    const double m=double(c.bodyA->inverseMass)+c.bodyB->inverseMass,
        ia=c.bodyA->inverseInertia,ib=c.bodyB->inverseInertia;
    const double k11=Checked(m+ia*ca[0]*ca[0]+ib*cb[0]*cb[0]),
        k22=Checked(m+ia*ca[1]*ca[1]+ib*cb[1]*cb[1]),
        k12=Checked(m+ia*ca[0]*ca[1]+ib*cb[0]*cb[1]);
    const double scale=std::max({k11,k22,std::abs(k12)});
    if(scale==0) return false;
    b[0]=Checked(vn[0]-c.points[0].velocityBias-Checked(k11*old[0]+k12*old[1]));
    b[1]=Checked(vn[1]-c.points[1].velocityBias-Checked(k12*old[0]+k22*old[1]));
    const double deltaA=ca[0]-ca[1],deltaB=cb[0]-cb[1],cross=ca[0]*cb[1]-ca[1]*cb[0];
    // Positive terms avoid catastrophic cancellation in k11*k22-k12*k12.
    const double determinant=Checked(m*ia*deltaA*deltaA+m*ib*deltaB*deltaB+ia*ib*cross*cross);
    const double a11=k11/scale,a22=k22/scale,a12=k12/scale,
        det=determinant/scale/scale;
    constexpr double eps=128*std::numeric_limits<double>::epsilon();
    double accepted[2]{};
    auto admissible=[&](double x1,double x2,bool active1,bool active2) {
        Checked(x1); Checked(x2);
        const double impulseScale=std::max({std::abs(x1),std::abs(x2),std::abs(b[0]/k11),std::abs(b[1]/k22)});
        const double xtol=eps*impulseScale;
        if(x1 < -xtol || x2 < -xtol) return false;
        x1=std::max(x1,0.0); x2=std::max(x2,0.0);
        const double w1=Checked(k11*x1+k12*x2+b[0]),w2=Checked(k12*x1+k22*x2+b[1]);
        const double t1=eps*Checked(std::abs(k11*x1)+std::abs(k12*x2)+std::abs(b[0])),
            t2=eps*Checked(std::abs(k12*x1)+std::abs(k22*x2)+std::abs(b[1]));
        if(w1 < -t1 || w2 < -t2 || (active1&&std::abs(w1)>t1) || (active2&&std::abs(w2)>t2)) return false;
        accepted[0]=x1; accepted[1]=x2; return true;
    };
    bool solved=false;
    if(det>eps*(a11+a22)*(a11+a22))
        solved=admissible((-a22*(b[0]/scale)+a12*(b[1]/scale))/det,
            (a12*(b[0]/scale)-a11*(b[1]/scale))/det,true,true);
    if(!solved) solved=admissible(-b[0]/k11,0,true,false);
    if(!solved) solved=admissible(0,-b[1]/k22,false,true);
    if(!solved) solved=admissible(0,0,false,false);
    if(!solved) {
        // Bounded projected scalar sweep for an ill-conditioned/inadmissible
        // block. It is an approximation, staged just like the exact active sets.
        accepted[0]=std::max(Checked(old[0]+(c.points[0].velocityBias-vn[0])/k11),0.0);
        accepted[1]=std::max(Checked(old[1]+(c.points[1].velocityBias-vn[1]-k12*(accepted[0]-old[0]))/k22),0.0);
    }
    const double d1=Checked(accepted[0]-old[0]),d2=Checked(accepted[1]-old[1]);
    ApplyCombinedVelocity(c.bodyA,c.bodyB,n*Checked(d1+d2),
        Checked(ca[0]*d1+ca[1]*d2),Checked(cb[0]*d1+cb[1]*d2));
    for(std::uint8_t i=0;i<2;++i)
        PublishImpulse(c,c.points[i],i,{accepted[i],c.points[i].impulse.tangent});
    return true;
}

ContactConstraint CollisionResolver::PrepareConstraint(const CollisionManifold &m,
                                                       ContactImpulseCache &cache,
                                                       const SimulationConfig &config) {
    ContactConstraint c;
    if (!m.hasCollision || !m.A || !m.B) {
        cache = ContactImpulseCache{};
        return c;
    }
    ValidateManifold(m);
    config.Validate();
    SynchronizeCache(m, cache);
    c.bodyA = m.A;
    c.bodyB = m.B;
    c.normal = m.normal;
    c.tangent = ContactTangent(m.normal);
    c.pointCount = cache.contactCount;
    c.staticFriction = CheckedFloat(
        std::sqrt(double(m.A->material.staticFriction) * m.B->material.staticFriction));
    c.dynamicFriction = CheckedFloat(
        std::sqrt(double(m.A->material.dynamicFriction) * m.B->material.dynamicFriction));
    c.positionCorrectionFactor = config.positionCorrectionFactor;
    c.penetrationSlop = config.penetrationSlop;
    c.maxPositionCorrection = config.maxPositionCorrection;
    c.velocityTolerance = config.velocityTolerance;
    c.cache = &cache;
    for (std::uint8_t i = 0; i < c.pointCount; ++i) {
        const auto contact = EffectiveContact(m, i);
        auto &p = c.points[i];
        p.localAnchorA = ToLocalPoint(m.A, contact.position);
        p.localAnchorB = ToLocalPoint(m.B, contact.position);
        p.penetration = contact.penetration > 0 ? contact.penetration : m.penetration;
        p.featureId = contact.featureId;
        p.impulse = cache.contacts[i].impulse;
        const auto ra = DVector(contact.position) - DVector(m.A->position),
                   rb = DVector(contact.position) - DVector(m.B->position);
        p.normalMass = EffectiveMass(m.A, m.B, ra, rb, c.normal);
        p.tangentMass = EffectiveMass(m.A, m.B, ra, rb, c.tangent);
        const double normalVelocity =
            Checked((PointVelocity(m.B, contact.position) - PointVelocity(m.A, contact.position))
                        .dot(c.normal));
        if (normalVelocity < -config.restitutionVelocityThreshold)
            p.velocityBias =
                Checked(-double(std::max(m.A->material.restitution, m.B->material.restitution)) *
                        normalVelocity);
    }
    return c;
}
void CollisionResolver::WarmStart(ContactConstraint &c) {
    if (!c.bodyA || !c.bodyB)
        return;
    if(c.pointCount==2) {
        DVector sum; double ta=0,tb=0;
        for(std::uint8_t i=0;i<2;++i) {
            const auto& p=c.points[i]; ValidateImpulse(p.impulse);
            const auto point=ConstraintPointPosition(c,p);
            const auto impulse=DVector(c.normal)*p.impulse.normal+DVector(c.tangent)*p.impulse.tangent;
            sum=sum+impulse;
            ta+= (point-DVector(c.bodyA->position)).cross(impulse);
            tb+= (point-DVector(c.bodyB->position)).cross(impulse);
        }
        ApplyCombinedVelocity(c.bodyA,c.bodyB,sum,ta,tb); return;
    }
    for (std::uint8_t i = 0; i < c.pointCount; ++i) {
        const auto &p = c.points[i];
        ValidateImpulse(p.impulse);
        ApplyVelocityImpulse(c.bodyA, c.bodyB, ConstraintPointPosition(c, p),
                             DVector(c.normal) * p.impulse.normal +
                                 DVector(c.tangent) * p.impulse.tangent);
    }
}
void CollisionResolver::SolveVelocity(ContactConstraint &c) {
    if (!c.bodyA || !c.bodyB)
        return;
    const bool block=ContactSolverDetail::SolveNormalBlock(c);
    for (std::uint8_t i = 0; i < c.pointCount; ++i) {
        auto &p = c.points[i];
        const auto worldPoint = ConstraintPointPosition(c, p);
        auto relative = PointVelocity(c.bodyB, worldPoint) - PointVelocity(c.bodyA, worldPoint);
        if (!block) {
            const double normalVelocity = Checked(relative.dot(c.normal));
            const double normal = std::max(
                Checked(p.impulse.normal + p.normalMass * (-normalVelocity + p.velocityBias)), 0.0);
            ApplyVelocityImpulse(c.bodyA, c.bodyB, worldPoint,
                                 DVector(c.normal) * (normal - p.impulse.normal));
            PublishImpulse(c, p, i, {normal, p.impulse.tangent});
        }
        relative = PointVelocity(c.bodyB, worldPoint) - PointVelocity(c.bodyA, worldPoint);
        const double tangentVelocity = Checked(relative.dot(c.tangent));
        bool solveTangent = std::abs(tangentVelocity) > c.velocityTolerance;
        if (block)
            solveTangent = solveTangent || std::abs(p.impulse.tangent) >
                Checked(p.impulse.normal * c.staticFriction);
        if (solveTangent) {
            const double candidate = Checked(p.impulse.tangent - p.tangentMass * tangentVelocity);
            const double maximumStatic = Checked(p.impulse.normal * c.staticFriction);
            double tangent = candidate;
            if (std::abs(candidate) > maximumStatic) {
                const double maximumDynamic = Checked(p.impulse.normal * c.dynamicFriction);
                tangent = std::clamp(candidate, -maximumDynamic, maximumDynamic);
            }
            ApplyVelocityImpulse(c.bodyA, c.bodyB, worldPoint,
                                 DVector(c.tangent) * (tangent - p.impulse.tangent));
            PublishImpulse(c, p, i, {p.impulse.normal, tangent});
        }
    }
}
bool CollisionResolver::SolvePosition(ContactConstraint &c) {
    if (!c.bodyA || !c.bodyB)
        return true;
    double minimumSeparation = 0;
    for (std::uint8_t i = 0; i < c.pointCount; ++i) {
        const auto &p = c.points[i];
        const auto anchorA = ToWorldPoint(c.bodyA, p.localAnchorA),
                   anchorB = ToWorldPoint(c.bodyB, p.localAnchorB);
        const auto ra = anchorA - DVector(c.bodyA->position),
                   rb = anchorB - DVector(c.bodyB->position);
        const double separation = Checked((anchorB - anchorA).dot(c.normal) - p.penetration);
        minimumSeparation = std::min(minimumSeparation, separation);
        const double correction =
            std::clamp(double(c.positionCorrectionFactor) * (separation + c.penetrationSlop),
                       -double(c.maxPositionCorrection), 0.0);
        const double mass = EffectiveMass(c.bodyA, c.bodyB, ra, rb, c.normal);
        ApplyPair(c.bodyA, c.bodyB, ra, rb, DVector(c.normal) * (-mass * correction), true);
    }
    return minimumSeparation >= -3.0 * c.penetrationSlop;
}
void CollisionResolver::WarmStart(const CollisionManifold &m, const ContactImpulse &j) {
    if (!m.hasCollision || !m.A || !m.B)
        return;
    ValidateManifold(m);
    ValidateImpulse(j);
    ApplyVelocityImpulse(m.A, m.B, EffectiveContact(m, 0).position,
                         DVector(m.normal) * j.normal +
                             DVector(ContactTangent(m.normal)) * j.tangent);
}
void CollisionResolver::WarmStart(const CollisionManifold &m, ContactImpulseCache &cache) {
    auto c = PrepareConstraint(m, cache, SimulationConfig{});
    WarmStart(c);
}
void CollisionResolver::Resolve(const CollisionManifold &m) {
    ContactImpulseCache cache;
    Resolve(m, cache, SimulationConfig{});
}
void CollisionResolver::Resolve(const CollisionManifold &m, ContactImpulse &impulse,
                                bool reduceWarmStart) {
    ContactImpulseCache cache;
    cache.contactCount = 1;
    cache.contacts[0] = {EffectiveContact(m, 0).featureId, impulse};
    Resolve(m, cache, SimulationConfig{}, reduceWarmStart);
    impulse = cache.contactCount > 0 ? cache.contacts[0].impulse : ContactImpulse{};
}
void CollisionResolver::Resolve(const CollisionManifold &m, ContactImpulseCache &cache,
                                const SimulationConfig &config, bool reduceWarmStart) {
    (void)reduceWarmStart;
    auto c = PrepareConstraint(m, cache, config);
    SolveVelocity(c);
    SolvePosition(c);
}
} // namespace PhysicsEngine

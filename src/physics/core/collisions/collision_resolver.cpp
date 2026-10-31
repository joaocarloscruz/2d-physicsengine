#include "physics/core/collisions/collision_resolver.h"
#include "physics/core/rigidbody.h"
#include "normal_contact_block.h"
#include "../trig_cache.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace PhysicsEngine {
namespace {
// Out-of-line throws keep the hot checked arithmetic small.
#if defined(__GNUC__) || defined(__clang__)
#define PHYSICS_COLD_NOINLINE __attribute__((noinline, cold))
#else
#define PHYSICS_COLD_NOINLINE
#endif
[[noreturn]] PHYSICS_COLD_NOINLINE void ThrowOverflow(const char* message) { throw std::overflow_error(message); }
[[noreturn]] PHYSICS_COLD_NOINLINE void ThrowInvalid(const char* message) { throw std::invalid_argument(message); }
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
// abs(v) <= max is false exactly for NaN and infinities (and, for floats,
// for out-of-range values): one compare instead of isfinite plus a range test.
double Checked(double value) {
    if (!(std::abs(value) <= std::numeric_limits<double>::max()))
        ThrowOverflow("Contact arithmetic exceeds finite double range");
    return value;
}
float CheckedFloat(double value) {
    if (!(std::abs(value) <= std::numeric_limits<float>::max()))
        ThrowOverflow("Contact result exceeds finite float state range");
    return static_cast<float>(value);
}
Vector2 CheckedVector(DVector v) {
    return {CheckedFloat(v.x), CheckedFloat(v.y)};
}
void ValidateFinite(double value) {
    if (!(std::abs(value) <= std::numeric_limits<double>::max()))
        ThrowInvalid("Contact input must be finite");
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
        ThrowInvalid("Contact inverse mass/inertia are invalid for the body type");
    const auto &m = body.material;
    ValidateFinite(m.restitution);
    ValidateFinite(m.staticFriction);
    ValidateFinite(m.dynamicFriction);
    if (m.restitution < 0 || m.restitution > 1 || m.staticFriction < 0 || m.dynamicFriction < 0)
        ThrowInvalid("Contact restitution/friction are invalid");
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
        ThrowInvalid("Contact requires distinct endpoints");
    ValidateFinite(m.normal);
    ValidateFinite(m.penetration);
    if (m.normal.x == 0 && m.normal.y == 0)
        ThrowInvalid("Contact normal must be nonzero");
    // Generated normals are unit vectors subject to float rounding. Preserve
    // that contract without introducing an arbitrary additional norm tolerance.
    for (std::uint8_t i = 0; i < EffectiveContactCount(m); ++i) {
        const auto p = EffectiveContact(m, i);
        ValidateFinite(p.position);
        ValidateFinite(p.penetration);
    }
}
DVector Rotate(DVector v, double angle) {
    const Detail::CosSin rotation = Detail::CachedCosSin(angle);
    const double c = rotation.cosine, s = rotation.sine;
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
// ConstraintPointPosition for each point, reusing the constraint's cached
// positions while the bodies, their poses and the anchors are bit-for-bit
// those they were computed from.
void PointPositions(ContactConstraint &c, DVector (&points)[2]) {
    const std::uint8_t count = std::min<std::uint8_t>(c.pointCount, 2);
    auto &cache = c.pointPositions;
    const float poses[6]{c.bodyA->position.x, c.bodyA->position.y, c.bodyA->orientation,
                         c.bodyB->position.x, c.bodyB->position.y, c.bodyB->orientation};
    float anchors[8]{};
    for (std::uint8_t i = 0; i < count; ++i) {
        anchors[4 * i] = c.points[i].localAnchorA.x; anchors[4 * i + 1] = c.points[i].localAnchorA.y;
        anchors[4 * i + 2] = c.points[i].localAnchorB.x; anchors[4 * i + 3] = c.points[i].localAnchorB.y;
    }
    if (cache.valid && cache.bodyA == c.bodyA && cache.bodyB == c.bodyB && cache.count == count
        && std::memcmp(cache.poses, poses, sizeof poses) == 0
        && std::memcmp(cache.anchors, anchors, sizeof anchors) == 0) {
        for (std::uint8_t i = 0; i < count; ++i)
            points[i] = {cache.positions[i][0], cache.positions[i][1]};
        return;
    }
    for (std::uint8_t i = 0; i < count; ++i) {
        points[i] = ConstraintPointPosition(c, c.points[i]);
        cache.positions[i][0] = points[i].x;
        cache.positions[i][1] = points[i].y;
    }
    cache.bodyA = c.bodyA;
    cache.bodyB = c.bodyB;
    std::memcpy(cache.poses, poses, sizeof poses);
    std::memcpy(cache.anchors, anchors, sizeof anchors);
    cache.count = count;
    cache.valid = true;
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
        ThrowInvalid("Contact cache count exceeds its capacity");
    for (std::uint8_t i = 0; i < cache.contactCount; ++i)
        ValidateImpulse(cache.contacts[i].impulse);
    const auto previous = cache;
    bool matched[2]{};
    cache = ContactImpulseCache{};
    cache.impulseSubsteps = previous.impulseSubsteps;
    cache.rigidSolve = previous.rigidSolve;
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
// validate = false only where both bodies were validated earlier in the same
// solver call and changed since then only through checked (finite) updates.
void ApplyPair(RigidBody *a, RigidBody *b, DVector ra, DVector rb, DVector impulse, bool position,
               bool validate = true) {
    if (validate) {
        ValidateBody(*a);
        ValidateBody(*b);
    }
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
void ApplyVelocityImpulse(RigidBody *a, RigidBody *b, DVector point, DVector impulse,
                          bool validate = true) {
    ApplyPair(a, b, point - DVector(a->position), point - DVector(b->position), impulse, false, validate);
}
void ApplyCombinedVelocity(RigidBody *a, RigidBody *b, DVector impulse,
                           double torqueA, double torqueB, bool validate = true) {
    if (validate) { ValidateBody(*a); ValidateBody(*b); }
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

namespace {
// points: the two constraint point positions. They depend only on the
// bodies' poses, which stay fixed through a velocity solve.
bool SolveNormalBlock(ContactConstraint &c, const DVector (&points)[2]) {
    if(c.pointCount!=2) return false;
    ValidateBody(*c.bodyA); ValidateBody(*c.bodyB);
    const DVector n=c.normal;
    double ca[2],cb[2],vn[2],old[2],b[2];
    for(int i=0;i<2;++i) {
        ValidateImpulse(c.points[i].impulse);
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
    // Both bodies were validated above and have not changed since.
    ApplyCombinedVelocity(c.bodyA,c.bodyB,n*Checked(d1+d2),
        Checked(ca[0]*d1+ca[1]*d2),Checked(cb[0]*d1+cb[1]*d2),false);
    for(std::uint8_t i=0;i<2;++i)
        PublishImpulse(c,c.points[i],i,{accepted[i],c.points[i].impulse.tangent});
    return true;
}
} // namespace

bool ContactSolverDetail::SolveNormalBlock(ContactConstraint &c) {
    if(c.pointCount!=2) return false;
    const DVector points[2]{ConstraintPointPosition(c,c.points[0]),ConstraintPointPosition(c,c.points[1])};
    return PhysicsEngine::SolveNormalBlock(c,points);
}

ContactConstraint CollisionResolver::PrepareConstraint(const CollisionManifold &m,
                                                       ContactImpulseCache &cache,
                                                       const SimulationConfig &config,
                                                       bool configValidated) {
    ContactConstraint c;
    if (!m.hasCollision || !m.A || !m.B) {
        cache = ContactImpulseCache{};
        return c;
    }
    ValidateManifold(m);
    if (!configValidated) config.Validate();
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
        // Negative point penetration is a speculative gap and is kept as is.
        p.penetration = contact.penetration != 0 ? contact.penetration : std::max(m.penetration, 0.0f);
        p.featureId = contact.featureId;
        p.impulse = cache.contacts[i].impulse;
        const auto ra = DVector(contact.position) - DVector(m.A->position),
                   rb = DVector(contact.position) - DVector(m.B->position);
        p.leverA[0] = ra.x; p.leverA[1] = ra.y;
        p.leverB[0] = rb.x; p.leverB[1] = rb.y;
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
    // Poses do not change during the velocity solve, so each point position
    // is computed once here and shared with the normal block.
    DVector points[2];
    PointPositions(c, points);
    const bool block = SolveNormalBlock(c, points);
    // After the first validated impulse in this call the bodies change only
    // through checked velocity updates, so later impulses skip revalidation.
    bool validated = block;
    for (std::uint8_t i = 0; i < c.pointCount; ++i) {
        auto &p = c.points[i];
        const auto worldPoint = points[i];
        auto relative = PointVelocity(c.bodyB, worldPoint) - PointVelocity(c.bodyA, worldPoint);
        if (!block) {
            const double normalVelocity = Checked(relative.dot(c.normal));
            const double normal = std::max(
                Checked(p.impulse.normal + p.normalMass * (-normalVelocity + p.velocityBias)), 0.0);
            ApplyVelocityImpulse(c.bodyA, c.bodyB, worldPoint,
                                 DVector(c.normal) * (normal - p.impulse.normal), !validated);
            validated = true;
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
                                 DVector(c.tangent) * (tangent - p.impulse.tangent), !validated);
            validated = true;
            PublishImpulse(c, p, i, {p.impulse.normal, tangent});
        }
    }
}
namespace {
// Corrects both points of a two-point manifold at once, as the 2x2 LCP
// lambda >= 0, w = K lambda + C >= 0, lambda.w = 0, where C_i is the
// correction target (positive while the point is within slop). Correcting the
// points one after the other rotates the body toward the point solved last;
// the early exit usually stops after one pass, so every step leaves the same
// small tilt and a tall stack amplifies it under gravity until it topples.
// Returns false (nothing applied) only for a degenerate block.
bool SolvePositionBlock(ContactConstraint &c, double &minimumSeparation) {
    ValidateBody(*c.bodyA);
    ValidateBody(*c.bodyB);
    const DVector n = c.normal;
    double ca[2], cb[2], correction[2];
    for (std::uint8_t i = 0; i < 2; ++i) {
        const auto &p = c.points[i];
        const auto anchorA = ToWorldPoint(c.bodyA, p.localAnchorA),
                   anchorB = ToWorldPoint(c.bodyB, p.localAnchorB);
        const double separation = Checked((anchorB - anchorA).dot(n) - p.penetration);
        minimumSeparation = std::min(minimumSeparation, separation);
        correction[i] = std::max(double(c.positionCorrectionFactor) * (separation + c.penetrationSlop),
                                 -double(c.maxPositionCorrection));
        ca[i] = (anchorA - DVector(c.bodyA->position)).cross(n);
        cb[i] = (anchorB - DVector(c.bodyB->position)).cross(n);
    }
    if (correction[0] >= 0 && correction[1] >= 0) return true;
    const double m = double(c.bodyA->inverseMass) + c.bodyB->inverseMass,
                 ia = c.bodyA->inverseInertia, ib = c.bodyB->inverseInertia;
    const double k11 = Checked(m + ia * ca[0] * ca[0] + ib * cb[0] * cb[0]),
                 k22 = Checked(m + ia * ca[1] * ca[1] + ib * cb[1] * cb[1]),
                 k12 = Checked(m + ia * ca[0] * ca[1] + ib * cb[0] * cb[1]);
    const double deltaA = ca[0] - ca[1], deltaB = cb[0] - cb[1], cross = ca[0] * cb[1] - ca[1] * cb[0];
    const double determinant = Checked(m * ia * deltaA * deltaA + m * ib * deltaB * deltaB + ia * ib * cross * cross);
    if (!(determinant > 1e-9 * k11 * k22)) return false;
    double l1 = Checked((-k22 * correction[0] + k12 * correction[1]) / determinant);
    double l2 = Checked((k12 * correction[0] - k11 * correction[1]) / determinant);
    if (l1 < 0 || l2 < 0) {
        l1 = std::max(-correction[0] / k11, 0.0); l2 = 0;
        if (k12 * l1 + correction[1] < 0) { l1 = 0; l2 = std::max(-correction[1] / k22, 0.0); }
    }
    const DVector impulse = n * (l1 + l2);
    const double turnA = ca[0] * l1 + ca[1] * l2, turnB = cb[0] * l1 + cb[1] * l2;
    // Stage both endpoints before publishing either.
    const Vector2 positionA = c.bodyA->IsStatic() ? c.bodyA->position
        : CheckedVector(DVector(c.bodyA->position) - impulse * c.bodyA->inverseMass);
    const Vector2 positionB = c.bodyB->IsStatic() ? c.bodyB->position
        : CheckedVector(DVector(c.bodyB->position) + impulse * c.bodyB->inverseMass);
    const float orientationA = c.bodyA->IsStatic() ? c.bodyA->orientation
        : CheckedFloat(double(c.bodyA->orientation) - ia * turnA);
    const float orientationB = c.bodyB->IsStatic() ? c.bodyB->orientation
        : CheckedFloat(double(c.bodyB->orientation) + ib * turnB);
    if (!c.bodyA->IsStatic()) { c.bodyA->position = positionA; c.bodyA->orientation = orientationA; }
    if (!c.bodyB->IsStatic()) { c.bodyB->position = positionB; c.bodyB->orientation = orientationB; }
    return true;
}
} // namespace
bool CollisionResolver::SolvePosition(ContactConstraint &c) {
    if (!c.bodyA || !c.bodyB)
        return true;
    double minimumSeparation = 0;
    if (c.pointCount == 2 && SolvePositionBlock(c, minimumSeparation))
        return minimumSeparation >= -3.0 * c.penetrationSlop;
    minimumSeparation = 0;
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
CollisionResolver::Softness CollisionResolver::MakeSoftness(double hertz, double dampingRatio,
                                                            double substep) {
    if (!(hertz > 0) || !(substep > 0))
        return {};
    // Box2D v3's soft constraint: a spring of this frequency and damping
    // ratio, integrated implicitly over the substep.
    const double omega = 2 * 3.14159265358979323846 * hertz;
    const double a1 = 2 * dampingRatio + substep * omega;
    const double a2 = substep * omega * a1;
    const double a3 = 1 / (1 + a2);
    return {omega / a1, a2 * a3, a3};
}
namespace {
DVector LeverA(const ContactConstraintPoint &p) { return {p.leverA[0], p.leverA[1]}; }
DVector LeverB(const ContactConstraintPoint &p) { return {p.leverB[0], p.leverB[1]}; }
DVector LeverVelocity(const RigidBody *body, DVector r) {
    return DVector(body->velocity) +
           DVector{-double(body->angularVelocity) * r.y, double(body->angularVelocity) * r.x};
}
} // namespace
double CollisionResolver::SagSizeLimit(const RigidBody &body) {
    return 0.01 * std::sqrt(std::max(double(body.shape->GetArea()), 0.0));
}
bool CollisionResolver::UpdateRigidSolve(ContactConstraint &c, const Softness &soft,
                                         int substeps) {
    if (!c.cache || !(soft.biasRate > 0) || substeps <= 1)
        return c.cache && c.cache->rigidSolve;
    return UpdateRigidSolve(c, soft, substeps, c.bodyA->IsStatic() ? 0.0 : SagSizeLimit(*c.bodyA),
                            c.bodyB->IsStatic() ? 0.0 : SagSizeLimit(*c.bodyB));
}
bool CollisionResolver::UpdateRigidSolve(ContactConstraint &c, const Softness &soft,
                                         int substeps, double sizeLimitA, double sizeLimitB) {
    if (!c.cache)
        return false;
    bool &rigid = c.cache->rigidSolve;
    if (!(soft.biasRate > 0) || substeps <= 1)
        return rigid;
    // A soft row carrying impulse lambda per substep settles where its bias
    // balances the relaxed impulse: overlap lambda * impulseScale /
    // (normalMass * massScale * biasRate).
    const double perSubstep = double(c.cache->impulseSubsteps) / substeps;
    double sag = 0;
    for (std::uint8_t i = 0; i < c.pointCount; ++i)
        if (c.points[i].normalMass > 0)
            sag = std::max(sag, c.points[i].impulse.normal * perSubstep * soft.impulseScale /
                                    (c.points[i].normalMass * soft.massScale * soft.biasRate));
    // The sag follows the load, not the body size, and a stack of small
    // bodies buckles once it is a sizable part of their size: the bound is
    // also 1% of the smaller body's size (the square root of its area).
    double limit = c.penetrationSlop;
    if (!c.bodyA->IsStatic()) limit = std::min(limit, sizeLimitA);
    if (!c.bodyB->IsStatic()) limit = std::min(limit, sizeLimitB);
    if (!rigid && sag > 1.25 * limit)
        rigid = true;
    else if (rigid && sag < 0.75 * limit)
        rigid = false;
    return rigid;
}
void CollisionResolver::MatchImpulseSubsteps(ContactConstraint &c, int substeps) {
    if (!c.cache || substeps < 1 || c.cache->impulseSubsteps == substeps)
        return;
    const double scale = double(c.cache->impulseSubsteps) / substeps;
    for (std::uint8_t i = 0; i < c.pointCount; ++i) {
        auto &p = c.points[i];
        PublishImpulse(c, p, i, {Checked(p.impulse.normal * scale), Checked(p.impulse.tangent * scale)});
    }
    c.cache->impulseSubsteps = static_cast<std::uint8_t>(substeps);
}
void CollisionResolver::WarmStartSubstep(ContactConstraint &c) {
    if (!c.bodyA || !c.bodyB)
        return;
    // Only the first warm start of the step validates (SubstepCache::validated).
    const bool validate = !c.substep.validated;
    if (validate) {
        ValidateBody(*c.bodyA);
        ValidateBody(*c.bodyB);
    }
    if (c.pointCount == 2) {
        // One combined update, as in WarmStart: applying the two points one
        // after the other rounds differently and tilts a symmetric stack.
        DVector sum;
        double torqueA = 0, torqueB = 0;
        for (std::uint8_t i = 0; i < 2; ++i) {
            const auto &p = c.points[i];
            if (validate) ValidateImpulse(p.impulse);
            const auto impulse = DVector(c.normal) * p.impulse.normal + DVector(c.tangent) * p.impulse.tangent;
            sum = sum + impulse;
            torqueA += LeverA(p).cross(impulse);
            torqueB += LeverB(p).cross(impulse);
        }
        ApplyCombinedVelocity(c.bodyA, c.bodyB, sum, torqueA, torqueB, false);
        c.substep.validated = true;
        return;
    }
    for (std::uint8_t i = 0; i < c.pointCount; ++i) {
        const auto &p = c.points[i];
        if (validate) ValidateImpulse(p.impulse);
        if (p.impulse.normal == 0 && p.impulse.tangent == 0)
            continue;
        ApplyPair(c.bodyA, c.bodyB, LeverA(p), LeverB(p),
                  DVector(c.normal) * p.impulse.normal + DVector(c.tangent) * p.impulse.tangent,
                  false, false);
    }
    c.substep.validated = true;
}
void CollisionResolver::SolveSubstep(ContactConstraint &c, double inverseSubstep,
                                     const Softness &soft, double maxPushSpeed, bool useBias,
                                     bool gapRows) {
    if (!c.bodyA || !c.bodyB)
        return;
    RigidBody *a = c.bodyA, *b = c.bodyB;
    auto &cache = c.substep;
    // After the step's first warm start the bodies and impulses change only
    // through checked (finite) updates, so the checks below cannot fail.
    const bool validate = !cache.validated;
    if (validate) {
        ValidateBody(*a);
        ValidateBody(*b);
    }
    const DVector n = c.normal, t = c.tangent;
    // Separation at the current poses; the normal stays the prepared one.
    // Poses change only between velocity passes, so the separations are
    // reused while both poses are bit-for-bit those they came from.
    const float poses[6]{a->position.x, a->position.y, a->orientation,
                         b->position.x, b->position.y, b->orientation};
    if (validate || !cache.separationsValid || std::memcmp(cache.poses, poses, sizeof poses) != 0) {
        cache.separationsValid = false;
        for (std::uint8_t i = 0; i < c.pointCount; ++i) {
            const auto &p = c.points[i];
            if (validate) ValidateImpulse(p.impulse);
            cache.separations[i] = Checked(
                (ToWorldPoint(b, p.localAnchorB) - ToWorldPoint(a, p.localAnchorA)).dot(n) -
                p.penetration);
        }
        std::memcpy(cache.poses, poses, sizeof poses);
        cache.separationsValid = true;
    }
    double biases[2]{}, massScales[2]{1, 1}, impulseScales[2]{}, normalVelocities[2]{};
    bool skip[2]{}, softRows[2]{};
    for (std::uint8_t i = 0; i < c.pointCount; ++i) {
        const double separation = cache.separations[i];
        if (separation > 0) {
            skip[i] = !gapRows;
            biases[i] = separation * inverseSubstep; // the gap may close this substep, no further
        } else if (useBias && separation < 0) {
            // Any overlap, not only overlap beyond the slop: a dead band would
            // leave the relative rotation of stacked boxes free.
            biases[i] = std::max(soft.biasRate * separation, -maxPushSpeed);
            massScales[i] = soft.massScale;
            impulseScales[i] = soft.impulseScale;
            softRows[i] = true;
        }
    }
    if (c.pointCount == 2 && !skip[0] && !skip[1]) {
        // Both normal rows together: the 2x2 LCP x >= 0, w = (K + G) x + q >= 0,
        // x.w = 0, where G holds each soft row's compliance K_ii * impulseScale /
        // massScale and q = v_n + bias - K * accumulated.
        double old[2], q[2], gamma[2];
        for (int i = 0; i < 2; ++i) {
            const auto &p = c.points[i];
            old[i] = p.impulse.normal;
            normalVelocities[i] = Checked((LeverVelocity(b, LeverB(p)) - LeverVelocity(a, LeverA(p))).dot(n));
        }
        // K depends only on the fixed levers and inverse masses.
        if (!cache.blockReady) {
            for (int i = 0; i < 2; ++i) {
                cache.ca[i] = LeverA(c.points[i]).cross(n);
                cache.cb[i] = LeverB(c.points[i]).cross(n);
            }
            const double *ca = cache.ca, *cb = cache.cb;
            const double m = double(a->inverseMass) + b->inverseMass, ia = a->inverseInertia, ib = b->inverseInertia;
            cache.k11 = Checked(m + ia * ca[0] * ca[0] + ib * cb[0] * cb[0]);
            cache.k22 = Checked(m + ia * ca[1] * ca[1] + ib * cb[1] * cb[1]);
            cache.k12 = Checked(m + ia * ca[0] * ca[1] + ib * cb[0] * cb[1]);
            const double deltaA = ca[0] - ca[1], deltaB = cb[0] - cb[1], cross = ca[0] * cb[1] - ca[1] * cb[0];
            cache.rigidDeterminant = m * ia * deltaA * deltaA + m * ib * deltaB * deltaB + ia * ib * cross * cross;
            cache.blockReady = true;
        }
        const double *ca = cache.ca, *cb = cache.cb;
        const double k11 = cache.k11, k22 = cache.k22, k12 = cache.k12;
        // The diagonal and determinant of K + G for each choice of soft rows,
        // for this softness (constant through a step).
        if (!cache.softReady || std::memcmp(cache.softness, &soft.massScale, sizeof(double)) != 0
            || std::memcmp(cache.softness + 1, &soft.impulseScale, sizeof(double)) != 0) {
            for (int combo = 0; combo < 4; ++combo) {
                const bool soft0 = combo & 1, soft1 = combo & 2;
                gamma[0] = k11 * (soft0 ? soft.impulseScale : 0.0) / (soft0 ? soft.massScale : 1.0);
                gamma[1] = k22 * (soft1 ? soft.impulseScale : 0.0) / (soft1 ? soft.massScale : 1.0);
                cache.diagonal[combo][0] = k11 + gamma[0];
                cache.diagonal[combo][1] = k22 + gamma[1];
                cache.determinants[combo] = cache.rigidDeterminant
                    + gamma[0] * k22 + gamma[1] * k11 + gamma[0] * gamma[1];
            }
            cache.softness[0] = soft.massScale;
            cache.softness[1] = soft.impulseScale;
            cache.softReady = true;
        }
        const int combo = (softRows[0] ? 1 : 0) | (softRows[1] ? 2 : 0);
        const double a11 = cache.diagonal[combo][0], a22 = cache.diagonal[combo][1];
        for (int i = 0; i < 2; ++i) q[i] = normalVelocities[i] + biases[i];
        q[0] = Checked(q[0] - (k11 * old[0] + k12 * old[1]));
        q[1] = Checked(q[1] - (k12 * old[0] + k22 * old[1]));
        const double determinant = Checked(cache.determinants[combo]);
        double x[2]{};
        bool solved = false;
        if (determinant > 1e-12 * a11 * a22) {
            x[0] = (-a22 * q[0] + k12 * q[1]) / determinant;
            x[1] = (k12 * q[0] - a11 * q[1]) / determinant;
            solved = x[0] >= 0 && x[1] >= 0;
        }
        if (!solved) {
            x[0] = std::max(-q[0] / a11, 0.0); x[1] = 0;
            if (x[0] > 0 && k12 * x[0] + q[1] >= 0) solved = true;
        }
        if (!solved) {
            x[0] = 0; x[1] = std::max(-q[1] / a22, 0.0);
            if (x[1] > 0 && k12 * x[1] + q[0] >= 0) solved = true;
        }
        if (!solved) { x[0] = 0; x[1] = 0; }
        const double d1 = Checked(x[0] - old[0]), d2 = Checked(x[1] - old[1]);
        ApplyCombinedVelocity(a, b, n * Checked(d1 + d2), Checked(ca[0] * d1 + ca[1] * d2),
                              Checked(cb[0] * d1 + cb[1] * d2), false);
        for (std::uint8_t i = 0; i < 2; ++i) {
            PublishImpulse(c, c.points[i], i, {x[i], c.points[i].impulse.tangent});
            c.points[i].maxNormalImpulse = std::max(c.points[i].maxNormalImpulse, x[i]);
        }
    } else for (std::uint8_t i = 0; i < c.pointCount; ++i) {
        if (skip[i]) continue;
        auto &p = c.points[i];
        const DVector ra = LeverA(p), rb = LeverB(p);
        const double bias = biases[i], massScale = massScales[i], impulseScale = impulseScales[i];
        const double normalVelocity = Checked((LeverVelocity(b, rb) - LeverVelocity(a, ra)).dot(n));
        const double change = Checked(-p.normalMass * massScale * (normalVelocity + bias) -
                                      impulseScale * p.impulse.normal);
        const double total = std::max(Checked(p.impulse.normal + change), 0.0);
        ApplyPair(a, b, ra, rb, n * (total - p.impulse.normal), false, false);
        PublishImpulse(c, p, i, {total, p.impulse.tangent});
        p.maxNormalImpulse = std::max(p.maxNormalImpulse, total);
    }
    for (std::uint8_t i = 0; i < c.pointCount; ++i) {
        auto &p = c.points[i];
        const DVector ra = LeverA(p), rb = LeverB(p);
        const double tangentVelocity = Checked((LeverVelocity(b, rb) - LeverVelocity(a, ra)).dot(t));
        const double maximumStatic = Checked(p.impulse.normal * c.staticFriction);
        if (!(std::abs(tangentVelocity) > c.velocityTolerance) && std::abs(p.impulse.tangent) <= maximumStatic)
            continue;
        const double candidate = Checked(p.impulse.tangent - p.tangentMass * tangentVelocity);
        double tangent = candidate;
        if (std::abs(candidate) > maximumStatic) {
            const double maximumDynamic = Checked(p.impulse.normal * c.dynamicFriction);
            tangent = std::clamp(candidate, -maximumDynamic, maximumDynamic);
        }
        ApplyPair(a, b, ra, rb, t * (tangent - p.impulse.tangent), false, false);
        PublishImpulse(c, p, i, {p.impulse.normal, tangent});
    }
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

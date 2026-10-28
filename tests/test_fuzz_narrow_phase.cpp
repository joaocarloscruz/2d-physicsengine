// Seeded property tests for the narrow phase, the standalone CCD sweeps and
// the spatial queries, each checked against the independent oracle in
// fuzz_support.h. Every failure message carries the seed, the case index and
// an exact (hex float) description of both shapes and poses.
//
// Tolerances (see docs/numerical-validation.md, "Seeded property tests"):
//  - Depth/normal: 3e-5 * the larger shape extent. The narrow phase prefers
//    a face within an intentional allowance of the minimum (1e-5 * min
//    polygon width, 2e-5 * min radius, see docs/polygon-manifold-numerics.md
//    and docs/capsules.md); the rest of the arithmetic is double on exact
//    float inputs, so only those allowances and the float result remain.
//  - Positions: the depth tolerance plus two float ulps of the largest world
//    coordinate, since contact points are rounded to float world coordinates.
#include "catch_amalgamated.hpp"
#include "fuzz_support.h"

#include "physics/core/collisions/collision_dispatcher.h"
#include "physics/core/collisions/continuous_collision.h"
#include "physics/core/material.h"
#include "physics/core/spatial_queries.h"

#include <functional>
#include <optional>
#include <sstream>

using namespace PhysicsEngine;
using namespace FuzzSupport;

namespace {

struct PairCase {
    ShapeSpec a, b;
    Vector2 positionA, positionB;
    float orientationA = 0, orientationB = 0;
    std::uint64_t seed = 0;
    int index = 0;
    std::string Describe() const {
        std::ostringstream text;
        text << "seed=" << seed << " case=" << index << "\n  A=" << a.Describe() << " at " << Hex(positionA)
             << " orientation " << Hex(orientationA) << "f\n  B=" << b.Describe() << " at " << Hex(positionB)
             << " orientation " << Hex(orientationB) << "f";
        return text.str();
    }
};

float RandomOrientation(Rng& rng) {
    if (rng.Chance(0.15)) return static_cast<float>(rng.Integer(-4, 4) * (Pi / 2));
    return static_cast<float>(rng.Range(-Pi, Pi));
}

Vector2 RandomBase(Rng& rng, double maximum) {
    const int kind = rng.Integer(0, 3);
    const double magnitude = kind == 0 ? 0 : kind == 1 ? rng.Range(0, 10) : kind == 2 ? rng.Range(10, 1000)
                                                                                         : rng.Range(1000, maximum);
    const double angle = rng.Range(-Pi, Pi);
    return {Quantize(magnitude * std::cos(angle)), Quantize(magnitude * std::sin(angle))};
}

PairCase RandomPair(Rng& rng, ShapeType ta, ShapeType tb, double baseMaximum) {
    PairCase c;
    c.a = RandomShape(rng, ta);
    c.b = RandomShape(rng, tb);
    const Vector2 base = RandomBase(rng, baseMaximum);
    c.positionA = base;
    const double reach = c.a.Extent() + c.b.Extent();
    const double distance = rng.Chance(0.05) ? 0 : rng.Range(0, 1.15 * reach);
    const double angle = rng.Range(-Pi, Pi);
    c.positionB = {Quantize(base.x + distance * std::cos(angle)), Quantize(base.y + distance * std::sin(angle))};
    c.orientationA = RandomOrientation(rng);
    c.orientationB = RandomOrientation(rng);
    return c;
}

struct Bodies {
    std::unique_ptr<Shape> shapeA, shapeB;
    std::unique_ptr<RigidBody> a, b;
    Bodies(const PairCase& c, Vector2 shift = {}) {
        shapeA = c.a.Make();
        shapeB = c.b.Make();
        a = std::make_unique<RigidBody>(*shapeA, Material{}, c.positionA + shift);
        b = std::make_unique<RigidBody>(*shapeB, Material{}, c.positionB + shift);
        a->orientation = c.orientationA;
        b->orientation = c.orientationB;
    }
};

std::string Show(const CollisionManifold& m) {
    std::ostringstream text;
    text << "hasCollision=" << m.hasCollision << " count=" << int(m.contactCount) << " penetration=" << m.penetration
         << " normal=(" << m.normal.x << ", " << m.normal.y << ")";
    for (int i = 0; i < m.contactCount; ++i)
        text << " contact" << i << "=(" << m.contacts[i].position.x << ", " << m.contacts[i].position.y
             << ") depth " << m.contacts[i].penetration << " id " << m.contacts[i].featureId;
    return text.str();
}

struct Tolerance {
    double depth, position, scale;
};
Tolerance TolerancesFor(const PairCase& c, Vector2 shift = {}) {
    const double extent = std::max(c.a.Extent(), c.b.Extent());
    const double coordinate = std::max({std::abs(c.positionA.x + shift.x), std::abs(c.positionA.y + shift.y),
                                        std::abs(c.positionB.x + shift.x), std::abs(c.positionB.y + shift.y)})
        + c.a.Extent() + c.b.Extent();
    const double depth = 3e-5 * extent + 1e-7;
    return {depth, depth + 2 * Ulp(coordinate), extent};
}

std::string PairName(const PairCase& c) {
    const auto name = [](ShapeType t) {
        return t == ShapeType::CIRCLE ? "circle" : t == ShapeType::POLYGON ? "polygon" : "capsule";
    };
    return std::string(name(c.a.type)) + "-" + name(c.b.type);
}

bool HasCapsule(const PairCase& c) { return c.a.type == ShapeType::CAPSULE || c.b.type == ShapeType::CAPSULE; }

// Structural invariants of any manifold. contact-manifolds.md makes the
// manifold depth the deepest point; for capsules, capsules.md makes it the
// exact minimum translation, which bounds every point depth.
void CheckStructure(const CollisionManifold& m, bool capsulePair, const std::string& where, Failures& failures) {
    bool ok = m.hasCollision == (m.contactCount > 0) && m.contactCount <= 2;
    if (m.hasCollision) {
        float deepest = -std::numeric_limits<float>::infinity();
        for (int i = 0; i < m.contactCount; ++i) {
            const auto& p = m.contacts[i];
            ok = ok && std::isfinite(p.position.x) && std::isfinite(p.position.y) && std::isfinite(p.penetration)
                && p.featureId != 0;
            deepest = std::max(deepest, p.penetration);
        }
        if (m.contactCount == 2) ok = ok && m.contacts[0].featureId != m.contacts[1].featureId;
        ok = ok && (capsulePair ? deepest <= m.penetration : deepest == m.penetration)
            && m.contactPoint == m.contacts[0].position;
        const double length = std::hypot(double(m.normal.x), double(m.normal.y));
        if (std::abs(length - 1) > 1e-6) failures.Add("unit normal: |n|=" + std::to_string(length) + "\n" + where);
    }
    if (!ok) failures.Add("manifold structure\n" + where);
}

// Depth, normal and contact points against the oracle, for a manifold that
// the oracle says must exist.
void CheckAgainstOracle(const CollisionManifold& m, const PairCase& c, const Geo& ga, const Geo& gb, double expected,
                        const Tolerance& tol, const std::string& where, Failures& failures) {
    if (std::abs(double(m.penetration) - expected) > tol.depth)
        failures.Add("penetration differs from the oracle minimum translation (" + PairName(c) + "): " + std::to_string(m.penetration)
                     + " vs " + std::to_string(expected) + "\n" + where);
    const D2 n = ToD2(m.normal);
    const double along = OverlapAlong(ga, gb, n * (1 / n.Norm()));
    if (along - expected > tol.depth)
        failures.Add("normal is not a minimum-translation direction: overlap along it " + std::to_string(along)
                     + " vs oracle " + std::to_string(expected) + "\n" + where);
    // A contact lies inside or on both shapes, or, when speculative, no
    // farther from either than its own gap, and no deeper in either than the
    // penetration. Documented conventions widen this:
    //  - circle-circle and circle-polygon contacts stay on a circle surface
    //    (contact-manifolds.md, collision-lifecycle-and-ccd.md), so once the
    //    depth exceeds the thinner shape the point can leave the other shape;
    //    the "inside both" half is then only checked for that circle surface;
    //  - capsule contacts are midpoints measured along the normal, which on an
    //    oblique reference face sit inside the capsule by up to its spine length.
    const bool circleSurface = c.a.type == ShapeType::CIRCLE && c.b.type != ShapeType::CAPSULE;
    const bool reverseCircle = c.b.type == ShapeType::CIRCLE && c.a.type == ShapeType::POLYGON;
    const double spine = 2 * std::max(c.a.type == ShapeType::CAPSULE ? c.a.halfLength : 0.0f,
                                      c.b.type == ShapeType::CAPSULE ? c.b.halfLength : 0.0f);
    for (int i = 0; i < m.contactCount; ++i) {
        const double gap = std::max(-double(m.contacts[i].penetration), 0.0);
        const D2 p = ToD2(m.contacts[i].position);
        const double da = SignedDistance(ga, p), db = SignedDistance(gb, p);
        const double outside = gap + tol.position, inside = std::max(expected, 0.0) + spine + tol.position;
        // Coincident circle centres report A's centre (contact-manifolds.md).
        if (c.a.type == ShapeType::CIRCLE && c.b.type == ShapeType::CIRCLE && c.positionA == c.positionB) continue;
        const double depth = std::max(expected, 0.0);
        bool ok = da >= -inside && db >= -inside && da <= outside + depth && db <= outside + depth;
        if (circleSurface && std::abs(da) > outside - gap) ok = false;
        if (reverseCircle && std::abs(db) > outside - gap) ok = false;
        // Polygon pairs keep contacts on the incident edge, except the
        // separated vertex-to-vertex point (ordinal 3), which is the midpoint
        // of the true gap (contact-solver.md, step 2).
        if (ga.IsPolygon() && gb.IsPolygon()) {
            if ((m.contacts[i].featureId & 3u) == 3u) {
                if (std::abs(da - gap / 2) > tol.position || std::abs(db - gap / 2) > tol.position) ok = false;
            } else if (std::min(std::abs(da), std::abs(db)) > tol.position) {
                ok = false;
            }
        }
        // Capsule contacts are midpoints between two surface points that are
        // the contact depth apart along the normal (capsules.md).
        if (HasCapsule(c)) {
            const D2 n = ToD2(m.normal) * (0.5 * double(m.contacts[i].penetration));
            const double sa = SignedDistance(ga, p + n), sb = SignedDistance(gb, p - n);
            if (sa > tol.position || sb > tol.position || sa < -spine - tol.position || sb < -spine - tol.position)
                ok = false;
        }
        if (!ok)
            failures.Add("contact point outside its slop of a shape (" + PairName(c) + "): contact " + std::to_string(i) + " sdA="
                         + std::to_string(da) + " sdB=" + std::to_string(db) + " gap " + std::to_string(gap) + "\n" + where);
    }
}

bool SameContacts(const CollisionManifold& m, const CollisionManifold& r, D2 shift, double tolerance) {
    if (m.contactCount != r.contactCount) return false;
    const auto near = [&](int i, int j) {
        const D2 d = ToD2(r.contacts[j].position) - ToD2(m.contacts[i].position) - shift;
        return d.Norm() <= tolerance;
    };
    if (m.contactCount == 1) return near(0, 0);
    if (m.contactCount == 2) return (near(0, 0) && near(1, 1)) || (near(0, 1) && near(1, 0));
    return true;
}

void CheckPairCase(const PairCase& c, Rng& rng, Failures& failures) {
    Bodies bodies(c);
    const Geo ga = GeoOf(c.a, c.positionA, c.orientationA), gb = GeoOf(c.b, c.positionB, c.orientationB);
    const Oracle oracle = Penetration(ga, gb);
    const Tolerance tol = TolerancesFor(c);
    const std::string where = c.Describe();

    const CollisionManifold m = CheckCollision(bodies.a.get(), bodies.b.get(), 0);
    const CollisionManifold r = CheckCollision(bodies.b.get(), bodies.a.get(), 0);
    CheckStructure(m, HasCapsule(c), where + "\n  forward: " + Show(m), failures);
    CheckStructure(r, HasCapsule(c), where + "\n  reverse: " + Show(r), failures);

    // Overlap agreement, outside a band around exact touch.
    if (oracle.penetration > tol.depth && !m.hasCollision)
        failures.Add("missed overlap, oracle penetration " + std::to_string(oracle.penetration) + "\n" + where);
    if (oracle.penetration < -tol.depth && m.hasCollision)
        failures.Add("separated pair reported at margin 0, oracle gap " + std::to_string(-oracle.penetration) + "\n"
                     + where + "\n  " + Show(m));
    if (m.hasCollision && oracle.penetration > -tol.depth)
        CheckAgainstOracle(m, c, ga, gb, std::max(oracle.penetration, 0.0), tol, where + "\n  forward: " + Show(m), failures);

    // Swap symmetry.
    const bool circles = c.a.type == ShapeType::CIRCLE && c.b.type == ShapeType::CIRCLE;
    if (m.hasCollision != r.hasCollision || m.contactCount != r.contactCount) {
        failures.Add("swap changes classification\n" + where + "\n  forward: " + Show(m) + "\n  reverse: " + Show(r));
    } else if (m.hasCollision) {
        const bool coincident = circles && c.positionA == c.positionB;
        const D2 sum = ToD2(m.normal) + ToD2(r.normal);
        bool ok = std::abs(double(m.penetration) - r.penetration) <= 1e-6 * tol.scale + 1e-9
            && r.A == bodies.b.get() && r.B == bodies.a.get();
        if (!coincident) ok = ok && sum.Norm() <= 1e-6;
        // Circle pairs keep the surface-of-A contact (docs/contact-manifolds.md):
        // reversed, the point moves by the penetration along the normal.
        const D2 shift = circles ? ToD2(m.normal) * -double(m.penetration) : D2{};
        if (!coincident) ok = ok && SameContacts(m, r, shift, tol.position);
        if (!ok) failures.Add("swap symmetry\n" + where + "\n  forward: " + Show(m) + "\n  reverse: " + Show(r));
    }

    // Translation invariance under an exact integer shift.
    const double baseMagnitude = std::max(std::abs(c.positionA.x), std::abs(c.positionA.y));
    if (baseMagnitude < 4000) {
        const Vector2 shift{float(rng.Integer(-10000, 10000)), float(rng.Integer(-10000, 10000))};
        Bodies moved(c, shift);
        const CollisionManifold t = CheckCollision(moved.a.get(), moved.b.get(), 0);
        const Tolerance shifted = TolerancesFor(c, shift);
        bool ok = t.hasCollision == m.hasCollision && t.contactCount == m.contactCount;
        if (ok && m.hasCollision) {
            ok = std::abs(double(t.penetration) - m.penetration) <= 1e-6 * tol.scale + 1e-9
                && (ToD2(t.normal) - ToD2(m.normal)).Norm() <= 1e-6
                && SameContacts(m, t, ToD2(shift), std::max(tol.position, shifted.position));
        }
        if (!ok)
            failures.Add("translation invariance, shift " + Hex(shift) + "\n" + where + "\n  original: " + Show(m)
                         + "\n  shifted: " + Show(t));
    }

    // Speculative margin.
    const float margin = float(rng.LogRange(1e-3, 0.5) * tol.scale);
    const CollisionManifold s = CheckCollision(bodies.a.get(), bodies.b.get(), margin);
    const CollisionManifold sr = CheckCollision(bodies.b.get(), bodies.a.get(), margin);
    const std::string marginWhere = where + "\n  margin " + Hex(margin) + "f: " + Show(s);
    CheckStructure(s, HasCapsule(c), marginWhere, failures);
    if (oracle.penetration > -double(margin) + tol.depth) {
        if (!s.hasCollision)
            failures.Add("margin: missed pair within margin, oracle penetration " + std::to_string(oracle.penetration) + "\n" + marginWhere);
        else
            CheckAgainstOracle(s, c, ga, gb, oracle.penetration, tol, marginWhere, failures);
    } else if (oracle.penetration < -double(margin) - tol.depth && s.hasCollision) {
        failures.Add("margin: pair beyond margin reported, oracle gap " + std::to_string(-oracle.penetration) + "\n" + marginWhere);
    }
    if (s.hasCollision != sr.hasCollision
        || (s.hasCollision && std::abs(double(s.penetration) - sr.penetration) > 1e-6 * tol.scale + 1e-9))
        failures.Add("margin swap symmetry\n" + marginWhere + "\n  reverse: " + Show(sr));
}

// ---- Spatial queries -------------------------------------------------------

struct Root {
    bool contained = false, hit = false, ambiguous = false;
    double t = 0, tMinimum = 0;
};
// First root of a convex function on [0, 1] by ternary search for its
// minimum and bisection before it.
Root FirstRoot(const std::function<double(double)>& f, double tol) {
    Root root;
    const double f0 = f(0);
    if (f0 <= 0) {
        root.contained = root.hit = true;
        root.ambiguous = f0 > -tol;
        return root;
    }
    if (f0 < tol) root.ambiguous = true;
    double low = 0, high = 1;
    for (int i = 0; i < 200 && high - low > 1e-15; ++i) {
        const double m1 = low + (high - low) / 3, m2 = high - (high - low) / 3;
        if (f(m1) <= f(m2)) high = m2; else low = m1;
    }
    root.tMinimum = 0.5 * (low + high);
    const double minimum = std::min({f(root.tMinimum), f(1.0)});
    if (f(1.0) < f(root.tMinimum)) root.tMinimum = 1;
    if (minimum > tol) return root;
    if (minimum >= -tol) { root.ambiguous = true; return root; }
    root.hit = true;
    low = 0; high = root.tMinimum;
    for (int i = 0; i < 200; ++i) {
        const double mid = 0.5 * (low + high);
        if (f(mid) > 0) low = mid; else high = mid;
    }
    root.t = 0.5 * (low + high);
    return root;
}

D2 Lerp(D2 a, D2 b, double t) { return a + (b - a) * t; }

void CheckQueries(const ShapeSpec& spec, const RigidBody& body, const Geo& g, const std::string& where, Rng& rng,
                  Failures& failures) {
    const double extent = spec.Extent();
    const double coordinate = std::max(std::abs(body.position.x), std::abs(body.position.y)) + 3 * extent;
    const double tol = 1e-6 * extent + 4 * Ulp(coordinate);
    const auto randomPoint = [&](double spread) {
        return Vector2{float(body.position.x + rng.Range(-spread, spread)), float(body.position.y + rng.Range(-spread, spread))};
    };

    for (int i = 0; i < 3; ++i) {
        const Vector2 p = randomPoint(1.3 * extent);
        const double sd = SignedDistance(g, ToD2(p));
        if (std::abs(sd) > tol && ContainsPoint(body, p) != (sd < 0))
            failures.Add("ContainsPoint " + Hex(p) + " sd=" + std::to_string(sd) + "\n" + where);
    }

    {
        const Vector2 center = randomPoint(2 * extent);
        const float radius = rng.Chance(0.1) ? 0.0f : float(rng.Range(0, extent));
        const double gap = SignedDistance(g, ToD2(center)) - radius;
        if (std::abs(gap) > tol) {
            const bool overlaps = OverlapsCircle(body, center, radius);
            if (overlaps != (gap < 0))
                failures.Add("OverlapsCircle " + Hex(center) + " radius " + Hex(radius) + "f gap=" + std::to_string(gap) + "\n" + where);
            if (radius > 0) {
                RigidBody probe(Circle(radius), Material{}, center);
                RigidBody target(*spec.Make(), Material{}, body.position);
                target.orientation = body.orientation;
                if (CheckCollision(&target, &probe, 0).hasCollision != overlaps)
                    failures.Add("OverlapsCircle disagrees with CheckCollision " + Hex(center) + " radius " + Hex(radius)
                                 + "f gap=" + std::to_string(gap) + "\n" + where);
            }
        }
    }

    // Ray cast and circle sweep (radius 0 is the ray itself).
    for (int sweep = 0; sweep < 2; ++sweep) {
        const Vector2 start = randomPoint(rng.Chance(0.2) ? 0.8 * extent : 3 * extent);
        const Vector2 end = rng.Chance(0.05) ? start : randomPoint(3 * extent);
        const float radius = sweep == 0 ? 0.0f : float(rng.LogRange(0.01, 1) * extent);
        const D2 s = ToD2(start), e = ToD2(end);
        const double length = (e - s).Norm();
        const auto f = [&](double t) { return SignedDistance(g, Lerp(s, e, t)) - radius; };
        const Root root = FirstRoot(f, tol);
        if (root.ambiguous) continue;
        const std::string query = std::string(sweep ? "SweepCircle" : "RayCast") + " start " + Hex(start) + " end "
            + Hex(end) + " radius " + Hex(radius) + "f\n" + where;
        std::optional<D2> point, normal, center;
        double fraction = 0;
        if (sweep == 0) {
            const auto hit = RayCast(body, start, end);
            if (hit) { point = ToD2(hit->point); normal = ToD2(hit->normal); center = point; fraction = hit->fraction; }
        } else {
            const auto hit = SweepCircle(body, start, end, radius);
            if (hit) { point = ToD2(hit->contactPoint); normal = ToD2(hit->normal); center = ToD2(hit->center); fraction = hit->fraction; }
        }
        if (root.hit != point.has_value()) {
            failures.Add(query + "\n  hit=" + std::to_string(point.has_value()) + " oracle hit=" + std::to_string(root.hit)
                         + " t=" + std::to_string(root.t));
            continue;
        }
        if (!point) continue;
        if (root.contained) {
            if (fraction != 0 || normal->x != 0 || normal->y != 0)
                failures.Add(query + "\n  contained start must report fraction 0 and a zero normal");
            continue;
        }
        bool ok = std::abs(f(fraction)) <= tol + 1e-12 * length && fraction <= root.tMinimum + 1e-9
            && (*center - Lerp(s, e, fraction)).Norm() <= tol
            && std::abs(SignedDistance(g, *point)) <= tol
            && std::abs((*center - *point).Norm() - radius) <= tol;
        const double nl = normal->Norm();
        ok = ok && std::abs(nl - 1) <= 1e-6;
        if (ok) {
            // A supporting outward normal moves exactly away from the shape.
            const double delta = 0.01 * extent;
            ok = SignedDistance(g, *point + *normal * delta) >= delta - tol - 1e-6 * delta
                && normal->Dot(e - s) <= 1e-6 * length;
        }
        if (!ok)
            failures.Add(query + "\n  fraction " + std::to_string(fraction) + " oracle " + std::to_string(root.t)
                         + " point (" + std::to_string(point->x) + ", " + std::to_string(point->y) + ") normal ("
                         + std::to_string(normal->x) + ", " + std::to_string(normal->y) + ")");
    }
}

void RunNarrowPhase(std::uint64_t seed, int casesPerPair, double baseMaximum) {
    const ShapeType types[] = {ShapeType::CIRCLE, ShapeType::POLYGON, ShapeType::CAPSULE};
    Failures failures;
    for (int i = 0; i < 3; ++i)
        for (int j = i; j < 3; ++j) {
            Rng rng(seed * 31 + std::uint64_t(i * 3 + j));
            for (int k = 0; k < casesPerPair; ++k) {
                PairCase c = RandomPair(rng, types[i], types[j], baseMaximum);
                c.seed = seed;
                c.index = k;
                const std::string where = "pair " + std::to_string(i) + std::to_string(j) + " " + c.Describe();
                try {
                    CheckPairCase(c, rng, failures);
                    if (k % 2 == 0) {
                        Bodies bodies(c);
                        CheckQueries(c.a, *bodies.a, GeoOf(c.a, c.positionA, c.orientationA), "query A of " + where, rng, failures);
                    }
                } catch (const std::exception& error) {
                    failures.Add(std::string("exception: ") + error.what() + "\n" + where);
                }
            }
        }
    INFO(failures.Summary());
    CHECK(failures.total == 0);
}

// ---- CCD sweeps --------------------------------------------------------------

void RunSweeps(std::uint64_t seed, int cases) {
    Failures failures;
    Rng rng(seed);
    for (int k = 0; k < cases; ++k) {
        const int kind = k % 3;
        const ShapeType targetType = kind == 0 ? ShapeType::CIRCLE : kind == 1 ? ShapeType::POLYGON : ShapeType::CAPSULE;
        const ShapeSpec target = RandomShape(rng, targetType);
        const float radius = float(rng.LogRange(0.02, 2));
        const Vector2 base = RandomBase(rng, 1e4);
        const float orientation = RandomOrientation(rng);
        const double reach = target.Extent() + radius;
        const double angle = rng.Range(-Pi, Pi), distance = rng.Range(0, 4 * reach);
        const Vector2 center{Quantize(base.x + distance * std::cos(angle)), Quantize(base.y + distance * std::sin(angle))};
        const auto randomMotion = [&](double scale) {
            if (rng.Chance(0.1)) return Vector2{};
            const double a = rng.Range(-Pi, Pi), l = rng.LogRange(1e-2, 1) * scale;
            return Vector2{float(l * std::cos(a)), float(l * std::sin(a))};
        };
        // Aim roughly at the target so a good share of sweeps hit.
        Vector2 motion = randomMotion(8 * reach);
        if (rng.Chance(0.6)) {
            const double l = rng.LogRange(0.5, 3) * distance + 1e-3;
            const double a = std::atan2(base.y - center.y, base.x - center.x) + rng.Range(-0.5, 0.5);
            motion = {float(l * std::cos(a)), float(l * std::sin(a))};
        }
        const Vector2 targetMotion = rng.Chance(0.5) ? Vector2{} : randomMotion(4 * reach);

        const Geo g = GeoOf(target, base, orientation);
        const D2 c0 = ToD2(center), relative = ToD2(motion) - ToD2(targetMotion);
        const double coordinate = std::max({std::abs(center.x), std::abs(center.y), std::abs(base.x), std::abs(base.y)})
            + 10 * reach + relative.Norm();
        const double tol = 1e-6 * reach + 4 * Ulp(coordinate);
        const auto f = [&](double t) { return SignedDistance(g, c0 + relative * t) - radius; };

        // Brute-force time stepping brackets the first contact, bisection refines it.
        Root root;
        const double f0 = f(0);
        root.contained = f0 <= 0;
        root.ambiguous = std::abs(f0) <= tol;
        if (!root.contained) {
            constexpr int steps = 2048;
            int first = -1;
            for (int i = 1; i <= steps && first < 0; ++i)
                if (f(double(i) / steps) <= 0) first = i;
            const Root convex = FirstRoot(f, tol); // catches windows shorter than one step
            root.ambiguous = root.ambiguous || convex.ambiguous;
            root.tMinimum = convex.tMinimum;
            if (first >= 0 || convex.hit) {
                root.hit = true;
                double low = first >= 0 ? double(first - 1) / steps : 0, high = first >= 0 ? double(first) / steps : convex.tMinimum;
                for (int i = 0; i < 200; ++i) {
                    const double mid = 0.5 * (low + high);
                    if (f(mid) > 0) low = mid; else high = mid;
                }
                root.t = 0.5 * (low + high);
            }
        }
        root.hit = root.hit || root.contained;
        if (root.ambiguous) continue;

        std::ostringstream text;
        text << "sweep seed=" << seed << " case=" << k << " circle center " << Hex(center) << " radius " << Hex(radius)
             << "f motion " << Hex(motion) << " target " << target.Describe() << " at " << Hex(base) << " orientation "
             << Hex(orientation) << "f motion " << Hex(targetMotion);
        const std::string where = text.str();
        SweepHit hit;
        try {
            if (targetType == ShapeType::CIRCLE) {
                hit = SweepCircleCircle(center, motion, radius, base, targetMotion, target.radius);
            } else if (targetType == ShapeType::POLYGON) {
                std::vector<Vector2> world;
                for (const auto& p : g.core) world.push_back({float(p.x), float(p.y)});
                // Rounding the world outline to float can break strict convexity.
                try { Polygon::ValidateOutline(world); } catch (const std::invalid_argument&) { continue; }
                const Geo rounded = [&] { Geo r; for (auto v : world) r.core.push_back(ToD2(v)); return r; }();
                if (std::abs(SignedDistance(rounded, c0) - SignedDistance(g, c0)) > tol) continue;
                hit = SweepCirclePolygon(center, motion, radius, world, targetMotion);
                // Re-evaluate with the exact float outline the sweep received.
                const auto fr = [&](double t) { return SignedDistance(rounded, c0 + relative * t) - radius; };
                const Root again = FirstRoot(fr, tol);
                if (again.ambiguous || again.hit != root.hit) continue;
            } else {
                const D2 a = g.core.front(), b = g.core.back();
                hit = SweepCircleCapsule(center, motion, radius, {float(a.x), float(a.y)}, {float(b.x), float(b.y)},
                                         target.radius, targetMotion);
            }
        } catch (const std::exception& error) {
            failures.Add(std::string("exception: ") + error.what() + "\n" + where);
            continue;
        }
        if (hit.hit != root.hit) {
            failures.Add("sweep hit=" + std::to_string(hit.hit) + " oracle=" + std::to_string(root.hit) + " t="
                         + std::to_string(root.t) + "\n" + where);
            continue;
        }
        if (!hit.hit) continue;
        const D2 n = ToD2(hit.normal);
        bool ok = std::abs(n.Norm() - 1) <= 1e-6;
        if (root.contained) {
            ok = ok && hit.fraction == 0;
        } else {
            // Fraction is a float: allow its rounding along the relative path.
            const double travel = relative.Norm();
            const double slack = tol + (double(hit.fraction) * 6e-8 + 1e-12) * travel;
            // On the entry side of the convex clearance minimum, a near-zero
            // clearance pins the fraction to the first contact as tightly as
            // the approach angle allows; a grazing approach permits more.
            ok = ok && std::abs(f(hit.fraction)) <= slack && hit.fraction <= root.tMinimum + 1e-9;
            // The point is on the target, translated to the impact time, and
            // the normal points from the circle into the target.
            const D2 shiftTarget = ToD2(targetMotion) * double(hit.fraction);
            const D2 local = ToD2(hit.point) - shiftTarget;
            ok = ok && std::abs(SignedDistance(g, local)) <= slack
                && SignedDistance(g, local - n * (0.01 * reach)) >= 0.01 * reach - slack - 1e-6 * reach;
        }
        if (!ok)
            failures.Add("sweep fraction " + std::to_string(hit.fraction) + " oracle " + std::to_string(root.t)
                         + " clearance there " + std::to_string(f(hit.fraction)) + " normal ("
                         + std::to_string(hit.normal.x) + ", " + std::to_string(hit.normal.y) + ") point ("
                         + std::to_string(hit.point.x) + ", " + std::to_string(hit.point.y) + ")\n" + where);
    }
    INFO(failures.Summary());
    CHECK(failures.total == 0);
}

} // namespace

TEST_CASE("Oracle self-check: penetration is the minimum projected overlap", "[fuzz][narrow-phase]") {
    Rng rng(7);
    for (int k = 0; k < 300; ++k) {
        const ShapeType types[] = {ShapeType::CIRCLE, ShapeType::POLYGON, ShapeType::CAPSULE};
        const PairCase c = RandomPair(rng, types[k % 3], types[(k / 3) % 3], 10);
        const Geo ga = GeoOf(c.a, c.positionA, c.orientationA), gb = GeoOf(c.b, c.positionB, c.orientationB);
        const double expected = Penetration(ga, gb).penetration;
        double sampled = std::numeric_limits<double>::infinity();
        for (int i = 0; i < 20000; ++i) {
            const double a = 2 * Pi * i / 20000;
            sampled = std::min(sampled, OverlapAlong(ga, gb, {std::cos(a), std::sin(a)}));
        }
        INFO(c.Describe());
        // Dense sampling can only overestimate the minimum, by about the
        // angular step times the shapes' extent.
        REQUIRE(sampled >= expected - 1e-9);
        REQUIRE(sampled <= expected + 2 * Pi / 20000 * (c.a.Extent() + c.b.Extent()) * 2 + 1e-9);
    }
}

TEST_CASE("Narrow phase properties hold for random shape pairs", "[fuzz][narrow-phase]") {
    RunNarrowPhase(20261028, 6000, 1e4);
}

TEST_CASE("Narrow phase properties hold for random shape pairs (long)", "[.fuzz-long][narrow-phase]") {
    for (std::uint64_t seed = 1; seed <= 8; ++seed) RunNarrowPhase(seed, 25000, 1e4);
}

TEST_CASE("Circle sweeps agree with brute-force time stepping", "[fuzz][ccd]") {
    RunSweeps(20261028, 9000);
}

TEST_CASE("Circle sweeps agree with brute-force time stepping (long)", "[.fuzz-long][ccd]") {
    for (std::uint64_t seed = 1; seed <= 8; ++seed) RunSweeps(seed, 30000);
}



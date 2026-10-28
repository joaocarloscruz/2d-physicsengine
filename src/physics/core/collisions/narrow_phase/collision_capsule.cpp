#include "physics/core/collisions/narrow_phase/collision_capsule.h"
#include "physics/core/shape.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <initializer_list>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

// A capsule is a spine segment rounded by a radius; a circle is the same
// shape with a zero-length spine. Penetration is the exact minimum
// translation of the rounded shapes: the minimum, over the face normals of
// the spines' Minkowski difference and (for disjoint spines) the direction
// between their closest points, of the projected interval overlap. A closest
// spine distance alone would misreport crossing spines.
namespace PhysicsEngine {
namespace {
struct Point {
    double x, y;
    Point operator+(Point p) const { return {x + p.x, y + p.y}; }
    Point operator-(Point p) const { return {x - p.x, y - p.y}; }
    Point operator*(double s) const { return {x * s, y * s}; }
    double dot(Point p) const { return x * p.x + y * p.y; }
    double norm() const { return std::hypot(x, y); }
};
Point Perp(Point p) { return {-p.y, p.x}; }
Point ToPoint(Vector2 p) { return {p.x, p.y}; }
float CheckedFloat(double value) {
    if (!std::isfinite(value) || std::abs(value) > std::numeric_limits<float>::max())
        throw std::overflow_error("Capsule manifold exceeds finite float range.");
    return static_cast<float>(value);
}
Vector2 CheckedVector(Point p) { return {CheckedFloat(p.x), CheckedFloat(p.y)}; }
void ValidateTransform(const RigidBody* body) {
    if (!body || !body->shape || !std::isfinite(body->position.x)
        || !std::isfinite(body->position.y) || !std::isfinite(body->orientation))
        throw std::invalid_argument("Capsule collision requires finite body transforms.");
}
double Sign(double value) { return value > 0 ? 1 : (value < 0 ? -1 : 0); }

// Feature IDs keep bit 31 clear (polygon pairs own it) and use pair kinds
// distinct from the existing 0x2/0x4/0x6 circle and CCD prefixes.
constexpr std::uint32_t CircleCapsuleKind = 0x11000000u;
constexpr std::uint32_t PolygonCapsuleKind = 0x12000000u;
constexpr std::uint32_t CapsuleCapsuleKind = 0x13000000u;
constexpr std::uint32_t FaceFeature = 1u << 23;
constexpr std::uint32_t SecondReference = 1u << 22;
constexpr std::uint32_t NegativeFace = 1u << 21;

struct Rounded {
    Point center, axis; // unit spine direction
    double halfLength, radius;
    Point End(int index) const { return center + axis * (index == 0 ? -halfLength : halfLength); }
    double Reach(Point normal) const { return halfLength * std::abs(axis.dot(normal)); }
    Point ClosestTo(Point p) const {
        return center + axis * std::clamp(axis.dot(p - center), -halfLength, halfLength);
    }
};
Rounded MakeRounded(const RigidBody* body, Point origin) {
    ValidateTransform(body);
    const Point center = ToPoint(body->position) - origin;
    if (body->shape->type == ShapeType::CIRCLE) {
        const auto* circle = dynamic_cast<const Circle*>(body->shape.get());
        if (!circle || !std::isfinite(circle->GetRadius()) || circle->GetRadius() <= 0)
            throw std::invalid_argument("Capsule collision requires a valid Circle.");
        return {center, {1, 0}, 0, circle->GetRadius()};
    }
    const auto* capsule = dynamic_cast<const Capsule*>(body->shape.get());
    if (body->shape->type != ShapeType::CAPSULE || !capsule
        || !std::isfinite(capsule->GetHalfLength()) || capsule->GetHalfLength() < 0
        || !std::isfinite(capsule->GetRadius()) || capsule->GetRadius() <= 0)
        throw std::invalid_argument("Capsule collision requires a valid Capsule.");
    const double angle = body->orientation;
    return {center, {std::cos(angle), std::sin(angle)}, capsule->GetHalfLength(), capsule->GetRadius()};
}

// Clip a one- or two-point segment to normal.dot(p) <= offset.
std::uint8_t ClipToPlane(const std::array<Point, 2>& input, std::uint8_t count,
                         Point normal, double offset, std::array<Point, 2>& output) {
    if (count == 0) return 0;
    const double first = normal.dot(input[0]) - offset;
    const double second = count > 1 ? normal.dot(input[1]) - offset : first;
    std::uint8_t result = 0;
    if (first <= 0) output[result++] = input[0];
    if (count > 1 && second <= 0) output[result++] = input[1];
    if (count > 1 && ((first < 0 && second > 0) || (first > 0 && second < 0)))
        output[result++] = input[0] + (input[1] - input[0]) * (first / (first - second));
    return result;
}
// Keep the portion of [from, to] whose projection on axis lies in [low, high].
std::uint8_t ClipToSlab(Point from, Point to, bool single, Point axis, double low, double high,
                        std::array<Point, 2>& output) {
    std::array<Point, 2> first{}, input{{from, to}};
    const auto count = ClipToPlane(input, single ? 1 : 2, axis * -1, -low, first);
    const auto result = ClipToPlane(first, count, axis, high, output);
    std::sort(output.begin(), output.begin() + result,
              [&](Point a, Point b) { return axis.dot(a) < axis.dot(b); });
    return result;
}

// A speculative margin inflates the first rounded shape's radius. Builder
// subtracts it from every penetration and moves the inflated surface midpoint
// (shift) back to the midpoint of the real surfaces.
struct Builder {
    CollisionManifold manifold;
    Point origin, shift{0, 0};
    double margin = 0;
    void Add(Point local, double penetration, std::uint32_t feature) {
        if (manifold.contactCount == 2) return;
        manifold.contacts[manifold.contactCount++] = {CheckedVector(origin + local + shift),
            CheckedFloat(margin > 0 ? penetration - margin : std::max(penetration, 0.0)), feature};
    }
    // depth is the projected overlap along the chosen normal (with the margin
    // still included). It is the manifold penetration: clipping to the
    // reference extent can drop the deepest incident point, so the deepest
    // remaining contact may be shallower than the minimum translation.
    CollisionManifold Finish(Point normal, double depth) {
        if (manifold.contactCount == 0) return manifold;
        manifold.hasCollision = true;
        manifold.normal = CheckedVector(normal);
        manifold.penetration = CheckedFloat(margin > 0 ? depth - margin : std::max(depth, 0.0));
        for (std::uint8_t i = 0; i < manifold.contactCount; ++i)
            manifold.penetration = std::max(manifold.penetration, manifold.contacts[i].penetration);
        manifold.contactPoint = manifold.contacts[0].position;
        return manifold;
    }
};

// Rounded segment pair with the normal from a toward b.
CollisionManifold CollideRounded(RigidBody* bodyA, RigidBody* bodyB, std::uint32_t kind, double margin) {
    const Point origin = ToPoint(bodyA->position);
    Rounded a = MakeRounded(bodyA, origin);
    const Rounded b = MakeRounded(bodyB, origin);
    Builder builder;
    builder.manifold.A = bodyA; builder.manifold.B = bodyB; builder.origin = origin;
    builder.margin = margin;
    const Point offset = b.center - a.center;
    // A dimensionless allowance prefers spine faces on near ties, so parallel
    // contact produces two points; overlap itself is decided exactly.
    const double allowance = 2e-5 * std::min(a.radius, b.radius);
    a.radius += margin;
    const double radii = a.radius + b.radius;

    // Spines are disjoint iff some candidate axis separates their projections.
    double coreGap = -std::numeric_limits<double>::infinity();
    for (const Point axis : {Perp(a.axis), Perp(b.axis), a.axis, b.axis})
        coreGap = std::max(coreGap, std::abs(offset.dot(axis)) - a.Reach(axis) - b.Reach(axis));

    struct Candidate { Point normal; double penetration; int reference; bool negative; };
    std::vector<Candidate> candidates;
    const auto consider = [&](Point normal, int reference, bool negative) {
        candidates.push_back({normal, offset.dot(normal) * -1 + a.Reach(normal) + b.Reach(normal) + radii,
                              reference, negative});
    };
    if (a.halfLength > 0) { consider(Perp(a.axis), 0, false); consider(Perp(a.axis) * -1, 0, true); }
    if (b.halfLength > 0) { consider(Perp(b.axis), 1, false); consider(Perp(b.axis) * -1, 1, true); }
    Point closestA{}, closestB{};
    if (coreGap > 0) {
        double best = std::numeric_limits<double>::infinity();
        const auto test = [&](Point p, Point q) {
            const double distance = (q - p).norm();
            if (distance < best) { best = distance; closestA = p; closestB = q; }
        };
        for (int i = 0; i < (a.halfLength > 0 ? 2 : 1); ++i) {
            const Point end = a.halfLength > 0 ? a.End(i) : a.center;
            test(end, b.ClosestTo(end));
        }
        for (int j = 0; j < (b.halfLength > 0 ? 2 : 1); ++j) {
            const Point end = b.halfLength > 0 ? b.End(j) : b.center;
            test(a.ClosestTo(end), end);
        }
        if (best >= radii) return builder.manifold;
        consider((closestB - closestA) * (1 / best), 2, false);
    }
    if (candidates.empty()) consider({1, 0}, 3, false); // Coincident circles.
    double minimum = std::numeric_limits<double>::infinity();
    for (const auto& candidate : candidates) minimum = std::min(minimum, candidate.penetration);
    if (minimum <= 0) return builder.manifold;
    const Candidate* chosen = nullptr;
    for (const auto& candidate : candidates)
        if (candidate.penetration <= minimum + allowance) { chosen = &candidate; break; }
    const Point n = chosen->normal;
    builder.shift = n * (-0.5 * margin);

    if (chosen->reference <= 1) {
        // Clip the incident spine to the reference spine's extent.
        const bool referenceIsA = chosen->reference == 0;
        const Rounded& reference = referenceIsA ? a : b;
        const Rounded& incident = referenceIsA ? b : a;
        const Point towardIncident = referenceIsA ? n : n * -1;
        std::array<Point, 2> clipped{};
        const auto count = ClipToSlab(incident.End(0) - reference.center, incident.End(1) - reference.center,
            incident.halfLength == 0, reference.axis, -reference.halfLength, reference.halfLength, clipped);
        const std::uint32_t face = kind | FaceFeature | (referenceIsA ? 0 : SecondReference)
            | (chosen->negative ? NegativeFace : 0);
        for (std::uint8_t i = 0; i < count; ++i) {
            const Point q = clipped[i];
            const double separation = towardIncident.dot(q) - radii;
            if (separation > allowance) continue;
            const Point referenceSurface = reference.axis * reference.axis.dot(q) + towardIncident * reference.radius;
            const Point incidentSurface = q - towardIncident * incident.radius;
            builder.Add(reference.center + (referenceSurface + incidentSurface) * 0.5, -separation,
                        face | (i + 1u));
        }
        if (builder.manifold.contactCount > 0) return builder.Finish(n, chosen->penetration);
    }
    Point supportA = closestA, supportB = closestB;
    std::uint32_t feature = kind | 1u;
    if (chosen->reference != 2) {
        // Support points along the chosen normal.
        const double sa = Sign(a.axis.dot(n)), sb = Sign(b.axis.dot(n));
        supportA = a.center + a.axis * (a.halfLength * sa);
        supportB = b.center - b.axis * (b.halfLength * sb);
        feature |= static_cast<std::uint32_t>(sa + 1) << 4 | static_cast<std::uint32_t>(sb + 1) << 6;
    } else {
        feature |= 3u << 8;
    }
    builder.Add(((supportA + n * a.radius) + (supportB - n * b.radius)) * 0.5, chosen->penetration, feature);
    return builder.Finish(n, chosen->penetration);
}

struct PolygonGeometry {
    std::vector<Point> vertices, normals, tangents;
    std::vector<double> lengths;
    double minimumWidth = std::numeric_limits<double>::max();
};
PolygonGeometry MakePolygon(const RigidBody* body, Point origin) {
    ValidateTransform(body);
    const auto* polygon = dynamic_cast<const Polygon*>(body->shape.get());
    if (body->shape->type != ShapeType::POLYGON || !polygon || polygon->getVertices().size() < 3)
        throw std::invalid_argument("Capsule collision requires a valid Polygon.");
    const auto& local = polygon->getVertices();
    for (const auto& v : local)
        if (!std::isfinite(v.x) || !std::isfinite(v.y))
            throw std::invalid_argument("Polygon vertices must be finite.");
    const double c = std::cos(double(body->orientation)), s = std::sin(double(body->orientation));
    const auto rotate = [&](Point p) -> Point { return {c * p.x - s * p.y, s * p.x + c * p.y}; };
    const Point translation = ToPoint(body->position) - origin;
    const Point firstEdge = ToPoint(local[1]) - ToPoint(local[0]);
    const Point nextEdge = ToPoint(local[2]) - ToPoint(local[1]);
    const bool ccw = firstEdge.x * nextEdge.y - firstEdge.y * nextEdge.x > 0;
    PolygonGeometry result;
    for (std::size_t i = 0; i < local.size(); ++i) {
        result.vertices.push_back(translation + rotate(ToPoint(local[i])));
        // Form edges before adding a potentially large translation.
        const Point edge = rotate(ToPoint(local[(i + 1) % local.size()]) - ToPoint(local[i]));
        const double length = edge.norm();
        if (length == 0) throw std::invalid_argument("Polygon edges must be nonzero.");
        const Point tangent = edge * (1 / length);
        result.lengths.push_back(length);
        result.tangents.push_back(tangent);
        result.normals.push_back(ccw ? Point{tangent.y, -tangent.x} : Point{-tangent.y, tangent.x});
    }
    for (std::size_t i = 0; i < local.size(); ++i) {
        double low = std::numeric_limits<double>::max(), high = -low;
        for (const auto& v : local) {
            const double projection = result.normals[i].dot(rotate(ToPoint(v) - ToPoint(local[i])));
            low = std::min(low, projection); high = std::max(high, projection);
        }
        result.minimumWidth = std::min(result.minimumWidth, high - low);
    }
    return result;
}

// Capsule against convex polygon with the normal from capsule to polygon.
CollisionManifold CollideCapsulePolygon(RigidBody* capsuleBody, RigidBody* polygonBody, bool capsuleIsA,
                                        double margin) {
    const Point origin = ToPoint(capsuleBody->position);
    const Rounded capsule = MakeRounded(capsuleBody, origin);
    const PolygonGeometry polygon = MakePolygon(polygonBody, origin);
    Builder builder;
    builder.manifold.A = capsuleIsA ? capsuleBody : polygonBody;
    builder.manifold.B = capsuleIsA ? polygonBody : capsuleBody;
    builder.origin = origin;
    builder.margin = margin;
    const double h = capsule.halfLength;
    const Point u = capsule.axis, w = Perp(u);
    const double allowance = 1e-5 * std::min(polygon.minimumWidth, 2 * capsule.radius);
    const double r = capsule.radius + margin;
    const std::size_t count = polygon.vertices.size();
    const auto lowest = [&](Point n) {
        double low = std::numeric_limits<double>::infinity();
        std::size_t index = 0;
        for (std::size_t j = 0; j < count; ++j) {
            const double projection = polygon.vertices[j].dot(n);
            if (projection < low) { low = projection; index = j; }
        }
        return std::pair<double, std::size_t>{low, index};
    };

    // Faces of the polygon minus spine: polygon faces and the spine normals.
    double coreGap = -std::numeric_limits<double>::infinity();
    struct Candidate { Point normal; double penetration; int kind; std::size_t index; bool negative; };
    std::vector<Candidate> candidates;
    for (std::size_t i = 0; i < count; ++i) {
        const Point m = polygon.normals[i];
        const double face = m.dot(polygon.vertices[i]); // Signed face offset from the capsule center.
        const double reach = capsule.Reach(m);
        coreGap = std::max(coreGap, -face - reach);
        candidates.push_back({m * -1, reach + r + face, 0, i, false});
    }
    if (h > 0) {
        for (const bool negative : {false, true}) {
            const Point n = negative ? w * -1 : w;
            const double low = lowest(n).first;
            coreGap = std::max(coreGap, low);
            candidates.push_back({n, r - low, 1, 0, negative});
        }
    }
    Point closestSpine{}, closestPolygon{};
    std::uint32_t closestFeature = 0;
    if (coreGap > 0) {
        double best = std::numeric_limits<double>::infinity();
        const auto test = [&](Point p, Point q, std::uint32_t feature) {
            const double distance = (q - p).norm();
            if (distance < best) { best = distance; closestSpine = p; closestPolygon = q; closestFeature = feature; }
        };
        for (int e = 0; e < (h > 0 ? 2 : 1); ++e) {
            const Point end = h > 0 ? capsule.End(e) : capsule.center;
            for (std::size_t i = 0; i < count; ++i) {
                const Point from = polygon.vertices[i];
                const double along = std::clamp(polygon.tangents[i].dot(end - from), 0.0, polygon.lengths[i]);
                test(end, from + polygon.tangents[i] * along,
                     static_cast<std::uint32_t>(e + 1) << 18 | static_cast<std::uint32_t>(i & 0xffffu) << 2);
            }
        }
        for (std::size_t j = 0; j < count; ++j)
            test(capsule.ClosestTo(polygon.vertices[j]), polygon.vertices[j],
                 3u << 18 | static_cast<std::uint32_t>(j & 0xffffu) << 2);
        if (best >= r) return builder.manifold;
        candidates.push_back({(closestPolygon - closestSpine) * (1 / best), r - best, 2, 0, false});
    }
    double minimum = std::numeric_limits<double>::infinity();
    for (const auto& candidate : candidates) minimum = std::min(minimum, candidate.penetration);
    if (minimum <= 0) return builder.manifold;
    const Candidate* chosen = nullptr;
    for (const auto& candidate : candidates)
        if (candidate.penetration <= minimum + allowance) { chosen = &candidate; break; }
    const Point n = chosen->normal;
    const Point normalAB = capsuleIsA ? n : n * -1;
    builder.shift = n * (-0.5 * margin);

    if (chosen->kind == 0) {
        // Polygon reference face: clip the spine to the face's side planes.
        const std::size_t i = chosen->index;
        const Point m = polygon.normals[i], t = polygon.tangents[i], from = polygon.vertices[i];
        std::array<Point, 2> clipped{};
        const auto clippedCount = ClipToSlab(capsule.End(0) - from, capsule.End(1) - from, h == 0,
                                             t, 0, polygon.lengths[i], clipped);
        const std::uint32_t face = PolygonCapsuleKind | FaceFeature | static_cast<std::uint32_t>(i & 0xffffu) << 2;
        for (std::uint8_t k = 0; k < clippedCount; ++k) {
            const Point q = clipped[k];
            const double separation = m.dot(q) - r;
            if (separation > allowance) continue;
            const Point faceSurface = q - m * m.dot(q), capsuleSurface = q - m * r;
            builder.Add(from + (faceSurface + capsuleSurface) * 0.5, -separation, face | (k + 1u));
        }
    } else if (chosen->kind == 1) {
        // Capsule reference side: clip the most anti-parallel polygon edge.
        std::size_t incident = 0;
        double alignment = std::numeric_limits<double>::infinity();
        for (std::size_t j = 0; j < count; ++j)
            if (polygon.normals[j].dot(n) < alignment) { alignment = polygon.normals[j].dot(n); incident = j; }
        std::array<Point, 2> clipped{};
        const auto clippedCount = ClipToSlab(polygon.vertices[incident], polygon.vertices[(incident + 1) % count],
                                             false, u, -h, h, clipped);
        const std::uint32_t face = PolygonCapsuleKind | FaceFeature | SecondReference
            | (chosen->negative ? NegativeFace : 0) | static_cast<std::uint32_t>(incident & 0xffffu) << 2;
        for (std::uint8_t k = 0; k < clippedCount; ++k) {
            const Point q = clipped[k];
            const double separation = n.dot(q) - r;
            if (separation > allowance) continue;
            const Point capsuleSurface = u * u.dot(q) + n * r;
            builder.Add((q + capsuleSurface) * 0.5, -separation, face | (k + 1u));
        }
    }
    if (builder.manifold.contactCount > 0) return builder.Finish(normalAB, chosen->penetration);

    Point spine = closestSpine, surface = closestPolygon;
    std::uint32_t feature = PolygonCapsuleKind | closestFeature | 1u;
    if (chosen->kind != 2) {
        const double side = Sign(u.dot(n));
        const auto support = lowest(n);
        spine = u * (h * side);
        surface = polygon.vertices[support.second];
        feature = PolygonCapsuleKind | static_cast<std::uint32_t>(side + 1) << 18
            | static_cast<std::uint32_t>(support.second & 0xffffu) << 2 | 1u;
    }
    builder.Add(((spine + n * r) + surface) * 0.5, chosen->penetration, feature);
    return builder.Finish(normalAB, chosen->penetration);
}
} // namespace

CollisionManifold CollisionCircleCapsule(RigidBody* a, RigidBody* b, float margin) {
    ValidateTransform(a); ValidateTransform(b);
    if (a == b) throw std::invalid_argument("Capsule collision requires two distinct bodies.");
    if (a->shape->type == ShapeType::CIRCLE) return CollideRounded(a, b, CircleCapsuleKind, margin);
    CollisionManifold manifold = CollideRounded(b, a, CircleCapsuleKind, margin);
    std::swap(manifold.A, manifold.B);
    manifold.normal = manifold.normal * -1;
    return manifold;
}

CollisionManifold CollisionPolygonCapsule(RigidBody* a, RigidBody* b, float margin) {
    ValidateTransform(a); ValidateTransform(b);
    if (a == b) throw std::invalid_argument("Capsule collision requires two distinct bodies.");
    const bool capsuleIsA = a->shape->type == ShapeType::CAPSULE;
    return capsuleIsA ? CollideCapsulePolygon(a, b, true, margin) : CollideCapsulePolygon(b, a, false, margin);
}

CollisionManifold CollisionCapsuleCapsule(RigidBody* a, RigidBody* b, float margin) {
    ValidateTransform(a); ValidateTransform(b);
    if (a == b) throw std::invalid_argument("Capsule collision requires two distinct bodies.");
    if (a->shape->type != ShapeType::CAPSULE || b->shape->type != ShapeType::CAPSULE)
        throw std::invalid_argument("Capsule collision requires two Capsule bodies.");
    // Canonical ID order makes exact feature ties independent of argument order.
    if (a->GetId() <= b->GetId()) return CollideRounded(a, b, CapsuleCapsuleKind, margin);
    CollisionManifold manifold = CollideRounded(b, a, CapsuleCapsuleKind, margin);
    std::swap(manifold.A, manifold.B);
    manifold.normal = manifold.normal * -1;
    return manifold;
}
} // namespace PhysicsEngine

#include "physics/core/collisions/narrow_phase/collision_polygon_polygon.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

namespace PhysicsEngine {
namespace {
struct Point {
    double x, y;
    Point operator+(Point p) const { return {x + p.x, y + p.y}; }
    Point operator-(Point p) const { return {x - p.x, y - p.y}; }
    Point operator*(double s) const { return {x * s, y * s}; }
    double dot(Point p) const { return x * p.x + y * p.y; }
};
Point ToPoint(Vector2 p) { return {p.x, p.y}; }
float CheckedFloat(double value) {
    if (!std::isfinite(value) || std::abs(value) > std::numeric_limits<float>::max())
        throw std::overflow_error("Polygon manifold exceeds finite float range.");
    return static_cast<float>(value);
}
Vector2 CheckedVector(Point p) { return {CheckedFloat(p.x), CheckedFloat(p.y)}; }

// Keep translation, original local origin and local offsets separate. In
// particular, subtracting two huge world vertices must not erase a small edge.
struct Geometry {
    Point position, localOrigin;
    std::vector<Point> offsets, normals, tangents;
    std::vector<double> edgeLengths;
    double minimumWidth = std::numeric_limits<double>::max();
};
Geometry GetGeometry(const RigidBody* body) {
    if (!body || !body->shape)
        throw std::invalid_argument("Polygon collision requires two polygon bodies.");
    const auto* polygon = dynamic_cast<const Polygon*>(body->shape.get());
    if (!polygon || !std::isfinite(body->position.x) || !std::isfinite(body->position.y)
        || !std::isfinite(body->orientation))
        throw std::invalid_argument("Polygon collision requires polygons and finite transforms.");
    const auto& vertices = polygon->getVertices();
    // Also validate derived Polygon implementations whose protected outline
    // might have changed after construction. Work remains O(N^2), like SAT.
    const Polygon validated(vertices);
    const double cosine = std::cos(static_cast<double>(body->orientation));
    const double sine = std::sin(static_cast<double>(body->orientation));
    const auto rotate = [&](Point p) -> Point {
        return {cosine * p.x - sine * p.y, sine * p.x + cosine * p.y};
    };
    Geometry result;
    result.position = ToPoint(body->position);
    const Point origin = ToPoint(vertices.front());
    result.localOrigin = rotate(origin);
    const Point firstEdge = ToPoint(vertices[1]) - origin;
    const Point nextEdge = ToPoint(vertices[2]) - ToPoint(vertices[1]);
    const bool ccw = firstEdge.x * nextEdge.y - firstEdge.y * nextEdge.x > 0;
    for (std::size_t i = 0; i < vertices.size(); ++i) {
        result.offsets.push_back(rotate(ToPoint(vertices[i]) - origin));
        const Point edge = rotate(ToPoint(vertices[(i + 1) % vertices.size()]) - ToPoint(vertices[i]));
        const Point tangent = edge * (1 / std::hypot(edge.x, edge.y));
        result.edgeLengths.push_back(std::hypot(edge.x, edge.y));
        result.tangents.push_back(tangent);
        result.normals.push_back(ccw ? Point{tangent.y, -tangent.x} : Point{-tangent.y, tangent.x});
    }
    for (std::size_t i = 0; i < vertices.size(); ++i) {
        const Point localEdge = ToPoint(vertices[(i + 1) % vertices.size()]) - ToPoint(vertices[i]);
        const Point localNormal = Point{-localEdge.y, localEdge.x} * (1 / std::hypot(localEdge.x, localEdge.y));
        double minimum = std::numeric_limits<double>::max(), maximum = -minimum;
        for (const Vector2& vertex : vertices) {
            const double projection = localNormal.dot(ToPoint(vertex) - ToPoint(vertices[i]));
            minimum = std::min(minimum, projection);
            maximum = std::max(maximum, projection);
        }
        result.minimumWidth = std::min(result.minimumWidth, maximum - minimum);
    }
    return result;
}
Point Difference(const Geometry& a, std::size_t i, const Geometry& b, std::size_t j) {
    return (a.position - b.position) + (a.localOrigin - b.localOrigin)
        + (a.offsets[i] - b.offsets[j]);
}
struct FaceSeparation {
    double separation = -std::numeric_limits<double>::max();
    std::size_t faceIndex = 0;
};
FaceSeparation FindMaximumSeparation(const Geometry& reference, const Geometry& incident) {
    FaceSeparation best;
    for (std::size_t face = 0; face < reference.offsets.size(); ++face) {
        double minimum = std::numeric_limits<double>::max();
        for (std::size_t i = 0; i < incident.offsets.size(); ++i)
            minimum = std::min(minimum, reference.normals[face].dot(Difference(incident, i, reference, face)));
        if (minimum > best.separation) best = {minimum, face};
    }
    return best;
}
std::size_t FindIncidentFace(const Geometry& incident, Point normal) {
    double minimum = std::numeric_limits<double>::max();
    std::size_t result = 0;
    for (std::size_t i = 0; i < incident.normals.size(); ++i) {
        const double alignment = incident.normals[i].dot(normal);
        if (alignment < minimum) { minimum = alignment; result = i; }
    }
    return result;
}
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
std::uint32_t MakeFeatureId(bool referenceIsB, std::size_t referenceFace,
                            std::size_t incidentFace, std::uint8_t ordinal) {
    return 0x80000000u | (static_cast<std::uint32_t>(referenceIsB) << 30)
        | (static_cast<std::uint32_t>(referenceFace & 0x3fffu) << 16)
        | (static_cast<std::uint32_t>(incidentFace & 0x3fffu) << 2)
        | static_cast<std::uint32_t>(ordinal + 1);
}
// Closest points of two separated convex polygons, relative to A's position.
struct Closest {
    double distance = std::numeric_limits<double>::max();
    Point onA{0, 0}, onB{0, 0};
    std::size_t vertexA = 0, vertexB = 0;
    bool vertexVertex = false;
};
Closest FindClosest(const Geometry& a, const Geometry& b) {
    Closest best;
    const auto vertex = [&](const Geometry& g, std::size_t i) {
        return (g.position - a.position) + g.localOrigin + g.offsets[i];
    };
    const auto test = [&](const Geometry& edges, const Geometry& points, bool edgesOnA) {
        for (std::size_t e = 0; e < edges.offsets.size(); ++e) {
            const Point from = vertex(edges, e);
            for (std::size_t v = 0; v < points.offsets.size(); ++v) {
                const Point q = vertex(points, v);
                const double raw = edges.tangents[e].dot(q - from);
                const double along = std::clamp(raw, 0.0, edges.edgeLengths[e]);
                const Point onEdge = from + edges.tangents[e] * along;
                const Point gap = q - onEdge;
                const double distance = std::hypot(gap.x, gap.y);
                const bool atVertex = along == 0 || along == edges.edgeLengths[e];
                // On ties prefer a face feature, which keeps two-point manifolds.
                if (distance < best.distance || (distance == best.distance && !atVertex)) {
                    const std::size_t edgeVertex = along * 2 < edges.edgeLengths[e] ? e : (e + 1) % edges.offsets.size();
                    best.distance = distance;
                    best.onA = edgesOnA ? onEdge : q;
                    best.onB = edgesOnA ? q : onEdge;
                    best.vertexA = edgesOnA ? edgeVertex : v;
                    best.vertexB = edgesOnA ? v : edgeVertex;
                    best.vertexVertex = atVertex;
                }
            }
        }
    };
    test(a, b, true);
    test(b, a, false);
    return best;
}
CollisionManifold VertexManifold(RigidBody* a, RigidBody* b, const Geometry& geometryA, const Closest& closest) {
    CollisionManifold manifold;
    manifold.A = a; manifold.B = b;
    const Point direction = (closest.onB - closest.onA) * (1 / closest.distance);
    manifold.hasCollision = true;
    manifold.contactCount = 1;
    manifold.normal = CheckedVector(direction);
    manifold.penetration = CheckedFloat(-closest.distance);
    // Ordinal bits 3 mark a vertex pair; clipped face points use 1 and 2.
    manifold.contacts[0] = {CheckedVector(geometryA.position + (closest.onA + closest.onB) * 0.5),
        manifold.penetration, 0x80000003u | (static_cast<std::uint32_t>(closest.vertexA & 0x3fffu) << 16)
            | (static_cast<std::uint32_t>(closest.vertexB & 0x3fffu) << 2)};
    manifold.contactPoint = manifold.contacts[0].position;
    return manifold;
}
CollisionManifold ComputeCanonicalManifold(RigidBody* a, RigidBody* b, double margin) {
    CollisionManifold manifold;
    manifold.A = a; manifold.B = b;
    const Geometry geometryA = GetGeometry(a), geometryB = GetGeometry(b);
    const FaceSeparation separationA = FindMaximumSeparation(geometryA, geometryB);
    if (separationA.separation >= margin) return manifold;
    const FaceSeparation separationB = FindMaximumSeparation(geometryB, geometryA);
    if (separationB.separation >= margin) return manifold;
    // Dimensionless hysteresis/contact allowance, bounded by the narrower
    // polygon's minimum face-normal thickness; exact SAT still rejects touching.
    const double allowance = 1e-5 * std::min(geometryA.minimumWidth, geometryB.minimumWidth);
    const bool referenceIsB = separationB.separation > separationA.separation + allowance;
    const Geometry& reference = referenceIsB ? geometryB : geometryA;
    const Geometry& incident = referenceIsB ? geometryA : geometryB;
    const std::size_t face = referenceIsB ? separationB.faceIndex : separationA.faceIndex;
    const Point normal = reference.normals[face], side = reference.tangents[face];
    const std::size_t incidentFace = FindIncidentFace(incident, normal);
    // Clip in the reference face's frame, without large world-plane offsets.
    const std::array<Point, 2> edge{{Difference(incident, incidentFace, reference, face),
        Difference(incident, (incidentFace + 1) % incident.offsets.size(), reference, face)}};
    std::array<Point, 2> first{}, second{};
    const auto firstCount = ClipToPlane(edge, 2, side * -1, 0, first);
    const double length = reference.edgeLengths[face];
    const auto count = ClipToPlane(first, firstCount, side, length, second);
    // Separated speculative pairs: face separation is only a lower bound on
    // the distance, and a closest vertex beyond the reference face is clipped
    // away. When the true gap exceeds the face separation the approach is
    // vertex to vertex and gets one point along the true gap.
    Closest closest;
    const double faceGap = std::max(separationA.separation, separationB.separation);
    const bool separated = margin > 0 && faceGap > 0;
    if (separated) {
        closest = FindClosest(geometryA, geometryB);
        if (closest.distance >= margin) return manifold;
        if (closest.vertexVertex && closest.distance > faceGap + allowance)
            return VertexManifold(a, b, geometryA, closest);
    }
    std::sort(second.begin(), second.begin() + count, [&](Point left, Point right) {
        return side.dot(left) < side.dot(right);
    });
    for (std::uint8_t i = 0; i < count; ++i) {
        const double separation = normal.dot(second[i]);
        if (separation <= allowance || separation < margin) {
            const auto index = manifold.contactCount++;
            manifold.contacts[index] = {CheckedVector(reference.position + reference.localOrigin
                + reference.offsets[face] + second[i]),
                CheckedFloat(margin > 0 ? -separation : std::max(-separation, 0.0)),
                MakeFeatureId(referenceIsB, face, incidentFace, index)};
        }
    }
    if (manifold.contactCount == 0)
        return separated && closest.distance > 0 ? VertexManifold(a, b, geometryA, closest) : manifold;
    manifold.hasCollision = true;
    manifold.normal = CheckedVector(referenceIsB ? normal * -1 : normal);
    manifold.penetration = manifold.contacts[0].penetration;
    for (std::uint8_t i = 1; i < manifold.contactCount; ++i)
        manifold.penetration = std::max(manifold.penetration, manifold.contacts[i].penetration);
    manifold.contactPoint = manifold.contacts[0].position;
    return manifold;
}
} // namespace
CollisionManifold CollisionPolygonPolygon(RigidBody* a, RigidBody* b, float margin) {
    if (!a || !b) throw std::invalid_argument("Polygon collision requires two bodies.");
    if (a->GetId() <= b->GetId()) return ComputeCanonicalManifold(a, b, margin);
    CollisionManifold manifold = ComputeCanonicalManifold(b, a, margin);
    std::swap(manifold.A, manifold.B);
    manifold.normal = manifold.normal * -1;
    return manifold;
}
} // namespace PhysicsEngine

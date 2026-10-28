#include "physics/core/collisions/narrow_phase/collision_polygon_polygon.h"
#include "../../trig_cache.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
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

// Inline storage for the usual small outlines; larger polygons spill to the
// heap. Narrow phase runs per candidate pair per step, so small polygons must
// not allocate.
constexpr std::size_t kInlineVertices = 8;
template <class T, std::size_t N = kInlineVertices>
class SmallVector {
public:
    void push_back(const T& value) {
        if (count < N) {
            local[count] = value;
        } else {
            if (count == N) heap.assign(local, local + N);
            heap.push_back(value);
        }
        ++count;
    }
    std::size_t size() const { return count; }
    const T& operator[](std::size_t i) const { return count <= N ? local[i] : heap[i]; }

private:
    T local[N];
    std::vector<T> heap;
    std::size_t count = 0;
};

// Keep translation, original local origin and local offsets separate. In
// particular, subtracting two huge world vertices must not erase a small edge.
struct Geometry {
    Point position, localOrigin;
    SmallVector<Point> offsets, normals, tangents;
    SmallVector<double> edgeLengths;
    const std::vector<Vector2>* vertices = nullptr;
    double minimumWidth = 0; // valid when hasMinimumWidth
    bool hasMinimumWidth = false;
};
// Narrowest face-normal thickness of the local outline. Only pairs that pass
// both separating-axis tests need it, so it is computed after those exits.
double MinimumWidth(const Geometry& geometry) {
    const auto& vertices = *geometry.vertices;
    double result = std::numeric_limits<double>::max();
    for (std::size_t i = 0; i < vertices.size(); ++i) {
        const Point localEdge = ToPoint(vertices[(i + 1) % vertices.size()]) - ToPoint(vertices[i]);
        const Point localNormal = Point{-localEdge.y, localEdge.x} * (1 / std::hypot(localEdge.x, localEdge.y));
        double minimum = std::numeric_limits<double>::max(), maximum = -minimum;
        for (const Vector2& vertex : vertices) {
            const double projection = localNormal.dot(ToPoint(vertex) - ToPoint(vertices[i]));
            minimum = std::min(minimum, projection);
            maximum = std::max(maximum, projection);
        }
        result = std::min(result, maximum - minimum);
    }
    return result;
}
double WidthOf(const Geometry& geometry) {
    return geometry.hasMinimumWidth ? geometry.minimumWidth : MinimumWidth(geometry);
}
Geometry GetGeometry(const RigidBody* body) {
    if (!body || !body->shape)
        throw std::invalid_argument("Polygon collision requires two polygon bodies.");
    const auto* polygon = dynamic_cast<const Polygon*>(body->shape.get());
    if (!polygon || !std::isfinite(body->position.x) || !std::isfinite(body->position.y)
        || !std::isfinite(body->orientation))
        throw std::invalid_argument("Polygon collision requires polygons and finite transforms.");
    const auto& vertices = polygon->getVertices();
    // Also validate derived Polygon implementations whose protected outline
    // might have changed after construction. Work remains O(N^2), like SAT,
    // but the outline is checked in place rather than copied into a Polygon.
    // Each body meets several neighbours per detection pass, and twice per
    // step, with the same outline and orientation. Reuse the rotated outline
    // when both match bit for bit: validation and the rotation are pure
    // functions of exactly those inputs, so a hit returns what this call
    // would compute.
    struct Slot {
        const Polygon* polygon = nullptr;
        std::uint32_t orientationBits = 0;
        std::size_t count = 0;
        Vector2 vertices[kInlineVertices];
        Geometry geometry;
    };
    constexpr unsigned kSlotBits = 10;
    thread_local std::unique_ptr<Slot[]> slots;
    std::uint32_t orientationBits;
    std::memcpy(&orientationBits, &body->orientation, sizeof orientationBits);
    const bool cacheable = vertices.size() <= kInlineVertices;
    Slot* slot = nullptr;
    if (cacheable) {
        if (!slots) slots.reset(new Slot[std::size_t{1} << kSlotBits]);
        const std::uint64_t key = reinterpret_cast<std::uintptr_t>(polygon) ^ (std::uint64_t{orientationBits} << 32);
        slot = &slots[(key * 0x9E3779B97F4A7C15ull) >> (64 - kSlotBits)];
        if (slot->polygon == polygon && slot->orientationBits == orientationBits && slot->count == vertices.size()
            && std::memcmp(slot->vertices, vertices.data(), vertices.size() * sizeof(Vector2)) == 0) {
            Geometry result = slot->geometry;
            result.vertices = &vertices;
            result.position = ToPoint(body->position);
            return result;
        }
    }
    Polygon::ValidateOutline(vertices);
    const Detail::CosSin rotation = Detail::CachedCosSin(static_cast<double>(body->orientation));
    const double cosine = rotation.cosine, sine = rotation.sine;
    const auto rotate = [&](Point p) -> Point {
        return {cosine * p.x - sine * p.y, sine * p.x + cosine * p.y};
    };
    Geometry result;
    result.vertices = &vertices;
    const Point origin = ToPoint(vertices.front());
    result.localOrigin = rotate(origin);
    const Point firstEdge = ToPoint(vertices[1]) - origin;
    const Point nextEdge = ToPoint(vertices[2]) - ToPoint(vertices[1]);
    const bool ccw = firstEdge.x * nextEdge.y - firstEdge.y * nextEdge.x > 0;
    for (std::size_t i = 0; i < vertices.size(); ++i) {
        result.offsets.push_back(rotate(ToPoint(vertices[i]) - origin));
        const Point edge = rotate(ToPoint(vertices[(i + 1) % vertices.size()]) - ToPoint(vertices[i]));
        const double length = std::hypot(edge.x, edge.y);
        const Point tangent = edge * (1 / length);
        result.edgeLengths.push_back(length);
        result.tangents.push_back(tangent);
        result.normals.push_back(ccw ? Point{tangent.y, -tangent.x} : Point{-tangent.y, tangent.x});
    }
    if (slot) {
        result.minimumWidth = MinimumWidth(result);
        result.hasMinimumWidth = true;
        slot->polygon = polygon;
        slot->orientationBits = orientationBits;
        slot->count = vertices.size();
        std::memcpy(slot->vertices, vertices.data(), vertices.size() * sizeof(Vector2));
        slot->geometry = result;
    }
    result.position = ToPoint(body->position);
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
// Face point id from A's face, B's face and the point's order along A's
// face. It does not record which polygon was the reference: two resting
// parallel faces swap reference as their tilt crosses zero, and an id that
// changed with it would drop the warm-start impulses at every swap.
std::uint32_t MakeFeatureId(std::size_t faceA, std::size_t faceB, std::uint8_t ordinal) {
    return 0x80000000u
        | (static_cast<std::uint32_t>(faceA & 0x3fffu) << 16)
        | (static_cast<std::uint32_t>(faceB & 0x3fffu) << 2)
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
    const double allowance = 1e-5 * std::min(WidthOf(geometryA), WidthOf(geometryB));
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
    // vertex to vertex and gets one point along the true gap. The clipped
    // vertex can also sit just past the side plane, with the true gap within
    // the allowance of the face separation, while the clip point that
    // replaces it lies far up a steep incident edge and overstates the gap
    // without bound as the edge turns toward the side plane. The face points
    // are only kept when the nearest of them reports the true gap.
    Closest closest;
    const double faceGap = std::max(separationA.separation, separationB.separation);
    const bool separated = margin > 0 && faceGap > 0;
    if (separated) {
        closest = FindClosest(geometryA, geometryB);
        if (closest.distance >= margin) return manifold;
        double nearestFace = std::numeric_limits<double>::infinity();
        for (std::uint8_t i = 0; i < count; ++i) nearestFace = std::min(nearestFace, normal.dot(second[i]));
        if (closest.vertexVertex
            && (closest.distance > faceGap + allowance || (count > 0 && nearestFace > closest.distance + allowance)))
            return VertexManifold(a, b, geometryA, closest);
    }
    std::sort(second.begin(), second.begin() + count, [&](Point left, Point right) {
        return side.dot(left) < side.dot(right);
    });
    const std::size_t faceA = referenceIsB ? incidentFace : face, faceB = referenceIsB ? face : incidentFace;
    for (std::uint8_t i = 0; i < count; ++i) {
        const double separation = normal.dot(second[i]);
        if (separation <= allowance || separation < margin) {
            const auto index = manifold.contactCount++;
            // Points are sorted along the reference face; B's facing side runs
            // the other way along A's.
            const auto ordinal = static_cast<std::uint8_t>(referenceIsB ? count - 1 - i : i);
            manifold.contacts[index] = {CheckedVector(reference.position + reference.localOrigin
                + reference.offsets[face] + second[i]),
                CheckedFloat(margin > 0 ? -separation : std::max(-separation, 0.0)),
                MakeFeatureId(faceA, faceB, ordinal)};
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

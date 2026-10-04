#include "physics/core/spatial_queries.h"
#include "physics/core/rigidbody.h"
#include "physics/core/world.h"
#include "engine.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace PhysicsEngine {
namespace {
struct DVector {
    double x, y;
    DVector operator+(DVector b) const { return {x + b.x, y + b.y}; }
    DVector operator-(DVector b) const { return {x - b.x, y - b.y}; }
    DVector operator*(double t) const { return {x * t, y * t}; }
};
DVector ToDouble(Vector2 p) { return {p.x, p.y}; }
Vector2 ToFloat(DVector p) {
    const double maximum = std::numeric_limits<float>::max();
    if (!std::isfinite(p.x) || !std::isfinite(p.y)
        || std::abs(p.x) > maximum || std::abs(p.y) > maximum)
        throw std::overflow_error("Spatial query result exceeds finite Vector2 range.");
    return {static_cast<float>(p.x), static_cast<float>(p.y)};
}
double Dot(DVector a, DVector b) { return a.x * b.x + a.y * b.y; }
double Cross(DVector a, DVector b) { return a.x * b.y - a.y * b.x; }
void Validate(Vector2 p) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y))
        throw std::invalid_argument("Spatial query coordinates must be finite.");
}
void ValidateRadius(float radius) {
    if (!std::isfinite(radius) || radius < 0)
        throw std::invalid_argument("Query circle radius must be finite and nonnegative.");
}
struct Transform {
    DVector position;
    double cosine, sine;
    Transform(Vector2 p, float angle) : position(ToDouble(p)) {
        Validate(p);
        if (!std::isfinite(angle))
            throw std::invalid_argument("Spatial query orientation must be finite.");
        cosine = std::cos(static_cast<double>(angle));
        sine = std::sin(static_cast<double>(angle));
    }
    DVector LocalDouble(DVector p) const {
        const DVector relative = p - position;
        return {cosine * relative.x + sine * relative.y,
            -sine * relative.x + cosine * relative.y};
    }
    DVector Local(Vector2 p) const { return LocalDouble(ToDouble(p)); }
    DVector Rotate(DVector p) const {
        return {cosine * p.x - sine * p.y, sine * p.x + cosine * p.y};
    }
    Vector2 World(DVector p) const { return ToFloat(Rotate(p) + position); }
};
const Circle& AsCircle(const Shape& shape) {
    const auto* circle = dynamic_cast<const Circle*>(&shape);
    if (!circle || !std::isfinite(circle->GetRadius()) || circle->GetRadius() <= 0)
        throw std::invalid_argument("Spatial queries require a valid Circle or convex Polygon.");
    return *circle;
}
const Polygon& AsPolygon(const Shape& shape) {
    const auto* polygon = dynamic_cast<const Polygon*>(&shape);
    if (!polygon || polygon->getVertices().size() < 3)
        throw std::invalid_argument("Spatial queries require a valid Circle or convex Polygon.");
    return *polygon;
}
// Polygon's constructor guarantees strict convexity in either winding.
double Winding(const Polygon& polygon) {
    const auto& v = polygon.getVertices();
    for (const auto& point : v) Validate(point);
    return Cross(ToDouble(v[1]) - ToDouble(v[0]), ToDouble(v[2]) - ToDouble(v[0])) > 0 ? 1 : -1;
}
DVector Outward(DVector a, DVector b, double winding) {
    const DVector edge = b - a;
    const double length = std::hypot(edge.x, edge.y);
    return {winding * edge.y / length, -winding * edge.x / length};
}
bool PolygonContains(const Polygon& polygon, DVector point) {
    const auto& vertices = polygon.getVertices();
    const double winding = Winding(polygon);
    for (std::size_t i = 0; i < vertices.size(); ++i) {
        const DVector a = ToDouble(vertices[i]);
        const DVector b = ToDouble(vertices[(i + 1) % vertices.size()]);
        if (winding * Cross(b - a, point - a) < 0) return false;
    }
    return true;
}
bool PolygonCircle(const Polygon& polygon, DVector center, double radius) {
    if (PolygonContains(polygon, center)) return true;
    const auto& vertices = polygon.getVertices();
    for (std::size_t i = 0; i < vertices.size(); ++i) {
        const DVector a = ToDouble(vertices[i]);
        const DVector edge = ToDouble(vertices[(i + 1) % vertices.size()]) - a;
        const double fraction = std::clamp(Dot(center - a, edge) / Dot(edge, edge), 0.0, 1.0);
        const DVector separation = center - (a + edge * fraction);
        if (std::hypot(separation.x, separation.y) <= radius) return true;
    }
    return false;
}
std::optional<RayHit> CircleRay(const Circle& circle, DVector position,
    Vector2 start, Vector2 end) {
    const double radius = circle.GetRadius();
    const DVector localStart = ToDouble(start) - position;
    const DVector localEnd = ToDouble(end) - position;
    if (Dot(localStart, localStart) <= radius * radius)
        return RayHit{0, start, {}};
    const DVector motion = ToDouble(end) - ToDouble(start);
    const double length = std::hypot(motion.x, motion.y);
    if (length == 0) return std::nullopt;
    const DVector direction = motion * (1 / length);
    // Geometric line/disk intersection avoids subtracting enormous squared
    // terms in the quadratic discriminant. Construct the point on the circle
    // itself: start + fraction * motion loses a small radius on enormous rays.
    const double perpendicular = Cross(direction, localStart);
    const double distance = std::abs(perpendicular);
    if (distance > radius) return std::nullopt;
    const double halfChord = std::sqrt((radius - distance) * (radius + distance));
    const double entry = -halfChord;
    const double startProjection = Dot(localStart, direction);
    const double endProjection = Dot(localEnd, direction);
    // Endpoint projections preserve small offsets that the total length loses.
    if (entry < startProjection || entry > endProjection) return std::nullopt;
    const DVector closest = DVector{-direction.y, direction.x} * perpendicular;
    const DVector localPoint = closest + direction * entry;
    return RayHit{std::clamp((entry - startProjection) / length, 0.0, 1.0),
        ToFloat(localPoint + position), ToFloat(localPoint * (1 / radius))};
}
std::optional<RayHit> PolygonRay(const Polygon& polygon, const Transform& transform,
    Vector2 start, Vector2 end) {
    const DVector localStart = transform.Local(start);
    if (PolygonContains(polygon, localStart)) return RayHit{0, start, {}};
    const DVector motion = ToDouble(end) - ToDouble(start);
    const double length = std::hypot(motion.x, motion.y);
    if (length == 0) return std::nullopt;
    const DVector worldDirection = motion * (1 / length);
    DVector direction{transform.cosine * worldDirection.x + transform.sine * worldDirection.y,
        -transform.sine * worldDirection.x + transform.cosine * worldDirection.y};
    const double directionLength = std::hypot(direction.x, direction.y);
    direction = direction * (1 / directionLength);
    // Anchor the infinite ray at a coordinate axis before rotating it. Choosing
    // the larger direction component bounds the slope and preserves simple
    // axis/diagonal lines without normalizing a tangent's line offset.
    DVector worldReference;
    if (std::abs(motion.x) >= std::abs(motion.y))
        worldReference = {0, std::fma(-static_cast<double>(start.x), motion.y / motion.x, start.y)};
    else
        worldReference = {std::fma(-static_cast<double>(start.y), motion.x / motion.y, start.x), 0};
    const DVector reference = transform.LocalDouble(worldReference);
    const auto& vertices = polygon.getVertices();
    const double winding = Winding(polygon);
    double entry = -std::numeric_limits<double>::infinity();
    double exit = std::numeric_limits<double>::infinity();
    DVector entryNormal{};
    for (std::size_t i = 0; i < vertices.size(); ++i) {
        const DVector vertex = ToDouble(vertices[i]);
        const DVector normal = Outward(vertex, ToDouble(vertices[(i + 1) % vertices.size()]), winding);
        const double offset = Dot(normal, vertex - reference);
        const double speed = Dot(normal, direction);
        if (speed == 0) {
            if (offset < 0) return std::nullopt;
            continue;
        }
        // Clip signed distance along the line near the polygon, rather than
        // endpoint fractions: entry fractions at different faces may round to
        // the same value for a ray from -FLT_MAX to FLT_MAX.
        const double distance = offset / speed;
        if (speed < 0) {
            if (distance > entry) {
                entry = distance;
                entryNormal = normal;
            }
        } else {
            exit = std::min(exit, distance);
        }
        if (entry > exit) return std::nullopt;
    }
    const double startProjection = Dot(worldDirection, ToDouble(start) - worldReference) * directionLength;
    const double endProjection = Dot(worldDirection, ToDouble(end) - worldReference) * directionLength;
    if (entry < startProjection || entry > endProjection) return std::nullopt;
    const DVector point = reference + direction * entry;
    return RayHit{std::clamp((entry - startProjection) / (length * directionLength), 0.0, 1.0),
        transform.World(point), ToFloat(transform.Rotate(entryNormal))};
}
bool Matches(const RigidBody& body, QueryFilter filter) {
    return (body.GetCollisionCategoryBits() & filter.maskBits) != 0
        && (filter.categoryBits & body.GetCollisionMaskBits()) != 0;
}
// Represent the supporting line near the target, rather than constructing a
// boundary point by interpolating huge segment endpoints. The dominant-axis
// anchor bounds its slope and preserves simple axis/diagonal line offsets.
struct SweepLine {
    DVector reference, direction;
    double startProjection, endProjection, length;
    SweepLine(Vector2 start, Vector2 end, const Transform& transform) {
        const DVector motion = ToDouble(end) - ToDouble(start);
        const double worldLength = std::hypot(motion.x, motion.y);
        const DVector worldDirection = motion * (1 / worldLength);
        DVector worldReference;
        if (std::abs(motion.x) >= std::abs(motion.y))
            worldReference = {0, std::fma(-static_cast<double>(start.x), motion.y / motion.x, start.y)};
        else
            worldReference = {std::fma(-static_cast<double>(start.y), motion.x / motion.y, start.x), 0};
        reference = transform.LocalDouble(worldReference);
        direction = {transform.cosine * worldDirection.x + transform.sine * worldDirection.y,
                     -transform.sine * worldDirection.x + transform.cosine * worldDirection.y};
        const double scale = std::hypot(direction.x, direction.y);
        direction = direction * (1 / scale);
        startProjection = Dot(worldDirection, ToDouble(start) - worldReference) * scale;
        endProjection = Dot(worldDirection, ToDouble(end) - worldReference) * scale;
        length = worldLength * scale;
    }
    bool Includes(double projection) const {
        return projection >= startProjection && projection <= endProjection;
    }
    double Fraction(double projection) const {
        return std::clamp((projection - startProjection) / length, 0.0, 1.0);
    }
};
struct SweepCandidate { double projection; DVector center, contact, normal; };
std::optional<SweepCandidate> DiskEntry(const SweepLine& line, DVector center,
                                       double expandedRadius, double targetRadius) {
    const double perpendicular = Cross(line.direction, line.reference - center);
    const double distance = std::abs(perpendicular);
    if (distance > expandedRadius) return std::nullopt;
    const double halfChord = std::sqrt((expandedRadius - distance) * (expandedRadius + distance));
    const double entry = Dot(center - line.reference, line.direction) - halfChord;
    if (!line.Includes(entry)) return std::nullopt;
    const DVector offset = DVector{-line.direction.y, line.direction.x} * perpendicular
        - line.direction * halfChord;
    const DVector normal = offset * (1 / expandedRadius);
    return SweepCandidate{entry, center + offset, center + normal * targetRadius, normal};
}
std::optional<SweptCircleHit> PolygonSweep(const Polygon& polygon, const Transform& transform,
    Vector2 start, Vector2 end, double radius) {
    const SweepLine line(start, end, transform);
    const auto& vertices = polygon.getVertices();
    const double winding = Winding(polygon);
    std::optional<SweepCandidate> best;
    const auto consider = [&](const SweepCandidate& candidate) {
        if (line.Includes(candidate.projection) && (!best || candidate.projection < best->projection))
            best = candidate;
    };
    // Finite outward offset faces are checked before corner disks, giving
    // deterministic exact-parameter feature ties in stored index order.
    for (std::size_t i = 0; i < vertices.size(); ++i) {
        const DVector a = ToDouble(vertices[i]), b = ToDouble(vertices[(i + 1) % vertices.size()]);
        const DVector normal = Outward(a, b, winding);
        if (Dot(normal, line.direction) >= 0) continue;
        const DVector edge = b - a;
        const double edgeLength = std::hypot(edge.x, edge.y);
        const DVector tangent = edge * (1 / edgeLength);
        const DVector offset = a + normal * radius;
        const double denominator = Cross(line.direction, tangent);
        const double alongEdge = Cross(offset - line.reference, line.direction) / denominator;
        if (alongEdge < 0 || alongEdge > edgeLength) continue;
        const DVector contact = a + tangent * alongEdge;
        const DVector center = contact + normal * radius;
        consider({Dot(center - line.reference, line.direction), center, contact, normal});
    }
    // The polygon, face strips and full vertex disks form the exact rounded
    // Minkowski expansion. Earliest entry into this union is on its boundary;
    // interior portions of a corner disk cannot preempt an earlier feature.
    for (const auto& vertex : vertices)
        if (const auto corner = DiskEntry(line, ToDouble(vertex), radius, 0)) consider(*corner);
    if (!best) return std::nullopt;
    return SweptCircleHit{line.Fraction(best->projection), transform.World(best->center),
        transform.World(best->contact), ToFloat(transform.Rotate(best->normal))};
}
bool Earlier(const WorldRayHit& first, const WorldRayHit& second) {
    if (first.hit.fraction != second.hit.fraction)
        return first.hit.fraction < second.hit.fraction;
    return first.body->GetId() < second.body->GetId();
}
bool EarlierSweep(const WorldSweptCircleHit& first, const WorldSweptCircleHit& second) {
    if (first.hit.fraction != second.hit.fraction) return first.hit.fraction < second.hit.fraction;
    return first.body->GetId() < second.body->GetId();
}
}

bool ContainsPoint(const Shape& shape, Vector2 point, Vector2 position, float orientation) {
    Validate(point);
    const Transform transform(position, orientation);
    if (shape.type == ShapeType::CIRCLE) {
        const double radius = AsCircle(shape).GetRadius();
        const DVector relative = ToDouble(point) - transform.position;
        return Dot(relative, relative) <= radius * radius;
    }
    if (shape.type == ShapeType::POLYGON)
        return PolygonContains(AsPolygon(shape), transform.Local(point));
    throw std::invalid_argument("Spatial queries require a Circle or convex Polygon.");
}
bool ContainsPoint(const RigidBody& body, Vector2 point) {
    if (!body.shape) throw std::invalid_argument("Spatial queries require a body shape.");
    return ContainsPoint(*body.shape, point, body.GetPosition(), body.GetOrientation());
}
bool OverlapsCircle(const Shape& shape, Vector2 center, float radius, Vector2 position, float orientation) {
    Validate(center); ValidateRadius(radius);
    if (radius == 0) return ContainsPoint(shape, center, position, orientation);
    const Transform transform(position, orientation);
    if (shape.type == ShapeType::CIRCLE) {
        const double sum = static_cast<double>(AsCircle(shape).GetRadius()) + radius;
        const DVector relative = ToDouble(center) - transform.position;
        return std::hypot(relative.x, relative.y) <= sum;
    }
    if (shape.type == ShapeType::POLYGON)
        return PolygonCircle(AsPolygon(shape), transform.Local(center), radius);
    throw std::invalid_argument("Spatial queries require a Circle or convex Polygon.");
}
bool OverlapsCircle(const RigidBody& body, Vector2 center, float radius) {
    if (!body.shape) throw std::invalid_argument("Spatial queries require a body shape.");
    return OverlapsCircle(*body.shape, center, radius, body.GetPosition(), body.GetOrientation());
}
std::optional<RayHit> RayCast(const Shape& shape, Vector2 start, Vector2 end,
    Vector2 position, float orientation) {
    Validate(start); Validate(end);
    const Transform transform(position, orientation);
    if (shape.type == ShapeType::CIRCLE)
        return CircleRay(AsCircle(shape), transform.position, start, end);
    if (shape.type == ShapeType::POLYGON)
        return PolygonRay(AsPolygon(shape), transform, start, end);
    throw std::invalid_argument("Spatial queries require a Circle or convex Polygon.");
}
std::optional<RayHit> RayCast(const RigidBody& body, Vector2 start, Vector2 end) {
    if (!body.shape) throw std::invalid_argument("Spatial queries require a body shape.");
    return RayCast(*body.shape, start, end, body.GetPosition(), body.GetOrientation());
}
std::optional<SweptCircleHit> SweepCircle(const Shape& shape, Vector2 start, Vector2 end,
    float radius, Vector2 position, float orientation) {
    Validate(start); Validate(end); ValidateRadius(radius);
    if (radius == 0) {
        const auto hit = RayCast(shape, start, end, position, orientation);
        if (!hit) return std::nullopt;
        return SweptCircleHit{hit->fraction, hit->point, hit->point, hit->normal};
    }
    const Transform transform(position, orientation);
    if (OverlapsCircle(shape, start, radius, position, orientation))
        return SweptCircleHit{0, start, start, {}};
    if (start.x == end.x && start.y == end.y) return std::nullopt;
    if (shape.type == ShapeType::CIRCLE) {
        const double targetRadius = AsCircle(shape).GetRadius();
        const SweepLine line(start, end, Transform({}, 0));
        const auto hit = DiskEntry(line, transform.position, targetRadius + radius, targetRadius);
        if (!hit) return std::nullopt;
        return SweptCircleHit{line.Fraction(hit->projection), ToFloat(hit->center),
            ToFloat(hit->contact), ToFloat(hit->normal)};
    }
    if (shape.type == ShapeType::POLYGON)
        return PolygonSweep(AsPolygon(shape), transform, start, end, radius);
    throw std::invalid_argument("Spatial queries require a Circle or convex Polygon.");
}
std::optional<SweptCircleHit> SweepCircle(const RigidBody& body, Vector2 start, Vector2 end, float radius) {
    if (!body.shape) throw std::invalid_argument("Spatial queries require a body shape.");
    return SweepCircle(*body.shape, start, end, radius, body.GetPosition(), body.GetOrientation());
}
namespace {
template<class Scene>
std::vector<RigidBodyPtr> PointResults(const Scene& world, Vector2 point, QueryFilter filter) {
    Validate(point);
    std::vector<RigidBodyPtr> result;
    for (const auto& body : world.getBodies())
        if (Matches(*body, filter) && ContainsPoint(*body, point)) result.push_back(body);
    std::sort(result.begin(), result.end(), [](const RigidBodyPtr& a, const RigidBodyPtr& b) {
        return a->GetId() < b->GetId();
    });
    return result;
}
template<class Scene>
std::vector<RigidBodyPtr> CircleResults(const Scene& world, Vector2 center, float radius, QueryFilter filter) {
    Validate(center); ValidateRadius(radius);
    std::vector<RigidBodyPtr> result;
    for (const auto& body : world.getBodies())
        if (Matches(*body, filter) && OverlapsCircle(*body, center, radius)) result.push_back(body);
    std::sort(result.begin(), result.end(), [](const RigidBodyPtr& a, const RigidBodyPtr& b) {
        return a->GetId() < b->GetId();
    });
    return result;
}
template<class Scene>
std::vector<WorldRayHit> RayResults(const Scene& world, Vector2 start, Vector2 end, QueryFilter filter) {
    Validate(start); Validate(end);
    std::vector<WorldRayHit> result;
    for (const auto& body : world.getBodies()) {
        if (!Matches(*body, filter)) continue;
        if (const auto hit = RayCast(*body, start, end)) result.push_back({body, *hit});
    }
    std::sort(result.begin(), result.end(), Earlier);
    return result;
}
template<class Scene>
std::optional<WorldRayHit> NearestRay(const Scene& world, Vector2 start, Vector2 end, QueryFilter filter) {
    Validate(start); Validate(end);
    std::optional<WorldRayHit> result;
    for (const auto& body : world.getBodies()) {
        if (!Matches(*body, filter)) continue;
        if (const auto hit = RayCast(*body, start, end)) {
            WorldRayHit candidate{body, *hit};
            if (!result || Earlier(candidate, *result)) result = candidate;
        }
    }
    return result;
}
template<class Scene>
std::vector<WorldSweptCircleHit> SweepResults(const Scene& world, Vector2 start, Vector2 end,
    float radius, QueryFilter filter) {
    Validate(start); Validate(end); ValidateRadius(radius);
    std::vector<WorldSweptCircleHit> result;
    for (const auto& body : world.getBodies()) {
        if (!Matches(*body, filter)) continue;
        if (const auto hit = SweepCircle(*body, start, end, radius)) result.push_back({body, *hit});
    }
    std::sort(result.begin(), result.end(), EarlierSweep);
    return result;
}
template<class Scene>
std::optional<WorldSweptCircleHit> NearestSweep(const Scene& world, Vector2 start, Vector2 end,
    float radius, QueryFilter filter) {
    Validate(start); Validate(end); ValidateRadius(radius);
    std::optional<WorldSweptCircleHit> result;
    for (const auto& body : world.getBodies()) {
        if (!Matches(*body, filter)) continue;
        if (const auto hit = SweepCircle(*body, start, end, radius)) {
            WorldSweptCircleHit candidate{body, *hit};
            if (!result || EarlierSweep(candidate, *result)) result = candidate;
        }
    }
    return result;
}
} // namespace

std::vector<RigidBodyPtr> QueryPoint(const World& s, Vector2 p, QueryFilter f) { return PointResults(s,p,f); }
std::vector<RigidBodyPtr> QueryPoint(const Engine& s, Vector2 p, QueryFilter f) { return PointResults(s,p,f); }
std::vector<RigidBodyPtr> QueryCircle(const World& s, Vector2 p, float r, QueryFilter f) { return CircleResults(s,p,r,f); }
std::vector<RigidBodyPtr> QueryCircle(const Engine& s, Vector2 p, float r, QueryFilter f) { return CircleResults(s,p,r,f); }
std::vector<WorldRayHit> RayCastAll(const World& s, Vector2 a, Vector2 b, QueryFilter f) { return RayResults(s,a,b,f); }
std::vector<WorldRayHit> RayCastAll(const Engine& s, Vector2 a, Vector2 b, QueryFilter f) { return RayResults(s,a,b,f); }
std::optional<WorldRayHit> RayCastNearest(const World& s, Vector2 a, Vector2 b, QueryFilter f) { return NearestRay(s,a,b,f); }
std::optional<WorldRayHit> RayCastNearest(const Engine& s, Vector2 a, Vector2 b, QueryFilter f) { return NearestRay(s,a,b,f); }
std::vector<WorldSweptCircleHit> SweepCircleAll(const World& s, Vector2 a, Vector2 b, float r, QueryFilter f) { return SweepResults(s,a,b,r,f); }
std::vector<WorldSweptCircleHit> SweepCircleAll(const Engine& s, Vector2 a, Vector2 b, float r, QueryFilter f) { return SweepResults(s,a,b,r,f); }
std::optional<WorldSweptCircleHit> SweepCircleNearest(const World& s, Vector2 a, Vector2 b, float r, QueryFilter f) { return NearestSweep(s,a,b,r,f); }
std::optional<WorldSweptCircleHit> SweepCircleNearest(const Engine& s, Vector2 a, Vector2 b, float r, QueryFilter f) { return NearestSweep(s,a,b,r,f); }
}

#pragma once

#include "physics/core/shape.h"
#include "physics/core/types.h"
#include <cstdint>
#include <optional>
#include <vector>

namespace PhysicsEngine {
class World;

// Query categories and masks use the same mutual agreement as body collisions.
// The default accepts every body whose category and mask are both nonzero.
struct QueryFilter {
    std::uint32_t categoryBits = 0xFFFFFFFFu;
    std::uint32_t maskBits = 0xFFFFFFFFu;
};

struct RayHit {
    double fraction = 0; // Parameter on the closed segment [start, end].
    Vector2 point;
    Vector2 normal; // Outward unit normal; zero for an initially contained start.
};

struct WorldRayHit {
    RigidBodyPtr body; // Retained ownership, valid after removal from the World.
    RayHit hit;
};

// Circle and convex Polygon queries. Polygon vertices remain in their original
// local frame, including an offset from the origin; position/orientation supply
// the body's world transform. Boundary points are contained.
bool ContainsPoint(const Shape& shape, Vector2 point,
    Vector2 position = {}, float orientation = 0);
bool ContainsPoint(const RigidBody& body, Vector2 point);

// Closed disk overlap, including boundary contact. Radius must be finite and
// nonnegative; zero radius delegates to ContainsPoint for identical semantics.
bool OverlapsCircle(const Shape& shape, Vector2 center, float radius,
    Vector2 position = {}, float orientation = 0);
bool OverlapsCircle(const RigidBody& body, Vector2 center, float radius);

// First contact with a finite segment, including both endpoints and tangency.
// A contained start (also a zero-length contained segment) returns fraction 0,
// point=start, normal=(0,0). An outside zero-length segment has no hit.
// Polygon corner normals use the lowest stored edge index among entry faces.
std::optional<RayHit> RayCast(const Shape& shape, Vector2 start, Vector2 end,
    Vector2 position = {}, float orientation = 0);
std::optional<RayHit> RayCast(const RigidBody& body, Vector2 start, Vector2 end);

// Linear scans of the current body transforms; no result limit or broad phase.
// Point results sort by ID; ray results sort by (fraction, ID), without epsilon
// ties. Static and sleeping bodies participate. Inputs must be finite, even in
// an empty world; relevant invalid body transforms throw invalid_argument.
std::vector<RigidBodyPtr> QueryPoint(const World& world, Vector2 point,
    QueryFilter filter = {});
// All overlapping bodies, in stable ID order, with retained ownership.
std::vector<RigidBodyPtr> QueryCircle(const World& world, Vector2 center,
    float radius, QueryFilter filter = {});
std::vector<WorldRayHit> RayCastAll(const World& world, Vector2 start, Vector2 end,
    QueryFilter filter = {});
std::optional<WorldRayHit> RayCastNearest(const World& world, Vector2 start,
    Vector2 end, QueryFilter filter = {});
}

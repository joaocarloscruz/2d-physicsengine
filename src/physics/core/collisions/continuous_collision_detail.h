#pragma once
// Engine-internal sweep entry points (not part of the public API).
#include "physics/core/collisions/continuous_collision.h"
#include <vector>

namespace PhysicsEngine {
namespace Detail {
// SweepCirclePolygon for a world outline that World CCD computes from a
// validated Polygon by rotating and translating its local vertices in float.
// That rounding can make two close vertices coincide or turn a strictly
// convex corner by an ulp, which the public function (rightly) rejects as an
// input. This entry point skips the outline validation, takes the winding
// from the signed area and skips zero-length edges (their vertex disks still
// count). For any outline the public function accepts it returns the same
// result bit for bit.
SweepHit SweepCirclePolygonRounded(Vector2 center, Vector2 displacement, float radius,
    const std::vector<Vector2>& vertices, Vector2 polygonDisplacement);
}
}

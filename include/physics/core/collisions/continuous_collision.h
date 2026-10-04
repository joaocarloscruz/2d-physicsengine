#pragma once
#include "physics/math/vector2.h"
#include <vector>

namespace PhysicsEngine {
struct SweepHit {
    bool hit = false;
    float fraction = 1.0f; // May round away a small impact offset on enormous travel.
    Vector2 normal; // from circle A toward shape B
    Vector2 point;
};

// Displacements cover the entire query interval. Polygon vertices are in world
// coordinates at its start; the polygon translates without rotating.
// Inputs/radii must be finite, radii positive; output Vector2 range is checked.
// Initial circle/circle overlap reports the A surface along the center direction
// (coincident fallback +X). Helper geometry can retain a local contact even when
// the public float fraction cannot reproduce that contact by interpolation.
SweepHit SweepCircleCircle(Vector2 a, Vector2 displacementA, float radiusA,
    Vector2 b, Vector2 displacementB, float radiusB);
SweepHit SweepCirclePolygon(Vector2 center, Vector2 displacement, float radius,
    const std::vector<Vector2>& vertices, Vector2 polygonDisplacement = {});
}

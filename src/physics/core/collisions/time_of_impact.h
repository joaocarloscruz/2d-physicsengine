#pragma once
// Engine-internal time of impact for any pair of convex shapes that translate
// and rotate at constant rates (not part of the public API). World CCD uses it
// for every pair that the exact circle sweeps in continuous_collision.cpp do
// not cover: pairs without a circle, and circles against a rotating body.
#include "physics/math/vector2.h"
#include <array>
#include <cstddef>

namespace PhysicsEngine {
class Shape;
namespace Detail {

// A convex shape as a core and a rounding radius: a circle is a point, a
// capsule a segment and a polygon its outline (radius 0). Vertices are in the
// body frame. A polygon proxy points at the shape's own vertices, so it must
// not outlive the shape.
struct CcdProxy {
    const Vector2* vertices = nullptr;
    std::size_t count = 0;
    double radius = 0;
    double reach = 0;     // largest distance of a core vertex from the body origin
    double thickness = 0; // smallest half-width of the shape (core half-width + radius)
    std::array<Vector2, 2> own{};

    const Vector2& vertex(std::size_t i) const { return vertices ? vertices[i] : own[i]; }
};

// False for a shape that is not a Circle, Polygon or Capsule (for example a
// custom Shape whose tag does not match its class); such a pair is not swept.
bool MakeCcdProxy(const Shape& shape, CcdProxy& proxy);

struct CcdPose { double x = 0, y = 0, angle = 0; };

struct CcdSeparation {
    // Signed distance between the rounded shapes: positive when apart,
    // minus the penetration depth when they overlap.
    double distance = 0;
    double normalX = 1, normalY = 0; // from A toward B
    // Witness points on (or, when overlapping, near) each rounded surface.
    double ax = 0, ay = 0, bx = 0, by = 0;
};

CcdSeparation ComputeSeparation(const CcdProxy& a, const CcdPose& poseA,
    const CcdProxy& b, const CcdPose& poseB);

// Pose at time 0 and constant linear and angular velocities.
struct CcdMotion { CcdPose pose; double vx = 0, vy = 0, omega = 0; };

CcdPose PoseAt(const CcdMotion& motion, double t);

struct CcdToi {
    enum class State { Separated, Hit, Exhausted } state = State::Separated;
    double time = 0;           // first time with distance <= target + tolerance
    CcdSeparation separation;  // at that time
    int iterations = 0;
};

// Conservative advancement: the earliest time in [0, tMax] at which the signed
// distance falls to target + tolerance, never below target. Each step moves by
// (distance - target) / bound, where bound is an upper limit on the closing
// speed along the current normal (linear part plus |omega| * reach for each
// body), so no contact is skipped. Hit at time 0 when the pair already starts
// within target + tolerance. Exhausted reports the last safe time when the
// iteration cap is reached first; callers treat it as a hit there.
CcdToi TimeOfImpact(const CcdProxy& a, const CcdMotion& motionA,
    const CcdProxy& b, const CcdMotion& motionB,
    double tMax, double target, double tolerance, int maxIterations = 64);

// Velocity of B's witness point minus A's, along the normal (negative closes).
double ClosingSpeed(const CcdSeparation& s, const CcdMotion& motionA, const CcdPose& poseA,
    const CcdMotion& motionB, const CcdPose& poseB);
}
}

#include "physics/core/collisions/continuous_collision.h"
#include "physics/core/shape.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace PhysicsEngine {
namespace {
void Validate(Vector2 p) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y))
        throw std::invalid_argument("Sweep coordinates must be finite.");
}
void ValidateRadius(float r) {
    if (!std::isfinite(r) || r <= 0)
        throw std::invalid_argument("Sweep radius must be positive and finite.");
}
// First intersection of a moving point and a stationary disk, in double precision.
float DiskTime(Vector2 offset, Vector2 motion, float radius) {
    const double a = static_cast<double>(motion.x)*motion.x + static_cast<double>(motion.y)*motion.y;
    const double b = static_cast<double>(offset.x)*motion.x + static_cast<double>(offset.y)*motion.y;
    const double c = static_cast<double>(offset.x)*offset.x + static_cast<double>(offset.y)*offset.y
        - static_cast<double>(radius)*radius;
    if (c <= 0) return 0;
    if (a == 0 || b >= 0) return std::numeric_limits<float>::infinity();
    const double discriminant = b*b - a*c;
    if (discriminant < 0) return std::numeric_limits<float>::infinity();
    return static_cast<float>(c / (-b + std::sqrt(discriminant)));
}
}

SweepHit SweepCircleCircle(Vector2 a, Vector2 da, float ra,
    Vector2 b, Vector2 db, float rb) {
    Validate(a); Validate(b); Validate(da); Validate(db);
    ValidateRadius(ra); ValidateRadius(rb);
    SweepHit result;
    const float t = DiskTime(a-b, da-db, ra+rb);
    if (t > 1) return result;
    Vector2 normal = b+db*t - a-da*t;
    normal = normal.magnitudeSquared() > 0 ? normal.normalized() : Vector2(1, 0);
    result = {true, t, normal, a+da*t+normal*ra};
    return result;
}

SweepHit SweepCirclePolygon(Vector2 center, Vector2 displacement, float radius,
    const std::vector<Vector2>& vertices, Vector2 polygonDisplacement) {
    Validate(center); Validate(displacement); Validate(polygonDisplacement);
    ValidateRadius(radius);
    Polygon validated(vertices);
    const Vector2 motion = displacement-polygonDisplacement;
    double signedArea = 0;
    for (std::size_t i=0; i<vertices.size(); ++i)
        signedArea += vertices[i].cross(vertices[(i+1)%vertices.size()]);
    bool inside = true;
    float closestDistance = std::numeric_limits<float>::infinity();
    Vector2 closestPoint;
    SweepHit result;
    auto accept = [&](float t, Vector2 normal) {
        if (t >= 0 && t <= 1 && (!result.hit || t < result.fraction))
            result = {true, t, normal, center+displacement*t+normal*radius};
    };
    for (std::size_t i=0; i<vertices.size(); ++i) {
        const Vector2 v = vertices[i];
        const Vector2 edge = vertices[(i+1)%vertices.size()]-v;
        const float length = edge.magnitude();
        const Vector2 tangent = edge/length;
        const Vector2 outward = Vector2(tangent.y, -tangent.x)*(signedArea > 0 ? 1.0f : -1.0f);
        const float distance = (center-v).dot(outward);
        inside = inside && distance <= 0;
        const Vector2 closest = v+tangent*std::clamp((center-v).dot(tangent), 0.0f, length);
        const float d2 = (closest-center).magnitudeSquared();
        if (d2 < closestDistance) { closestDistance = d2; closestPoint = closest; }
        const float speed = motion.dot(outward);
        if (speed < 0) {
            const float t = (radius-distance)/speed;
            const float projection = (center+motion*t-v).dot(tangent);
            if (projection >= 0 && projection <= length) accept(t, outward*-1.0f);
        }
        const float t = DiskTime(center-v, motion, radius);
        if (t <= 1) {
            const Vector2 direction = v-center-motion*t;
            if (direction.magnitudeSquared() > 0) accept(t, direction.normalized());
        }
    }
    if (inside || closestDistance <= radius*radius) {
        Vector2 normal = closestPoint-center;
        if (inside) normal = normal*-1.0f;
        normal = normal.magnitudeSquared() > 0 ? normal.normalized() : Vector2(1, 0);
        return {true, 0, normal, closestPoint};
    }
    return result;
}
}

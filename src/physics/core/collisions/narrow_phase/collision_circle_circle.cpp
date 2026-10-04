#include "physics/core/collisions/narrow_phase/collision_circle_circle.h"
#include "physics/core/shape.h"
#include <cmath>
#include <limits>
#include <stdexcept>

namespace {
float ManifoldFloat(double value) {
    if (!std::isfinite(value) || std::abs(value) > std::numeric_limits<float>::max())
        throw std::overflow_error("Circle contact exceeds finite manifold range");
    return static_cast<float>(value);
}
}

PhysicsEngine::CollisionManifold PhysicsEngine::CollisionCircleCircle(RigidBody* a, RigidBody* b) {
    CollisionManifold manifold;
    manifold.A = a;
    manifold.B = b;
    const auto* circleA = static_cast<const Circle*>(a->shape.get());
    const auto* circleB = static_cast<const Circle*>(b->shape.get());
    const double dx = double(b->position.x) - a->position.x;
    const double dy = double(b->position.y) - a->position.y;
    if (!std::isfinite(dx) || !std::isfinite(dy))
        throw std::invalid_argument("Circle contact positions must be finite");
    const double distance = std::hypot(dx, dy);
    const double sumRadii = double(circleA->GetRadius()) + circleB->GetRadius();
    if (distance >= sumRadii) return manifold;

    manifold.penetration = ManifoldFloat(sumRadii - distance);
    if (distance == 0) {
        manifold.normal = Vector2(1, 0);
        manifold.contactPoint = a->position;
    } else {
        const double nx = dx / distance, ny = dy / distance;
        manifold.normal = Vector2(ManifoldFloat(nx), ManifoldFloat(ny));
        // Retain the surface-of-A convention, checking the final stored point.
        manifold.contactPoint = Vector2(
            ManifoldFloat(a->position.x + nx * circleA->GetRadius()),
            ManifoldFloat(a->position.y + ny * circleA->GetRadius()));
    }
    manifold.hasCollision = true;
    manifold.contactCount = 1;
    manifold.contacts[0] = ContactPoint{manifold.contactPoint, manifold.penetration, 0x20000001u};
    return manifold;
}

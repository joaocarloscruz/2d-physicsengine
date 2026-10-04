#include "physics/core/collisions/narrow_phase/collision_circle_polygon.h"
#include "physics/core/shape.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace PhysicsEngine {
namespace {
struct D2 {
    double x, y;
    D2 operator*(double s) const { return {x*s,y*s}; }
    double dot(D2 v) const { return x*v.x+y*v.y; }
    double norm() const { return std::hypot(x,y); }
};
float Store(double value) {
    if (!std::isfinite(value) || std::abs(value)>std::numeric_limits<float>::max())
        throw std::overflow_error("Circle-polygon manifold exceeds finite float range.");
    return static_cast<float>(value);
}
void Validate(const RigidBody* body) {
    if (!body || !body->shape || !std::isfinite(body->position.x)
        || !std::isfinite(body->position.y) || !std::isfinite(body->orientation))
        throw std::invalid_argument("Circle-polygon collision requires finite body transforms.");
}
}

CollisionManifold CollisionCirclePolygon(RigidBody* a, RigidBody* b) {
    Validate(a); Validate(b);
    RigidBody* circleBody = a->shape->type==ShapeType::CIRCLE ? a : b;
    RigidBody* polygonBody = circleBody==a ? b : a;
    const auto* circle = dynamic_cast<const Circle*>(circleBody->shape.get());
    const auto* polygon = dynamic_cast<const Polygon*>(polygonBody->shape.get());
    if (!circle || !polygon || a==b || !std::isfinite(circle->GetRadius()) || circle->GetRadius()<=0)
        throw std::invalid_argument("Circle-polygon collision requires one Circle and one Polygon.");

    const auto& local = polygon->getVertices();
    if (local.size()<3) throw std::invalid_argument("Polygon requires at least three vertices.");
    const double c=std::cos(double(polygonBody->orientation)), s=std::sin(double(polygonBody->orientation));
    const D2 translation{double(polygonBody->position.x)-circleBody->position.x,
                         double(polygonBody->position.y)-circleBody->position.y};
    std::vector<D2> relative;
    relative.reserve(local.size());
    for (const auto& v : local) {
        if (!std::isfinite(v.x) || !std::isfinite(v.y))
            throw std::invalid_argument("Polygon vertices must be finite.");
        relative.push_back({translation.x+c*v.x-s*v.y,translation.y+s*v.x+c*v.y});
    }
    CollisionManifold result; result.A=a; result.B=b;
    const double radius=circle->GetRadius();
    double minimum=std::numeric_limits<double>::infinity();
    D2 escape{};
    auto testAxis = [&](D2 axis) {
        double low=std::numeric_limits<double>::infinity(), high=-low;
        // Use the circle as the projection origin to remove common world
        // translations from the separating interval arithmetic.
        for (const auto& p : relative) {
            const double projection=p.dot(axis);
            low=std::min(low,projection); high=std::max(high,projection);
        }
        if (high<=-radius || low>=radius) return false; // Exact touch is not overlap.
        const double negative=radius-low, positive=high+radius;
        const double overlap=std::min(negative,positive);
        if (overlap<minimum) {
            minimum=overlap; escape=axis*(negative<positive ? -1.0 : 1.0);
        }
        return true;
    };
    for (std::size_t i=0;i<local.size();++i) {
        // Form edges before adding a potentially enormous translation.
        const auto& from=local[i]; const auto& to=local[(i+1)%local.size()];
        const double dx=double(to.x)-from.x, dy=double(to.y)-from.y;
        const D2 edge{c*dx-s*dy,s*dx+c*dy};
        const double length=edge.norm();
        if (length==0) throw std::invalid_argument("Polygon edges must be nonzero.");
        if (!testAxis({-edge.y/length,edge.x/length})) return result;
    }
    D2 closest=relative.front(); double closestDistance=closest.norm();
    for (const auto& vertex : relative) {
        const double distance=vertex.norm();
        if (distance<closestDistance) { closest=vertex; closestDistance=distance; }
    }
    // Every nonzero corner direction matters, regardless of length units.
    if (closestDistance>0 && !testAxis(closest*(-1/closestDistance))) return result;

    const D2 normal=escape*(circleBody==a ? -1.0 : 1.0);
    const D2 towardsPolygon=escape*(-1);
    result.normal={Store(normal.x),Store(normal.y)};
    result.penetration=Store(minimum);
    result.contactPoint={Store(circleBody->position.x+towardsPolygon.x*radius),
                         Store(circleBody->position.y+towardsPolygon.y*radius)};
    result.hasCollision=true; result.contactCount=1;
    result.contacts[0]={result.contactPoint,result.penetration,0x40000001u};
    return result;
}
}

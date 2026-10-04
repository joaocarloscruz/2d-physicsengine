#include "physics/core/collisions/continuous_collision.h"
#include "physics/core/shape.h"
#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <limits>
#include <optional>
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
struct DVector {
    double x, y;
    DVector operator+(DVector b) const { return {x+b.x,y+b.y}; }
    DVector operator-(DVector b) const { return {x-b.x,y-b.y}; }
    DVector operator*(double t) const { return {x*t,y*t}; }
};
DVector Double(Vector2 p) { return {p.x,p.y}; }
double Dot(DVector a,DVector b) { return a.x*b.x+a.y*b.y; }
double Cross(DVector a,DVector b) { return a.x*b.y-a.y*b.x; }
Vector2 Store(DVector p) {
    const double maximum=std::numeric_limits<float>::max();
    if (!std::isfinite(p.x)||!std::isfinite(p.y)||std::abs(p.x)>maximum||std::abs(p.y)>maximum)
        throw std::overflow_error("Sweep result exceeds finite Vector2 range.");
    return {static_cast<float>(p.x),static_cast<float>(p.y)};
}
// Keep a small endpoint displacement when large original terms cancel. All
// inputs are finite floats (or a bounded line anchor), so these sums fit double.
double Sum(std::initializer_list<double> values) {
    double total=0,correction=0;
    for (double value : values) {
        const double next=total+value;
        correction+=std::abs(total)>=std::abs(value)?(total-next)+value:(value-next)+total;
        total=next;
    }
    return total+correction;
}
struct Line {
    DVector reference,direction;
    Vector2 start,displacement,targetDisplacement;
    double startProjection,endProjection,length;
    Line(Vector2 a,Vector2 da,Vector2 db)
        : start(a),displacement(da),targetDisplacement(db) {
        const DVector motion=Double(da)-Double(db);
        length=std::hypot(motion.x,motion.y);
        direction=motion*(1/length);
        if (std::abs(motion.x)>=std::abs(motion.y))
            reference={0,std::fma(-static_cast<double>(a.x),motion.y/motion.x,a.y)};
        else
            reference={std::fma(-static_cast<double>(a.y),motion.x/motion.y,a.x),0};
        startProjection=Dot(direction,Double(a)-reference);
        const DVector relativeEnd{Sum({a.x,da.x,-static_cast<double>(db.x),-reference.x}),
                                  Sum({a.y,da.y,-static_cast<double>(db.y),-reference.y})};
        endProjection=Dot(direction,relativeEnd);
    }
    bool includes(double projection) const { return projection>=startProjection&&projection<=endProjection; }
    double fraction(double projection) const { return std::clamp((projection-startProjection)/length,0.0,1.0); }
    DVector endpointRelativeTo(DVector point) const {
        return {Sum({start.x,displacement.x,-static_cast<double>(targetDisplacement.x),-point.x}),
                Sum({start.y,displacement.y,-static_cast<double>(targetDisplacement.y),-point.y})};
    }
    DVector translated(DVector point,DVector offset,double projection,double localProjection) const {
        // Split bulk and feature projections before multiplying translation:
        // a tiny local impact correction can survive a rounded .5 fraction.
        const double bulk=-startProjection/length,feature=projection/length,local=localProjection/length;
        return {Sum({std::fma(targetDisplacement.x,feature,point.x),targetDisplacement.x*bulk,
                     targetDisplacement.x*local,offset.x}),
                Sum({std::fma(targetDisplacement.y,feature,point.y),targetDisplacement.y*bulk,
                     targetDisplacement.y*local,offset.y})};
    }
};
struct DiskHit { double projection,centerProjection,localProjection; DVector outward; };
std::optional<DiskHit> DiskEntry(const Line& line,DVector center,double radius) {
    const double perpendicular=Cross(line.direction,line.reference-center);
    const double distance=std::abs(perpendicular);
    if (distance>radius) return std::nullopt;
    const double halfChord=std::sqrt((radius-distance)*(radius+distance));
    const double centerProjection=Dot(center-line.reference,line.direction);
    // Compare near the disk itself; distant target coordinates must not round
    // an endpoint several radii short into the same enormous line projection.
    if (-halfChord<Dot(Double(line.start)-center,line.direction)
        ||-halfChord>Dot(line.endpointRelativeTo(center),line.direction)) return std::nullopt;
    const double projection=centerProjection-halfChord;
    const DVector offset=DVector{-line.direction.y,line.direction.x}*perpendicular-line.direction*halfChord;
    return DiskHit{projection,centerProjection,-halfChord,offset*(1/radius)};
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
    const DVector offset=Double(b)-Double(a);
    const double distance=std::hypot(offset.x,offset.y);
    const double radius=static_cast<double>(ra)+rb;
    if (distance<=radius) {
        const DVector normal=distance>0?offset*(1/distance):DVector{1,0};
        return {true,0,Store(normal),Store(Double(a)+normal*ra)};
    }
    const DVector motion=Double(da)-Double(db);
    if (motion.x==0&&motion.y==0) return {};
    const Line line(a,da,db);
    const auto hit=DiskEntry(line,Double(b),radius);
    if (!hit) return {};
    // At regular contact, A's surface and B's surface coincide. Construct from
    // B's small boundary feature instead of cancelling A's enormous travel.
    const DVector point=line.translated(Double(b),hit->outward*rb,hit->centerProjection,hit->localProjection);
    return {true,static_cast<float>(line.fraction(hit->projection)),Store(hit->outward*(-1)),Store(point)};
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

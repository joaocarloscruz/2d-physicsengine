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
    double startProjection,length;
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
    }
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
    const DVector motion=Double(displacement)-Double(polygonDisplacement);
    const double winding=Cross(Double(vertices[1])-Double(vertices[0]),Double(vertices[2])-Double(vertices[0]))>0?1:-1;
    bool inside = true;
    double closestDistance = std::numeric_limits<double>::infinity();
    DVector closestPoint{};
    for (std::size_t i=0; i<vertices.size(); ++i) {
        const DVector v=Double(vertices[i]),edge=Double(vertices[(i+1)%vertices.size()])-v;
        const double length=std::hypot(edge.x,edge.y);
        const DVector tangent=edge*(1/length),outward=DVector{tangent.y,-tangent.x}*winding;
        const double distance=Dot(Double(center)-v,outward);
        inside = inside && distance <= 0;
        const DVector closest=v+tangent*std::clamp(Dot(Double(center)-v,tangent),0.0,length);
        const DVector delta=closest-Double(center);
        const double distanceToEdge=std::hypot(delta.x,delta.y);
        if (distanceToEdge<closestDistance) {closestDistance=distanceToEdge;closestPoint=closest;}
    }
    if (inside || closestDistance <= radius) {
        const DVector delta=(closestPoint-Double(center))*(inside?-1:1);
        const double length=std::hypot(delta.x,delta.y);
        const DVector normal=length>0?delta*(1/length):DVector{1,0};
        return {true,0,Store(normal),Store(closestPoint)};
    }
    if (motion.x==0&&motion.y==0) return {};
    const Line line(center,displacement,polygonDisplacement);
    struct Candidate { double projection,baseProjection,localProjection; DVector point,offset,normal; };
    std::optional<Candidate> best;
    const auto accept=[&](const Candidate& candidate) {
        if (!best||candidate.projection<best->projection) best=candidate;
    };
    for (std::size_t i=0;i<vertices.size();++i) {
        const DVector v=Double(vertices[i]),edge=Double(vertices[(i+1)%vertices.size()])-v;
        const double length=std::hypot(edge.x,edge.y);
        const DVector tangent=edge*(1/length),outward=DVector{tangent.y,-tangent.x}*winding;
        if (Dot(outward,line.direction)<0
            && Dot(Double(center)-v,outward)>=radius
            && Dot(line.endpointRelativeTo(v),outward)<=radius) {
            const DVector offset=v+outward*radius;
            const double along=Cross(offset-line.reference,line.direction)/Cross(line.direction,tangent);
            if (along>=0&&along<=length) {
                const DVector contactOffset=tangent*along;
                const double baseProjection=Dot(v-line.reference,line.direction);
                const double localProjection=Dot(contactOffset+outward*radius,line.direction);
                accept({baseProjection+localProjection,baseProjection,localProjection,v,contactOffset,outward*(-1)});
            }
        }
        // Preserve stored edge-then-vertex traversal for exact feature ties.
        // Finite face strips and vertex disks form the rounded expansion.
        if (const auto corner=DiskEntry(line,v,radius))
            accept({corner->projection,corner->centerProjection,corner->localProjection,v,{},corner->outward*(-1)});
    }
    if (!best) return {};
    return {true,static_cast<float>(line.fraction(best->projection)),Store(best->normal),
        Store(line.translated(best->point,best->offset,best->baseProjection,best->localProjection))};
}
}

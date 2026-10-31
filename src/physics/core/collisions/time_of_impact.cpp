#include "time_of_impact.h"
#include "physics/core/shape.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace PhysicsEngine {
namespace Detail {
namespace {
struct P { double x = 0, y = 0; };
P operator+(P a, P b) { return {a.x + b.x, a.y + b.y}; }
P operator-(P a, P b) { return {a.x - b.x, a.y - b.y}; }
P operator*(P a, double s) { return {a.x * s, a.y * s}; }
double Dot(P a, P b) { return a.x * b.x + a.y * b.y; }

// World copy of a core. Small outlines stay on the stack.
class Core {
  public:
    Core(const CcdProxy& proxy, const CcdPose& pose) : n(proxy.count) {
        if (n > small.size()) large.resize(n);
        data = n > small.size() ? large.data() : small.data();
        const double c = std::cos(pose.angle), s = std::sin(pose.angle);
        for (std::size_t i = 0; i < n; ++i) {
            const Vector2& v = proxy.vertex(i);
            data[i] = {pose.x + c * v.x - s * v.y, pose.y + s * v.x + c * v.y};
        }
    }
    std::size_t size() const { return n; }
    P operator[](std::size_t i) const { return data[i]; }
    // A point has one degenerate edge, a segment one edge, an outline n.
    std::size_t edges() const { return n <= 2 ? 1 : n; }
    P edgeStart(std::size_t i) const { return data[i]; }
    P edgeEnd(std::size_t i) const { return n == 1 ? data[0] : data[(i + 1) % n]; }

  private:
    std::size_t n;
    std::array<P, 12> small;
    std::vector<P> large;
    P* data;
};

P ClosestOnSegment(P p, P a, P b) {
    const P e = b - a;
    const double length2 = Dot(e, e);
    if (length2 == 0) return a;
    const double t = std::clamp(Dot(p - a, e) / length2, 0.0, 1.0);
    return a + e * t;
}

// Distance from a point to a core: zero inside an outline.
double CoreDistance(P p, const Core& core) {
    double best = std::numeric_limits<double>::infinity();
    bool inside = core.size() >= 3;
    double sign = 0;
    for (std::size_t j = 0; j < core.edges(); ++j) {
        const P a = core.edgeStart(j), b = core.edgeEnd(j);
        const P q = ClosestOnSegment(p, a, b);
        best = std::min(best, std::hypot(p.x - q.x, p.y - q.y));
        if (core.size() >= 3) {
            const double cross = (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x);
            if (cross != 0) {
                if (sign == 0) sign = cross;
                else if ((cross > 0) != (sign > 0)) inside = false;
            }
        }
    }
    return inside ? 0 : best;
}

// Largest separation over the candidate axes, oriented from A toward B. A
// positive value proves the cores apart; otherwise it is minus the overlap
// depth (edge normals of both outlines contain the minimum-depth axis).
double SatSeparation(const Core& a, const Core& b, P& axis) {
    double best = -std::numeric_limits<double>::infinity();
    const bool degenerate = a.size() <= 2 || b.size() <= 2;
    const auto test = [&](P u) {
        const double length = std::hypot(u.x, u.y);
        if (length == 0) return;
        u = u * (1 / length);
        double minA = std::numeric_limits<double>::infinity(), maxA = -minA, minB = minA, maxB = -minA;
        for (std::size_t i = 0; i < a.size(); ++i) {
            const double d = Dot(a[i], u);
            minA = std::min(minA, d); maxA = std::max(maxA, d);
        }
        for (std::size_t i = 0; i < b.size(); ++i) {
            const double d = Dot(b[i], u);
            minB = std::min(minB, d); maxB = std::max(maxB, d);
        }
        const double forward = minB - maxA, backward = minA - maxB;
        if (forward >= backward) {
            if (forward > best) { best = forward; axis = u; }
        } else if (backward > best) {
            best = backward; axis = u * -1;
        }
    };
    for (const Core* core : {&a, &b}) {
        if (core->size() == 1) continue;
        for (std::size_t i = 0; i < core->edges(); ++i) {
            const P e = core->edgeEnd(i) - core->edgeStart(i);
            test({e.y, -e.x});
            // A point or segment may be apart only along an edge direction.
            if (degenerate) test(e);
        }
    }
    return best;
}
}

bool MakeCcdProxy(const Shape& shape, CcdProxy& proxy) {
    proxy = CcdProxy{};
    if (shape.type == ShapeType::CIRCLE) {
        const auto* circle = dynamic_cast<const Circle*>(&shape);
        if (!circle) return false;
        proxy.count = 1;
        proxy.radius = circle->GetRadius();
        proxy.thickness = proxy.radius;
        return true;
    }
    if (shape.type == ShapeType::CAPSULE) {
        const auto* capsule = dynamic_cast<const Capsule*>(&shape);
        if (!capsule) return false;
        const float h = capsule->GetHalfLength();
        proxy.own = {Vector2{-h, 0}, Vector2{h, 0}};
        proxy.count = h > 0 ? 2 : 1;
        proxy.radius = capsule->GetRadius();
        proxy.reach = h;
        proxy.thickness = proxy.radius;
        return true;
    }
    if (shape.type == ShapeType::POLYGON) {
        const auto* polygon = dynamic_cast<const Polygon*>(&shape);
        if (!polygon) return false;
        const auto& v = polygon->getVertices();
        proxy.vertices = v.data();
        proxy.count = v.size();
        double thickness = std::numeric_limits<double>::infinity();
        for (std::size_t i = 0; i < v.size(); ++i) {
            proxy.reach = std::max(proxy.reach, std::hypot(double(v[i].x), double(v[i].y)));
            const P e = P{v[(i + 1) % v.size()].x, v[(i + 1) % v.size()].y} - P{v[i].x, v[i].y};
            const double length = std::hypot(e.x, e.y);
            if (length == 0) continue;
            const P u{e.y / length, -e.x / length};
            double low = std::numeric_limits<double>::infinity(), high = -low;
            for (const Vector2& w : v) {
                const double d = Dot(P{w.x, w.y}, u);
                low = std::min(low, d); high = std::max(high, d);
            }
            thickness = std::min(thickness, 0.5 * (high - low));
        }
        proxy.thickness = std::isfinite(thickness) ? thickness : 0;
        return true;
    }
    return false;
}

CcdSeparation ComputeSeparation(const CcdProxy& pa, const CcdPose& poseA,
    const CcdProxy& pb, const CcdPose& poseB) {
    const Core a(pa, poseA), b(pb, poseB);
    CcdSeparation s;
    P witnessA, witnessB, normal{1, 0};
    double core;
    if (a.size() == 1 && b.size() == 1) {
        witnessA = a[0]; witnessB = b[0];
        const P d = witnessB - witnessA;
        core = std::hypot(d.x, d.y);
        if (core > 0) normal = d * (1 / core);
    } else {
        core = SatSeparation(a, b, normal);
        if (core > 0) {
            // Apart: the closest pair includes a vertex of one core.
            double best = std::numeric_limits<double>::infinity();
            const auto scan = [&](const Core& points, const Core& edges, bool pointsOnA) {
                for (std::size_t i = 0; i < points.size(); ++i)
                    for (std::size_t j = 0; j < edges.edges(); ++j) {
                        const P q = ClosestOnSegment(points[i], edges.edgeStart(j), edges.edgeEnd(j));
                        const P d = q - points[i];
                        const double distance = std::hypot(d.x, d.y);
                        if (distance < best) {
                            best = distance;
                            witnessA = pointsOnA ? points[i] : q;
                            witnessB = pointsOnA ? q : points[i];
                        }
                    }
            };
            scan(a, b, true);
            scan(b, a, false);
            core = best;
            const P d = witnessB - witnessA;
            if (core > 0) normal = d * (1 / core);
        } else {
            // Overlapping or touching. The witness is the middle of the
            // vertices of either core that lie within the other rounded
            // shape (the overlap region), not a support vertex: on a long
            // wall that vertex can be far away, and the closing speed of a
            // turning body would then be measured at the wrong place.
            P sum{0, 0};
            int inside = 0;
            const double reach = pa.radius + pb.radius;
            const auto collect = [&](const Core& points, const Core& shape) {
                for (std::size_t i = 0; i < points.size(); ++i)
                    if (CoreDistance(points[i], shape) <= reach) { sum = sum + points[i]; ++inside; }
            };
            collect(a, b);
            collect(b, a);
            if (inside > 0) {
                const P middle = sum * (1.0 / inside);
                witnessA = middle - normal * (0.5 * core);
                witnessB = middle + normal * (0.5 * core);
            } else {
                double lowest = std::numeric_limits<double>::infinity();
                for (std::size_t i = 0; i < b.size(); ++i) {
                    const double d = Dot(b[i], normal);
                    if (d < lowest) { lowest = d; witnessB = b[i]; }
                }
                witnessA = witnessB - normal * core;
            }
        }
    }
    s.distance = core - pa.radius - pb.radius;
    s.normalX = normal.x; s.normalY = normal.y;
    const P surfaceA = witnessA + normal * pa.radius, surfaceB = witnessB - normal * pb.radius;
    s.ax = surfaceA.x; s.ay = surfaceA.y; s.bx = surfaceB.x; s.by = surfaceB.y;
    return s;
}

CcdPose PoseAt(const CcdMotion& m, double t) {
    return {m.pose.x + m.vx * t, m.pose.y + m.vy * t, m.pose.angle + m.omega * t};
}

double ClosingSpeed(const CcdSeparation& s, const CcdMotion& ma, const CcdPose& poseA,
    const CcdMotion& mb, const CcdPose& poseB) {
    const double mx = 0.5 * (s.ax + s.bx), my = 0.5 * (s.ay + s.by);
    const double vax = ma.vx - ma.omega * (my - poseA.y), vay = ma.vy + ma.omega * (mx - poseA.x);
    const double vbx = mb.vx - mb.omega * (my - poseB.y), vby = mb.vy + mb.omega * (mx - poseB.x);
    return (vbx - vax) * s.normalX + (vby - vay) * s.normalY;
}

CcdToi TimeOfImpact(const CcdProxy& a, const CcdMotion& ma, const CcdProxy& b, const CcdMotion& mb,
    double tMax, double target, double tolerance, int maxIterations) {
    CcdToi out;
    const double rvx = ma.vx - mb.vx, rvy = ma.vy - mb.vy;
    const double spin = std::abs(ma.omega) * a.reach + std::abs(mb.omega) * b.reach;
    // Bounding circles about the body origins: rotation never moves them.
    {
        const double gap = std::hypot(mb.pose.x - ma.pose.x, mb.pose.y - ma.pose.y)
            - (a.reach + a.radius) - (b.reach + b.radius);
        if (gap - target > std::hypot(rvx, rvy) * tMax + tolerance) return out;
    }
    double t = 0;
    for (int i = 0; i < maxIterations; ++i) {
        ++out.iterations;
        const CcdPose poseA = PoseAt(ma, t), poseB = PoseAt(mb, t);
        out.separation = ComputeSeparation(a, poseA, b, poseB);
        out.time = t;
        if (out.separation.distance <= target + tolerance) {
            out.state = CcdToi::State::Hit;
            return out;
        }
        // The separation along this fixed normal is a lower bound on the
        // distance, and it shrinks no faster than this.
        const double bound = rvx * out.separation.normalX + rvy * out.separation.normalY + spin;
        if (!(bound > 0)) return out;
        t += (out.separation.distance - target) / bound;
        if (!(t < tMax)) return out;
    }
    out.time = t;
    out.separation = ComputeSeparation(a, PoseAt(ma, t), b, PoseAt(mb, t));
    out.state = CcdToi::State::Exhausted;
    return out;
}
}
}

#pragma once
// Shared helpers for the seeded property and fuzz tests: a portable RNG, a
// shape description that can be printed exactly, and an independent geometric
// oracle for circles, convex polygons and capsules.
//
// The oracle treats every shape as a convex core (a point, a segment or a
// polygon) rounded by a radius, evaluated in double from the exact float
// inputs. It shares no code with the engine's narrow phase.

#include "physics/core/rigidbody.h"
#include "physics/core/shape.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace FuzzSupport {
using namespace PhysicsEngine;

// SplitMix64: identical sequences on every platform and standard library,
// unlike std::uniform_real_distribution.
struct Rng {
    std::uint64_t state;
    explicit Rng(std::uint64_t seed) : state(seed) {}
    std::uint64_t Next() {
        std::uint64_t z = (state += 0x9E3779B97F4A7C15ull);
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
        return z ^ (z >> 31);
    }
    double Uniform() { return static_cast<double>(Next() >> 11) * 0x1p-53; }
    double Range(double low, double high) { return low + (high - low) * Uniform(); }
    double LogRange(double low, double high) { return std::exp(Range(std::log(low), std::log(high))); }
    int Integer(int low, int high) { return low + static_cast<int>(Next() % static_cast<std::uint64_t>(high - low + 1)); }
    bool Chance(double p) { return Uniform() < p; }
};

constexpr double Pi = 3.14159265358979323846;

// Positions on a 2^-10 grid are exact floats below 2^14 in magnitude, so a
// shift by an integer vector is exact as long as the result stays below 16384.
inline float Quantize(double value) { return static_cast<float>(std::round(value * 1024.0) / 1024.0); }

inline std::string Hex(float value) {
    char buffer[64];
    std::snprintf(buffer, sizeof buffer, "%a", static_cast<double>(value));
    return buffer;
}
inline std::string Hex(Vector2 v) { return "{" + Hex(v.x) + "f, " + Hex(v.y) + "f}"; }

struct ShapeSpec {
    ShapeType type = ShapeType::CIRCLE;
    float radius = 1, halfLength = 0;
    std::vector<Vector2> vertices;

    std::unique_ptr<Shape> Make() const {
        if (type == ShapeType::CIRCLE) return std::make_unique<Circle>(radius);
        if (type == ShapeType::CAPSULE) return std::make_unique<Capsule>(halfLength, radius);
        return std::make_unique<Polygon>(vertices);
    }
    // Bounding radius about the local origin.
    double Extent() const {
        if (type == ShapeType::CIRCLE) return radius;
        if (type == ShapeType::CAPSULE) return static_cast<double>(halfLength) + radius;
        double result = 0;
        for (const auto& v : vertices) result = std::max(result, std::hypot(double(v.x), double(v.y)));
        return result;
    }
    std::string Describe() const {
        if (type == ShapeType::CIRCLE) return "Circle(" + Hex(radius) + "f)";
        if (type == ShapeType::CAPSULE) return "Capsule(" + Hex(halfLength) + "f, " + Hex(radius) + "f)";
        std::string text = "Polygon({";
        for (std::size_t i = 0; i < vertices.size(); ++i) text += (i ? ", " : "") + Hex(vertices[i]);
        return text + "})";
    }
};

// degenerate=false keeps aspect ratios at or above 1/20 and drops the
// near-coincident vertex, for scenes that must avoid documented limits.
inline std::vector<Vector2> RandomOutline(Rng& rng, bool degenerate = true) {
    for (;;) {
        std::vector<Vector2> result;
        if (rng.Chance(0.15)) {
            const double w = rng.LogRange(0.02, 4), h = rng.Chance(0.3) ? w * rng.LogRange(degenerate ? 1e-3 : 0.05, 0.05) : rng.LogRange(0.02, 4);
            result = {{float(-w / 2), float(-h / 2)}, {float(w / 2), float(-h / 2)}, {float(w / 2), float(h / 2)}, {float(-w / 2), float(h / 2)}};
        } else {
            const int n = rng.Integer(3, 8);
            const double scale = rng.LogRange(0.05, 3);
            // A quarter of the outlines are thin slivers down to 1/1000 aspect.
            double aspect = rng.Chance(0.25) ? rng.LogRange(1e-3, 0.1) : rng.Range(0.3, 1.0);
            if (!degenerate) aspect = std::max(aspect, 0.05);
            std::vector<double> angles(n);
            for (double& a : angles) a = rng.Range(0, 2 * Pi);
            std::sort(angles.begin(), angles.end());
            // Near-degenerate but valid: one vertex almost on top of a neighbour.
            if (n > 3 && rng.Chance(0.2) && degenerate) {
                const int k = rng.Integer(1, n - 1);
                angles[k] = angles[k - 1] + rng.LogRange(1e-4, 1e-2);
                std::sort(angles.begin(), angles.end());
            }
            const double phi = rng.Range(-Pi, Pi), cp = std::cos(phi), sp = std::sin(phi);
            const double ox = rng.Chance(0.5) ? rng.Range(-2, 2) * scale : 0;
            const double oy = rng.Chance(0.5) ? rng.Range(-2, 2) * scale : 0;
            for (double a : angles) {
                const double x = scale * std::cos(a), y = scale * aspect * std::sin(a);
                result.push_back({float(ox + cp * x - sp * y), float(oy + sp * x + cp * y)});
            }
        }
        if (rng.Chance(0.5)) std::reverse(result.begin(), result.end());
        std::rotate(result.begin(), result.begin() + rng.Integer(0, int(result.size()) - 1), result.end());
        try {
            Polygon::ValidateOutline(result);
            return result;
        } catch (const std::invalid_argument&) {
        }
    }
}

// Well-conditioned outlines: 3-8 vertices at jittered even angles on an
// ellipse of aspect 0.3-1, either winding. Rotating them into float world
// coordinates cannot break strict convexity at scene scales.
inline std::vector<Vector2> RegularOutline(Rng& rng) {
    for (;;) {
        const int n = rng.Integer(3, 8);
        const double aspect = rng.Range(0.3, 1.0), start = rng.Range(-Pi, Pi), step = 2 * Pi / n;
        std::vector<Vector2> result;
        for (int k = 0; k < n; ++k) {
            const double a = start + step * (k + rng.Range(-0.25, 0.25));
            result.push_back({float(std::cos(a)), float(aspect * std::sin(a))});
        }
        if (rng.Chance(0.5)) std::reverse(result.begin(), result.end());
        try {
            Polygon::ValidateOutline(result);
            return result;
        } catch (const std::invalid_argument&) {
        }
    }
}

inline ShapeSpec RandomShape(Rng& rng, ShapeType type, bool degenerate = true) {
    ShapeSpec spec;
    spec.type = type;
    if (type == ShapeType::CIRCLE) {
        spec.radius = float(rng.LogRange(0.05, 3));
    } else if (type == ShapeType::CAPSULE) {
        spec.halfLength = rng.Chance(0.2) ? 0.0f : float(rng.LogRange(0.01, 3));
        spec.radius = float(rng.LogRange(0.05, 2));
    } else {
        spec.vertices = RandomOutline(rng, degenerate);
    }
    return spec;
}

struct D2 {
    double x = 0, y = 0;
    D2 operator+(D2 p) const { return {x + p.x, y + p.y}; }
    D2 operator-(D2 p) const { return {x - p.x, y - p.y}; }
    D2 operator*(double s) const { return {x * s, y * s}; }
    double Dot(D2 p) const { return x * p.x + y * p.y; }
    double Cross(D2 p) const { return x * p.y - y * p.x; }
    double Norm() const { return std::hypot(x, y); }
};
inline D2 ToD2(Vector2 v) { return {v.x, v.y}; }

// A convex core rounded by radius r, in world coordinates.
struct Geo {
    std::vector<D2> core; // 1 point, 2 segment endpoints, or >= 3 polygon vertices
    double r = 0;
    bool IsPolygon() const { return core.size() >= 3; }
    double Support(D2 n) const {
        double best = -std::numeric_limits<double>::infinity();
        for (const auto& p : core) best = std::max(best, p.Dot(n));
        return best + r;
    }
    std::size_t EdgeCount() const { return IsPolygon() ? core.size() : 1; }
    std::pair<D2, D2> Edge(std::size_t i) const {
        if (IsPolygon()) return {core[i], core[(i + 1) % core.size()]};
        return {core.front(), core.back()};
    }
    Geo Translated(D2 offset) const {
        Geo result = *this;
        for (auto& p : result.core) p = p + offset;
        return result;
    }
};

inline Geo GeoOf(const ShapeSpec& spec, Vector2 position, float orientation) {
    const double c = std::cos(double(orientation)), s = std::sin(double(orientation));
    const D2 p = ToD2(position);
    Geo geo;
    if (spec.type == ShapeType::CIRCLE) {
        geo.core = {p};
        geo.r = spec.radius;
    } else if (spec.type == ShapeType::CAPSULE) {
        const D2 axis{c, s};
        const double h = spec.halfLength;
        geo.core = h > 0 ? std::vector<D2>{p - axis * h, p + axis * h} : std::vector<D2>{p};
        geo.r = spec.radius;
    } else {
        for (const auto& v : spec.vertices)
            geo.core.push_back(p + D2{c * v.x - s * v.y, s * v.x + c * v.y});
    }
    return geo;
}

inline double PointSegmentDistance(D2 p, D2 a, D2 b) {
    const D2 e = b - a;
    const double length2 = e.Dot(e);
    if (length2 == 0) return (p - a).Norm();
    const double t = std::clamp((p - a).Dot(e) / length2, 0.0, 1.0);
    return (p - (a + e * t)).Norm();
}
inline D2 ClosestOnSegment(D2 p, D2 a, D2 b) {
    const D2 e = b - a;
    const double length2 = e.Dot(e);
    if (length2 == 0) return a;
    return a + e * std::clamp((p - a).Dot(e) / length2, 0.0, 1.0);
}
inline double SegmentSegmentDistance(D2 a, D2 b, D2 c, D2 d) {
    if (!(a.x == b.x && a.y == b.y) && !(c.x == d.x && c.y == d.y)) {
        const double o1 = (b - a).Cross(c - a), o2 = (b - a).Cross(d - a);
        const double o3 = (d - c).Cross(a - c), o4 = (d - c).Cross(b - c);
        if (((o1 < 0 && o2 > 0) || (o1 > 0 && o2 < 0)) && ((o3 < 0 && o4 > 0) || (o3 > 0 && o4 < 0))) return 0;
    }
    return std::min({PointSegmentDistance(a, c, d), PointSegmentDistance(b, c, d),
                     PointSegmentDistance(c, a, b), PointSegmentDistance(d, a, b)});
}
inline bool PolygonContains(const Geo& g, D2 p) {
    double area = 0;
    for (std::size_t i = 0; i < g.core.size(); ++i) area += g.core[i].Cross(g.core[(i + 1) % g.core.size()]);
    const double sign = area > 0 ? 1 : -1;
    for (std::size_t i = 0; i < g.core.size(); ++i) {
        const D2 a = g.core[i], b = g.core[(i + 1) % g.core.size()];
        if (sign * (b - a).Cross(p - a) < 0) return false;
    }
    return true;
}
// Signed distance from a point to the rounded shape (negative inside).
inline double SignedDistance(const Geo& g, D2 p) {
    double edge = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < g.EdgeCount(); ++i) {
        const auto e = g.Edge(i);
        edge = std::min(edge, PointSegmentDistance(p, e.first, e.second));
    }
    const double core = g.IsPolygon() && PolygonContains(g, p) ? -edge : edge;
    return core - g.r;
}
inline double CoreDistance(const Geo& a, const Geo& b) {
    if (a.IsPolygon()) for (const auto& p : b.core) if (PolygonContains(a, p)) return 0;
    if (b.IsPolygon()) for (const auto& p : a.core) if (PolygonContains(b, p)) return 0;
    double best = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < a.EdgeCount(); ++i)
        for (std::size_t j = 0; j < b.EdgeCount(); ++j) {
            const auto ea = a.Edge(i), eb = b.Edge(j);
            best = std::min(best, SegmentSegmentDistance(ea.first, ea.second, eb.first, eb.second));
        }
    return best;
}
// Overlap of the projections when b is pushed along +n: hA(n) + hB(-n).
// Its minimum over unit n is the penetration (minus the gap when separated).
inline double OverlapAlong(const Geo& a, const Geo& b, D2 n) { return a.Support(n) + b.Support(n * -1); }

struct Oracle {
    double penetration; // minimum translation distance, or minus the gap
    bool coresOverlap;
};
inline Oracle Penetration(const Geo& a, const Geo& b) {
    const double coreDistance = CoreDistance(a, b);
    if (coreDistance > 0) return {a.r + b.r - coreDistance, false};
    // Intersecting cores: the Minkowski difference is a polygon containing the
    // origin, so its nearest face is one of the cores' edge normals.
    double best = std::numeric_limits<double>::infinity();
    for (const Geo* g : {&a, &b})
        for (std::size_t i = 0; i < g->EdgeCount(); ++i) {
            const auto e = g->Edge(i);
            const D2 edge = e.second - e.first;
            const double length = edge.Norm();
            if (length == 0) continue;
            const D2 n{-edge.y / length, edge.x / length};
            best = std::min({best, OverlapAlong(a, b, n), OverlapAlong(a, b, n * -1)});
        }
    if (!std::isfinite(best)) best = a.r + b.r; // two coincident points
    return {best, true};
}

inline double Ulp(double magnitude) {
    const float f = static_cast<float>(std::abs(magnitude));
    return double(std::nextafter(f, std::numeric_limits<float>::infinity())) - f;
}

// Collects property violations so one run reports every failing property:
// a count per property (the text before the first ':' or newline) and the
// first examples of each.
struct Failures {
    std::vector<std::pair<std::string, std::size_t>> counts;
    std::vector<std::string> messages;
    std::size_t total = 0;
    void Add(const std::string& message) {
        ++total;
        const std::string key = message.substr(0, message.find_first_of(":\n"));
        std::size_t* count = nullptr;
        for (auto& entry : counts) if (entry.first == key) count = &entry.second;
        if (!count) { counts.push_back({key, 0}); count = &counts.back().second; }
        if (++*count <= 2 && messages.size() < 16) messages.push_back(message);
    }
    std::string Summary() const {
        std::string text = std::to_string(total) + " property violation(s)";
        for (const auto& entry : counts) text += "\n  " + std::to_string(entry.second) + " x " + entry.first;
        for (const auto& m : messages) text += "\n---\n" + m;
        return text;
    }
};
} // namespace FuzzSupport

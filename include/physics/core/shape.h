#ifndef SHAPE_H
#define SHAPE_H

#include <cmath>
#include <vector>
#include <numeric>
#include <algorithm> // For std::reverse if needed
#include <stdexcept>
#include <memory>
#include <limits>
#include "../math/vector2.h"

namespace PhysicsEngine {
    enum class ShapeType {
        CIRCLE,
        POLYGON,
        COUNT
    };

    class Shape {
    public:
        const ShapeType type;
        Shape(ShapeType type) : type(type) {}
        virtual ~Shape() = default;

        virtual float GetArea() const = 0;
        virtual std::unique_ptr<Shape> Clone() const = 0;
        virtual float GetInertia(float mass) const = 0;
        virtual float GetRadius() const { return 0.0f; };
    };

    class Circle : public Shape {
    protected:
        float radius;

    public:
        Circle(float radius) : Shape(ShapeType::CIRCLE), radius(radius) {
            if (!std::isfinite(radius) || radius <= 0.0f) {
                throw std::invalid_argument("Circle radius must be positive and finite.");
            }
        }

        std::unique_ptr<Shape> Clone() const override { return std::make_unique<Circle>(*this); }
        float GetArea() const override { return 3.14159265358979323846f * radius * radius; }
        float GetInertia(float mass) const override { return 0.5f * mass * radius * radius; }
        float GetRadius() const override { return radius; }
    };

    class Polygon : public Shape {
    protected:
        std::vector<Vector2> vertices;

    public:
        Polygon(const std::vector<Vector2>& vertices) : Shape(ShapeType::POLYGON), vertices(vertices) {
            if (vertices.size() < 3) {
                throw std::invalid_argument("Polygon requires at least three vertices.");
            }

            for (const Vector2& vertex : vertices) {
                if (!std::isfinite(vertex.x) || !std::isfinite(vertex.y)) {
                    throw std::invalid_argument("Polygon vertices must be finite.");
                }
            }
            // Every other vertex must lie strictly on the same side of every
            // edge. Local turn tests alone also accept self-intersecting stars.
            // Double precision avoids an absolute area threshold tied to units.
            double winding = 0;
            for (size_t i = 0; i < vertices.size(); ++i) {
                const auto& a = vertices[i];
                const auto& b = vertices[(i + 1) % vertices.size()];
                const double dx = static_cast<double>(b.x) - a.x;
                const double dy = static_cast<double>(b.y) - a.y;
                for (size_t j = 0; j < vertices.size(); ++j) {
                    if (j == i || j == (i + 1) % vertices.size()) continue;
                    const double cross = dx * (static_cast<double>(vertices[j].y) - a.y)
                        - dy * (static_cast<double>(vertices[j].x) - a.x);
                    if (cross == 0 || (winding != 0 && (cross > 0) != (winding > 0)))
                        throw std::invalid_argument("Polygon must be simple, strictly convex and non-degenerate.");
                    winding = cross;
                }
            }
        }

        // --- NEW: Static Factories to replace old Classes ---
        std::unique_ptr<Shape> Clone() const override { return std::make_unique<Polygon>(*this); }
        
        static Polygon MakeBox(float width, float height) {
            if (!std::isfinite(width) || !std::isfinite(height)
                || width <= 0.0f || height <= 0.0f) {
                throw std::invalid_argument("Box dimensions must be positive and finite.");
            }
            // Create a centered box (CCW order)
            float hw = width / 2.0f;
            float hh = height / 2.0f;
            return Polygon({
                Vector2(-hw, -hh), // Bottom-Left
                Vector2(hw, -hh),  // Bottom-Right
                Vector2(hw, hh),   // Top-Right
                Vector2(-hw, hh)   // Top-Left
            });
        }

        static Polygon MakeTriangle(Vector2 p1, Vector2 p2, Vector2 p3) {
            return Polygon({ p1, p2, p3 });
        }

        // ----------------------------------------------------

        float GetArea() const override {
            double doubleArea = 0.0;
            size_t count = vertices.size();
            for (size_t i = 0; i < count; ++i) {
                Vector2 p1 = vertices[i];
                Vector2 p2 = vertices[(i + 1) % count];
                doubleArea += static_cast<double>(p1.x) * p2.y - static_cast<double>(p1.y) * p2.x;
            }
            return static_cast<float>(std::abs(doubleArea) * 0.5);
        }

        float GetInertia(float mass) const override {
            double numerator = 0.0;
            double denominator = 0.0;
            size_t count = vertices.size();
            if (count < 3) return 0.0f;

            for (size_t i = 0; i < count; ++i) {
                Vector2 p1 = vertices[i];
                Vector2 p2 = vertices[(i + 1) % count];
                const double cross = static_cast<double>(p1.x) * p2.y - static_cast<double>(p1.y) * p2.x;
                const double intTerm = static_cast<double>(p1.x) * p1.x + static_cast<double>(p1.y) * p1.y
                    + static_cast<double>(p1.x) * p2.x + static_cast<double>(p1.y) * p2.y
                    + static_cast<double>(p2.x) * p2.x + static_cast<double>(p2.y) * p2.y;
                numerator += cross * intTerm;
                denominator += cross;
            }
            if (denominator == 0.0f) return 0.0f;
            return static_cast<float>((mass / 6.0) * (numerator / denominator));
        }

        const std::vector<Vector2>& getVertices() const { return vertices; }

        // Uniform-area centroid in the original local frame. Triangulating
        // relative to a stored vertex avoids subtracting large world moments.
        Vector2 GetCentroid() const {
            const auto& origin = vertices.front();
            double twiceArea = 0, momentX = 0, momentY = 0;
            for (size_t i = 1; i + 1 < vertices.size(); ++i) {
                const double ax = static_cast<double>(vertices[i].x) - origin.x;
                const double ay = static_cast<double>(vertices[i].y) - origin.y;
                const double bx = static_cast<double>(vertices[i + 1].x) - origin.x;
                const double by = static_cast<double>(vertices[i + 1].y) - origin.y;
                const double cross = ax * by - ay * bx;
                twiceArea += cross;
                momentX += cross * (ax + bx);
                momentY += cross * (ay + by);
            }
            const double x = origin.x + momentX / (3 * twiceArea);
            const double y = origin.y + momentY / (3 * twiceArea);
            const double maximum = std::numeric_limits<float>::max();
            if (!std::isfinite(x) || !std::isfinite(y) || std::abs(x) > maximum || std::abs(y) > maximum)
                throw std::overflow_error("Polygon centroid exceeds finite Vector2 range.");
            return {static_cast<float>(x), static_cast<float>(y)};
        }

        // Preserve this outline and return a copy shifted by GetCentroid().
        // Shape validation still applies after rounding the new float vertices.
        Polygon Recentered() const {
            const Vector2 center = GetCentroid();
            std::vector<Vector2> shifted;
            shifted.reserve(vertices.size());
            const double maximum = std::numeric_limits<float>::max();
            for (const auto& vertex : vertices) {
                const double x = static_cast<double>(vertex.x) - center.x;
                const double y = static_cast<double>(vertex.y) - center.y;
                if (std::abs(x) > maximum || std::abs(y) > maximum)
                    throw std::overflow_error("Recentered polygon exceeds finite Vector2 range.");
                shifted.emplace_back(static_cast<float>(x), static_cast<float>(y));
            }
            return Polygon(shifted);
        }
    };
}

#endif // SHAPE_H

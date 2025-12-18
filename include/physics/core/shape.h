#ifndef SHAPE_H
#define SHAPE_H

#include <cmath>
#include <vector>
#include <numeric>
#include <algorithm> // For std::reverse if needed
#include "../math/vector2.h"

namespace PhysicsEngine {
    enum class ShapeType {
        CIRCLE,
        POLYGON,
        COUNT
    };

    class Shape {
    public:
        ShapeType type;
        Shape(ShapeType type) : type(type) {}
        virtual ~Shape() = default;

        virtual float GetArea() const = 0;
        virtual float GetInertia(float mass) const = 0;
        virtual float GetRadius() const { return 0.0f; };
    };

    class Circle : public Shape {
    protected:
        float radius;

    public:
        Circle(float radius) : Shape(ShapeType::CIRCLE), radius(radius) {}

        float GetArea() const override { return M_PI * radius * radius; }
        float GetInertia(float mass) const override { return 0.5f * mass * radius * radius; }
        float GetRadius() const override { return radius; }
    };

    class Polygon : public Shape {
    protected:
        std::vector<Vector2> vertices;

    public:
        Polygon(const std::vector<Vector2>& vertices) : Shape(ShapeType::POLYGON), vertices(vertices) {}

        // --- NEW: Static Factories to replace old Classes ---
        
        static Polygon MakeBox(float width, float height) {
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
            float doubleArea = 0.0f;
            size_t count = vertices.size();
            for (size_t i = 0; i < count; ++i) {
                Vector2 p1 = vertices[i];
                Vector2 p2 = vertices[(i + 1) % count];
                doubleArea += (p1.x * p2.y - p1.y * p2.x);
            }
            return std::abs(doubleArea) * 0.5f;
        }

        float GetInertia(float mass) const override {
            float numerator = 0.0f;
            float denominator = 0.0f;
            size_t count = vertices.size();
            if (count < 3) return 0.0f;

            for (size_t i = 0; i < count; ++i) {
                Vector2 p1 = vertices[i];
                Vector2 p2 = vertices[(i + 1) % count];
                float cross = std::abs(p1.x * p2.y - p1.y * p2.x);
                float intTerm = (p1.magnitudeSquared() + p1.dot(p2) + p2.magnitudeSquared());
                numerator += cross * intTerm;
                denominator += cross;
            }
            if (denominator == 0.0f) return 0.0f;
            return (mass / 6.0f) * (numerator / denominator);
        }

        const std::vector<Vector2>& getVertices() const { return vertices; }
    };
}

#endif // SHAPE_H
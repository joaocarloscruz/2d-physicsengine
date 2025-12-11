#ifndef SHAPE_H
#define SHAPE_H


#include <cmath>

namespace PhysicsEngine {
    enum class ShapeType {
        CIRCLE,
        RECTANGLE,
        TRIANGLE,
        COUNT
    };

    class Shape {
    public:
        ShapeType type;
        Shape(ShapeType type) : type(type) {}
        virtual ~Shape() = default;

        // Each shape implements these methods
        virtual float GetArea() const = 0;
        virtual float GetInertia(float mass) const = 0;
        virtual float GetWidth() const { return 0.0f; };
        virtual float GetHeight() const { return 0.0f; };
        virtual float GetRadius() const { return 0.0f; };
    };

    class Circle : public Shape {
    protected:
        float radius;

    public:
        Circle(float radius) : Shape(ShapeType::CIRCLE), radius(radius) {}

        float GetArea() const override { // pi r^2
            return M_PI * radius * radius;
        }

        float GetInertia(float mass) const override { // m r^2 / 2
            return 0.5f * mass * radius * radius;
        }

        float GetRadius() const override {
            return radius;
        }
    };

    class Rectangle : public Shape {
    protected:
        float width, height;

    public:
        Rectangle(float width, float height) : Shape(ShapeType::RECTANGLE), width(width), height(height) {}

        float GetArea() const override { // w h
            return width * height;
        }

        float GetInertia(float mass) const override { // m (w^2 + h^2) / 12
            return (mass * (width * width + height * height)) / 12.0f;
        }

        float GetWidth() const override {
            return width;
        }

        float GetHeight() const override {
            return height;
        }
    };

    class Triangle : public Shape {
    protected:
        float base, height;

    public:
        Triangle(float base, float height) : Shape(ShapeType::TRIANGLE), base(base), height(height) {}

        float GetArea() const override { // 0.5 * b * h
            return 0.5f * base * height;
        }

        float GetInertia(float mass) const override { // m (b^2 / 24 + h^2 / 18)
            return mass * ((base * base) / 24.0f + (height * height) / 18.0f);
        }

        float GetWidth() const override {
            return base;
        }

        float GetHeight() const override {
            return height;
        }

        float GetBase() const {
            return base;
        }
    };
}

#endif // SHAPE_H
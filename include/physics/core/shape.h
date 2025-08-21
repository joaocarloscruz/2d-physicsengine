#ifndef SHAPE_H
#define SHAPE_H

#define M_PI 3.14159265358979323846

enum class ShapeType {
    CIRCLE,
    RECTANGLE,
    TRIANGLE
};

class Shape {
public:
    ShapeType type;
    Shape(ShapeType type) : type(type) {}
    virtual ~Shape() = default;

    // Each shape implements these methods
    virtual float GetArea() const = 0;
    virtual float GetInertia(float mass) const = 0;
};

class Circle : public Shape {
public:
    float radius;

    Circle(float radius) : Shape(ShapeType::CIRCLE), radius(radius) {}

    float GetArea() const override { // pi r^2
        return M_PI * radius * radius;
    }

    float GetInertia(float mass) const override { // m r^2 / 2
        return 0.5f * mass * radius * radius;
    }
};

class Rectangle : public Shape {
public:
    float width, height;

    Rectangle(float width, float height) : Shape(ShapeType::RECTANGLE), width(width), height(height) {}

    float GetArea() const override { // w h
        return width * height;
    }

    float GetInertia(float mass) const override { // m (w^2 + h^2) / 12
        return (mass * (width * width + height * height)) / 12.0f;
    }
};

class Triangle : public Shape {
public:
    float height, base;

    Triangle(float height, float base) : Shape(ShapeType::TRIANGLE), height(height), base(base) {}

    float GetArea() const override { // 0.5 * b * h
        return 0.5f * base * height;
    }

    float GetInertia(float mass) const override { // m b h^2 / 12
        return (mass * base * height * height) / 12.0f;
    }
};

#endif // SHAPE_H
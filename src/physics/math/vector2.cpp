#include "physics/math/vector2.h"

namespace PhysicsEngine {

    Vector2::Vector2(float x, float y) : x(x), y(y) {}

    Vector2 Vector2::operator+(const Vector2& other) const {
        return Vector2(x + other.x, y + other.y);
    }

    Vector2 Vector2::operator-(const Vector2& other) const {
        return Vector2(x - other.x, y - other.y);
    }

    Vector2 Vector2::operator*(float scalar) const {
        return Vector2(x * scalar, y * scalar);
    }

    Vector2 Vector2::operator/(float scalar) const {
        return Vector2(x / scalar, y / scalar);
    }

    float Vector2::magnitude() const {
        return sqrt(x * x + y * y);
    }

    float Vector2::magnitudeSquared() const {
        return x * x + y * y;
    }

    Vector2 Vector2::normalized() const {
        float mag = magnitude();
        if (mag > 0) {
            return Vector2(x / mag, y / mag);
        }
        return Vector2(0.0f, 0.0f);
    }

    float Vector2::dot(const Vector2& other) const {
        return x * other.x + y * other.y;
    }

}
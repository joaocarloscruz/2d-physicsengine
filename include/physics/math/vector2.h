#ifndef VECTOR2_H
#define VECTOR2_H

#include <cmath>

namespace PhysicsEngine {
    class Vector2 {
    public:
        float x, y;

        Vector2(float x = 0.0f, float y = 0.0f);

        Vector2 operator+(const Vector2& other) const;
        Vector2 operator-(const Vector2& other) const;
        Vector2 operator*(float scalar) const;
        Vector2 operator/(float scalar) const;
        bool operator==(const Vector2& other) const;

        float magnitude() const;
        float magnitudeSquared() const;
        Vector2 normalized() const;
        float dot(const Vector2& other) const;
    };
}

#endif
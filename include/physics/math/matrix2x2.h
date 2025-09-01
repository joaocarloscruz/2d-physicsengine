#ifndef MATRIX2X2_H
#define MATRIX2X2_H

#include "vector2.h"

namespace PhysicsEngine {
    class Matrix2x2 {
    public:
        float m[2][2];

        Matrix2x2();
        Matrix2x2(float m00, float m01, float m10, float m11);

        static Matrix2x2 rotation(float angle);

        Vector2 operator*(const Vector2& v) const;
        Matrix2x2 operator*(const Matrix2x2& other) const;

        float determinant() const;
        Matrix2x2 inverse() const;
    };
}

#endif
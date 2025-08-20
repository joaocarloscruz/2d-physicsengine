#include "../include/physics/math/matrix2x2.h"
#include <cmath>

Matrix2x2::Matrix2x2() {
    m[0][0] = 0.0f; m[0][1] = 0.0f;
    m[1][0] = 0.0f; m[1][1] = 0.0f;
}

Matrix2x2::Matrix2x2(float m00, float m01, float m10, float m11) {
    m[0][0] = m00; m[0][1] = m01;
    m[1][0] = m10; m[1][1] = m11;
}

Matrix2x2 Matrix2x2::rotation(float angle) {
    float cos_a = cos(angle);
    float sin_a = sin(angle);
    return Matrix2x2(cos_a, -sin_a, sin_a, cos_a);
}

Vector2 Matrix2x2::operator*(const Vector2& v) const {
    return Vector2(m[0][0] * v.x + m[0][1] * v.y, m[1][0] * v.x + m[1][1] * v.y);
}

Matrix2x2 Matrix2x2::operator*(const Matrix2x2& other) const {
    return Matrix2x2(
        m[0][0] * other.m[0][0] + m[0][1] * other.m[1][0],
        m[0][0] * other.m[0][1] + m[0][1] * other.m[1][1],
        m[1][0] * other.m[0][0] + m[1][1] * other.m[1][0],
        m[1][0] * other.m[0][1] + m[1][1] * other.m[1][1]
    );
}

float Matrix2x2::determinant() const {
    return m[0][0] * m[1][1] - m[0][1] * m[1][0];
}

Matrix2x2 Matrix2x2::inverse() const {
    float det = determinant();
    if (det == 0.0f) {
        return Matrix2x2();
    }
    float inv_det = 1.0f / det;
    return Matrix2x2(m[1][1] * inv_det, -m[0][1] * inv_det, -m[1][0] * inv_det, m[0][0] * inv_det);
}
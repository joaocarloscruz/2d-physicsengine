#pragma once

#include "physics/math/vector2.h"
#include <cmath>
#include <limits>
#include <stdexcept>

namespace PhysicsEngine::ForceArithmetic {
inline Vector2 CheckedForce(double x, double y) {
    const double maximum = std::numeric_limits<float>::max();
    if (!std::isfinite(x) || !std::isfinite(y) || std::abs(x) > maximum || std::abs(y) > maximum)
        throw std::overflow_error("Generated force exceeds finite float range.");
    return {static_cast<float>(x), static_cast<float>(y)};
}
}

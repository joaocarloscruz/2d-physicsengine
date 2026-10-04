#pragma once
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>

namespace PhysicsEngine::Wasm {
// Receive JS numbers as doubles: Embind's size_t conversion would truncate or
// wrap before native validation sees the original value.
inline std::size_t Count(double value) {
    if (!std::isfinite(value) || value<0 || std::floor(value)!=value ||
        value>=std::ldexp(1.0,std::numeric_limits<std::size_t>::digits))
        throw std::invalid_argument("Count must be an exactly representable nonnegative integer");
    return static_cast<std::size_t>(value);
}
inline std::size_t Index(double value, std::size_t count) {
    const auto index=Count(value);
    if (index>=count) throw std::out_of_range("Element index out of range");
    return index;
}
}

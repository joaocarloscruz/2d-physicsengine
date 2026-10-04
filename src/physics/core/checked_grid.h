#ifndef PHYSICS_CHECKED_GRID_H
#define PHYSICS_CHECKED_GRID_H

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <utility>

namespace PhysicsEngine::CheckedGrid {

// Bound empty-cell traversal and sampling independently of allocation limits.
constexpr std::uint64_t MaximumGridVisits = 16'777'216;
constexpr std::uint64_t MaximumSamples = 1'048'576;

inline void PositiveFinite(float value) {
    if (!std::isfinite(value) || value <= 0.0f) {
        throw std::invalid_argument("Grid and sampling dimensions must be positive and finite.");
    }
}

inline int Integer(double value) {
    if (!std::isfinite(value)
        || value < std::numeric_limits<int>::min()
        || value > std::numeric_limits<int>::max()) {
        throw std::overflow_error("Grid coordinate or sample count exceeds integer range.");
    }
    return static_cast<int>(value);
}

inline int Coordinate(float value, float cellSize) {
    if (!std::isfinite(value)) {
        throw std::invalid_argument("Grid positions must be finite.");
    }
    PositiveFinite(cellSize);
    return Integer(std::floor(static_cast<double>(value) / cellSize));
}

inline int Extent(float radius, float cellSize) {
    PositiveFinite(radius);
    PositiveFinite(cellSize);
    return Integer(std::ceil(static_cast<double>(radius) / cellSize));
}

inline void Charge(std::uint64_t amount, std::uint64_t& remaining) {
    if (amount > remaining) {
        throw std::length_error("Grid or sampling operation exceeds its work budget.");
    }
    remaining -= amount;
}

struct Window {
    std::int64_t minX, maxX, minY, maxY;
};

inline Window Around(const std::pair<int, int>& origin, int extent) {
    const std::int64_t low = std::numeric_limits<int>::min();
    const std::int64_t high = std::numeric_limits<int>::max();
    return {
        std::max(low, static_cast<std::int64_t>(origin.first) - extent),
        std::min(high, static_cast<std::int64_t>(origin.first) + extent),
        std::max(low, static_cast<std::int64_t>(origin.second) - extent),
        std::min(high, static_cast<std::int64_t>(origin.second) + extent)
    };
}

inline void Charge(const Window& window, std::uint64_t& remaining) {
    if (window.minX > window.maxX || window.minY > window.maxY) {
        throw std::invalid_argument("Grid bounds must be ordered.");
    }
    const auto width = static_cast<std::uint64_t>(window.maxX - window.minX + 1);
    const auto height = static_cast<std::uint64_t>(window.maxY - window.minY + 1);
    // Divide before multiplying: a full integer-coordinate plane exceeds uint64_t.
    if (width > remaining / height) {
        throw std::length_error("Grid operation exceeds its cell visit budget.");
    }
    Charge(width * height, remaining);
}

inline bool WithinRadius(float firstX, float firstY, float secondX,
                         float secondY, float radius) {
    const double dx = static_cast<double>(secondX) - firstX;
    const double dy = static_cast<double>(secondY) - firstY;
    return dx * dx + dy * dy <= static_cast<double>(radius) * radius;
}

} // namespace PhysicsEngine::CheckedGrid

#endif

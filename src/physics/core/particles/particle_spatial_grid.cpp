#include "physics/core/particles/particle_spatial_grid.h"

#include "../checked_grid.h"

#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>

namespace PhysicsEngine {

ParticleSpatialGrid::ParticleSpatialGrid(float gridCellSize) : cellSize(gridCellSize) {
    if (!std::isfinite(cellSize) || cellSize <= 0.0f) {
        throw std::invalid_argument("Particle grid cell size must be positive and finite.");
    }
}

std::size_t ParticleSpatialGrid::CellKeyHash::operator()(const CellKey& key) const {
    const std::size_t xHash = std::hash<int>{}(key.first);
    const std::size_t yHash = std::hash<int>{}(key.second);
    return xHash ^ (yHash << 1);
}

ParticleSpatialGrid::CellKey ParticleSpatialGrid::getCell(const Vector2& position) const {
    return {
        CheckedGrid::Coordinate(position.x, cellSize),
        CheckedGrid::Coordinate(position.y, cellSize),
    };
}

void ParticleSpatialGrid::rebuild(const std::vector<Particle>& particles) {
    decltype(cells) rebuiltCells;
    for (std::size_t index = 0; index < particles.size(); ++index) {
        rebuiltCells[getCell(particles[index].position)].push_back(index);
    }
    cells.swap(rebuiltCells);
}

std::vector<ParticleSpatialGrid::ParticlePair> ParticleSpatialGrid::findPotentialPairs(
    const std::vector<Particle>& particles,
    float interactionRadius
) const {
    if (!std::isfinite(interactionRadius) || interactionRadius <= 0.0f) {
        throw std::invalid_argument("Particle interaction radius must be positive and finite.");
    }

    const int cellRange = CheckedGrid::Extent(interactionRadius, cellSize);
    std::uint64_t remainingVisits = CheckedGrid::MaximumGridVisits;
    std::vector<CheckedGrid::Window> windows;
    windows.reserve(particles.size());
    for (const auto& particle : particles) {
        const auto window = CheckedGrid::Around(getCell(particle.position), cellRange);
        CheckedGrid::Charge(window, remainingVisits);
        windows.push_back(window);
    }
    std::set<ParticlePair> uniquePairs;

    for (std::size_t firstIndex = 0; firstIndex < particles.size(); ++firstIndex) {
        const auto& window = windows[firstIndex];
        for (std::int64_t x = window.minX; x <= window.maxX; ++x) {
            for (std::int64_t y = window.minY; y <= window.maxY; ++y) {
                const auto cell = cells.find({static_cast<int>(x), static_cast<int>(y)});
                if (cell == cells.end()) {
                    continue;
                }

                for (std::size_t secondIndex : cell->second) {
                    if (secondIndex <= firstIndex || secondIndex >= particles.size()) {
                        continue;
                    }

                    if (CheckedGrid::WithinRadius(
                        particles[firstIndex].position.x, particles[firstIndex].position.y,
                        particles[secondIndex].position.x, particles[secondIndex].position.y,
                        interactionRadius)) {
                        uniquePairs.emplace(firstIndex, secondIndex);
                    }
                }
            }
        }
    }

    return {uniquePairs.begin(), uniquePairs.end()};
}

float ParticleSpatialGrid::getCellSize() const {
    return cellSize;
}

}

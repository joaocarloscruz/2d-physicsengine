#include "physics/core/particles/particle_spatial_grid.h"

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
        static_cast<int>(std::floor(position.x / cellSize)),
        static_cast<int>(std::floor(position.y / cellSize)),
    };
}

void ParticleSpatialGrid::rebuild(const std::vector<Particle>& particles) {
    cells.clear();
    for (std::size_t index = 0; index < particles.size(); ++index) {
        cells[getCell(particles[index].position)].push_back(index);
    }
}

std::vector<ParticleSpatialGrid::ParticlePair> ParticleSpatialGrid::findPotentialPairs(
    const std::vector<Particle>& particles,
    float interactionRadius
) const {
    if (!std::isfinite(interactionRadius) || interactionRadius <= 0.0f) {
        throw std::invalid_argument("Particle interaction radius must be positive and finite.");
    }

    const int cellRange = static_cast<int>(std::ceil(interactionRadius / cellSize));
    const float radiusSquared = interactionRadius * interactionRadius;
    std::set<ParticlePair> uniquePairs;

    for (std::size_t firstIndex = 0; firstIndex < particles.size(); ++firstIndex) {
        const CellKey origin = getCell(particles[firstIndex].position);
        for (int x = origin.first - cellRange; x <= origin.first + cellRange; ++x) {
            for (int y = origin.second - cellRange; y <= origin.second + cellRange; ++y) {
                const auto cell = cells.find({x, y});
                if (cell == cells.end()) {
                    continue;
                }

                for (std::size_t secondIndex : cell->second) {
                    if (secondIndex <= firstIndex || secondIndex >= particles.size()) {
                        continue;
                    }

                    const Vector2 offset = particles[secondIndex].position
                        - particles[firstIndex].position;
                    if (offset.magnitudeSquared() <= radiusSquared) {
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

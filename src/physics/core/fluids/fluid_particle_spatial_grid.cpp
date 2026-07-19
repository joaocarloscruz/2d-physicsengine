#include "physics/core/fluids/fluid_particle_spatial_grid.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace PhysicsEngine {

FluidParticleSpatialGrid::FluidParticleSpatialGrid(float gridCellSize)
    : cellSize(gridCellSize) {
    if (!std::isfinite(cellSize) || cellSize <= 0.0f) {
        throw std::invalid_argument(
            "Fluid grid cell size must be positive and finite."
        );
    }
}

std::size_t FluidParticleSpatialGrid::CellKeyHash::operator()(
    const CellKey& key
) const {
    const std::size_t xHash = std::hash<int>{}(key.first);
    const std::size_t yHash = std::hash<int>{}(key.second);
    return xHash ^ (yHash + 0x9e3779b9u + (xHash << 6) + (xHash >> 2));
}

FluidParticleSpatialGrid::CellKey FluidParticleSpatialGrid::getCell(
    const Vector2& position
) const {
    return {
        static_cast<int>(std::floor(position.x / cellSize)),
        static_cast<int>(std::floor(position.y / cellSize)),
    };
}

void FluidParticleSpatialGrid::rebuild(
    const std::vector<FluidParticle>& particles
) {
    cells.clear();
    cells.reserve(particles.size());
    for (std::size_t index = 0; index < particles.size(); ++index) {
        cells[getCell(particles[index].position)].push_back(index);
    }
    lastStatistics = FluidNeighborStatistics{};
    lastStatistics.particleCount = particles.size();
    lastStatistics.occupiedCellCount = cells.size();
}

std::vector<FluidParticleSpatialGrid::ParticlePair>
FluidParticleSpatialGrid::findNeighborPairs(
    const std::vector<FluidParticle>& particles,
    float interactionRadius
) {
    if (!std::isfinite(interactionRadius) || interactionRadius <= 0.0f) {
        throw std::invalid_argument(
            "Fluid interaction radius must be positive and finite."
        );
    }
    if (particles.size() != lastStatistics.particleCount) {
        throw std::invalid_argument(
            "Fluid grid must be rebuilt after the particle count changes."
        );
    }

    lastStatistics.candidatePairCount = 0;
    lastStatistics.neighborPairCount = 0;
    lastStatistics.maximumNeighborCount = 0;
    const int cellRange = static_cast<int>(
        std::ceil(interactionRadius / cellSize)
    );
    const float radiusSquared = interactionRadius * interactionRadius;
    std::vector<ParticlePair> pairs;
    pairs.reserve(pairCapacityHint);
    std::vector<std::size_t> neighborCounts(particles.size(), 0);

    for (std::size_t first = 0; first < particles.size(); ++first) {
        const std::size_t firstPair = pairs.size();
        const CellKey origin = getCell(particles[first].position);
        for (int x = origin.first - cellRange;
             x <= origin.first + cellRange;
             ++x) {
            for (int y = origin.second - cellRange;
                 y <= origin.second + cellRange;
                 ++y) {
                const auto cell = cells.find({x, y});
                if (cell == cells.end()) {
                    continue;
                }
                for (std::size_t second : cell->second) {
                    if (second <= first || second >= particles.size()) {
                        continue;
                    }
                    ++lastStatistics.candidatePairCount;
                    const Vector2 offset = particles[second].position
                        - particles[first].position;
                    if (offset.magnitudeSquared() <= radiusSquared) {
                        pairs.emplace_back(first, second);
                        ++neighborCounts[first];
                        ++neighborCounts[second];
                    }
                }
            }
        }
        std::sort(pairs.begin() + firstPair, pairs.end());
    }

    pairCapacityHint = std::max(pairCapacityHint, pairs.size());
    lastStatistics.neighborPairCount = pairs.size();
    if (!neighborCounts.empty()) {
        lastStatistics.maximumNeighborCount = *std::max_element(
            neighborCounts.begin(),
            neighborCounts.end()
        );
    }
    return pairs;
}

float FluidParticleSpatialGrid::getCellSize() const {
    return cellSize;
}

const FluidNeighborStatistics&
FluidParticleSpatialGrid::getLastStatistics() const {
    return lastStatistics;
}

}

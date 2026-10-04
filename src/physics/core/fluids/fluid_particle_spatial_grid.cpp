#include "physics/core/fluids/fluid_particle_spatial_grid.h"

#include "../checked_grid.h"

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
        CheckedGrid::Coordinate(position.x, cellSize),
        CheckedGrid::Coordinate(position.y, cellSize),
    };
}

void FluidParticleSpatialGrid::rebuild(
    const std::vector<FluidParticle>& particles
) {
    decltype(cells) rebuiltCells;
    rebuiltCells.reserve(particles.size());
    for (std::size_t index = 0; index < particles.size(); ++index) {
        rebuiltCells[getCell(particles[index].position)].push_back(index);
    }
    cells.swap(rebuiltCells);
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

    auto statistics = lastStatistics;
    statistics.candidatePairCount = 0;
    statistics.neighborPairCount = 0;
    statistics.maximumNeighborCount = 0;
    const int cellRange = CheckedGrid::Extent(interactionRadius, cellSize);
    std::uint64_t remainingVisits = CheckedGrid::MaximumGridVisits;
    std::vector<CheckedGrid::Window> windows;
    windows.reserve(particles.size());
    for (const auto& particle : particles) {
        const auto window = CheckedGrid::Around(getCell(particle.position), cellRange);
        CheckedGrid::Charge(window, remainingVisits);
        windows.push_back(window);
    }
    std::vector<ParticlePair> pairs;
    pairs.reserve(pairCapacityHint);
    std::vector<std::size_t> neighborCounts(particles.size(), 0);

    for (std::size_t first = 0; first < particles.size(); ++first) {
        const std::size_t firstPair = pairs.size();
        const auto& window = windows[first];
        for (std::int64_t x = window.minX; x <= window.maxX; ++x) {
            for (std::int64_t y = window.minY; y <= window.maxY; ++y) {
                const auto cell = cells.find({static_cast<int>(x), static_cast<int>(y)});
                if (cell == cells.end()) {
                    continue;
                }
                for (std::size_t second : cell->second) {
                    if (second <= first || second >= particles.size()) {
                        continue;
                    }
                    ++statistics.candidatePairCount;
                    if (CheckedGrid::WithinRadius(
                        particles[first].position.x, particles[first].position.y,
                        particles[second].position.x, particles[second].position.y,
                        interactionRadius)) {
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
    statistics.neighborPairCount = pairs.size();
    if (!neighborCounts.empty()) {
        statistics.maximumNeighborCount = *std::max_element(
            neighborCounts.begin(),
            neighborCounts.end()
        );
    }
    lastStatistics = statistics;
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

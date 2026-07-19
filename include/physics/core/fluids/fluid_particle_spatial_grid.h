#ifndef FLUID_PARTICLE_SPATIAL_GRID_H
#define FLUID_PARTICLE_SPATIAL_GRID_H

#include "fluid_particle.h"

#include <cstddef>
#include <unordered_map>
#include <utility>
#include <vector>

namespace PhysicsEngine {

struct FluidNeighborStatistics {
    std::size_t particleCount = 0;
    std::size_t occupiedCellCount = 0;
    std::size_t candidatePairCount = 0;
    std::size_t neighborPairCount = 0;
    std::size_t maximumNeighborCount = 0;
};

class FluidParticleSpatialGrid {
public:
    using ParticlePair = std::pair<std::size_t, std::size_t>;

    explicit FluidParticleSpatialGrid(float cellSize);

    void rebuild(const std::vector<FluidParticle>& particles);
    std::vector<ParticlePair> findNeighborPairs(
        const std::vector<FluidParticle>& particles,
        float interactionRadius
    );

    float getCellSize() const;
    const FluidNeighborStatistics& getLastStatistics() const;

private:
    using CellKey = std::pair<int, int>;

    struct CellKeyHash {
        std::size_t operator()(const CellKey& key) const;
    };

    CellKey getCell(const Vector2& position) const;

    float cellSize;
    std::unordered_map<CellKey, std::vector<std::size_t>, CellKeyHash> cells;
    FluidNeighborStatistics lastStatistics;
    std::size_t pairCapacityHint = 0;
};

}

#endif // FLUID_PARTICLE_SPATIAL_GRID_H

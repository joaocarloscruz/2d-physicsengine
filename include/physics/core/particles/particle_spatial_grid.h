#ifndef PARTICLE_SPATIAL_GRID_H
#define PARTICLE_SPATIAL_GRID_H

#include <cstddef>
#include <unordered_map>
#include <utility>
#include <vector>

#include "particle.h"

namespace PhysicsEngine {

class ParticleSpatialGrid {
public:
    using ParticlePair = std::pair<std::size_t, std::size_t>;

    explicit ParticleSpatialGrid(float cellSize);

    void rebuild(const std::vector<Particle>& particles);
    std::vector<ParticlePair> findPotentialPairs(
        const std::vector<Particle>& particles,
        float interactionRadius
    ) const;
    float getCellSize() const;

private:
    using CellKey = std::pair<int, int>;

    struct CellKeyHash {
        std::size_t operator()(const CellKey& key) const;
    };

    CellKey getCell(const Vector2& position) const;

    float cellSize;
    std::unordered_map<CellKey, std::vector<std::size_t>, CellKeyHash> cells;
};

}

#endif // PARTICLE_SPATIAL_GRID_H

#include "physics/core/collisions/broad_phase/uniform_grid.h"
#include "physics/core/types.h"
#include <algorithm>
#include <cmath>
#include <set>
#include <unordered_set>

namespace PhysicsEngine {

    UniformGrid::UniformGrid(float cellSize) : cellSize(cellSize) {
        if (this->cellSize <= 0.0f) {
            this->cellSize = 100.0f; // fallback
        }
    }

    float UniformGrid::getCellSize() const {
        return cellSize;
    }

    void UniformGrid::setCellSize(float size) {
        if (size > 0.0f) {
            cellSize = size;
        }
    }

    UniformGrid::CellKey UniformGrid::GetCellCoords(const Vector2& pos) const {
        return {
            static_cast<int>(std::floor(pos.x / cellSize)),
            static_cast<int>(std::floor(pos.y / cellSize))
        };
    }

    std::vector<CollisionPair> UniformGrid::FindPotentialCollisions(const std::vector<RigidBodyPtr>& bodies) {
        std::vector<CollisionPair> potentialCollisions;
        if (bodies.size() < 2) {
            return potentialCollisions;
        }

        GridMap grid;

        // Populate grid
        for (const auto& body : bodies) {
            AABB aabb = body->GetAABB();
            
            CellKey minCell = GetCellCoords(aabb.min);
            CellKey maxCell = GetCellCoords(aabb.max);

            for (int x = minCell.first; x <= maxCell.first; ++x) {
                for (int y = minCell.second; y <= maxCell.second; ++y) {
                    grid[{x, y}].push_back(body);
                }
            }
        }

        // To avoid duplicate pairs across multiple cells
        auto pairHash = [](const CollisionPair& p) {
            auto h1 = std::hash<void*>{}(p.first.get());
            auto h2 = std::hash<void*>{}(p.second.get());
            return h1 ^ (h2 << 1);
        };

        auto pairEqual = [](const CollisionPair& p1, const CollisionPair& p2) {
            return (p1.first == p2.first && p1.second == p2.second) ||
                   (p1.first == p2.second && p1.second == p2.first);
        };

        std::unordered_set<CollisionPair, decltype(pairHash), decltype(pairEqual)> uniquePairs(0, pairHash, pairEqual);

        // Check collisions within cells
        for (const auto& [cell, cellBodies] : grid) {
            size_t numBodies = cellBodies.size();
            if (numBodies < 2) continue;

            for (size_t i = 0; i < numBodies; ++i) {
                RigidBodyPtr bodyA = cellBodies[i];
                for (size_t j = i + 1; j < numBodies; ++j) {
                    RigidBodyPtr bodyB = cellBodies[j];

                    if (bodyA->IsStatic() && bodyB->IsStatic()) {
                        continue;
                    }

                    AABB aabbA = bodyA->GetAABB();
                    AABB aabbB = bodyB->GetAABB();

                    if (aabbA.IsOverlapping(aabbB)) {
                        uniquePairs.insert({bodyA, bodyB});
                    }
                }
            }
        }

        potentialCollisions.assign(uniquePairs.begin(), uniquePairs.end());
        return potentialCollisions;
    }
}

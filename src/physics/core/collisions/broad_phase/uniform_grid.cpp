#include "physics/core/collisions/broad_phase/uniform_grid.h"
#include "physics/core/types.h"
#include "../../checked_grid.h"
#include <algorithm>
#include <cmath>
#include <set>
#include <unordered_set>

namespace PhysicsEngine {

    UniformGrid::UniformGrid(float cellSize) : cellSize(cellSize) {
        CheckedGrid::PositiveFinite(cellSize);
    }

    float UniformGrid::getCellSize() const {
        return cellSize;
    }

    void UniformGrid::setCellSize(float size) {
        CheckedGrid::PositiveFinite(size);
        cellSize = size;
    }

    UniformGrid::CellKey UniformGrid::GetCellCoords(const Vector2& pos) const {
        return {
            CheckedGrid::Coordinate(pos.x, cellSize),
            CheckedGrid::Coordinate(pos.y, cellSize)
        };
    }

    std::vector<CollisionPair> UniformGrid::FindPotentialCollisions(const std::vector<RigidBodyPtr>& bodies) {
        std::vector<CollisionPair> potentialCollisions;

        GridMap grid;
        std::uint64_t remainingVisits = CheckedGrid::MaximumGridVisits;

        // Populate grid
        for (const auto& body : bodies) {
            if (!body || !body->shape) {
                throw std::invalid_argument("Uniform grid requires valid rigid bodies.");
            }
            AABB aabb = body->GetAABB();
            
            CellKey minCell = GetCellCoords(aabb.min);
            CellKey maxCell = GetCellCoords(aabb.max);

            CheckedGrid::Charge(CheckedGrid::Window{
                minCell.first, maxCell.first, minCell.second, maxCell.second
            }, remainingVisits);
            for (std::int64_t x = minCell.first; x <= maxCell.first; ++x) {
                for (std::int64_t y = minCell.second; y <= maxCell.second; ++y) {
                    grid[{static_cast<int>(x), static_cast<int>(y)}].push_back(body);
                }
            }
        }

        // To avoid duplicate pairs across multiple cells
        auto pairHash = [](const CollisionPair& p) {
            const std::uint64_t firstId = std::min(p.first->GetId(), p.second->GetId());
            const std::uint64_t secondId = std::max(p.first->GetId(), p.second->GetId());
            auto h1 = std::hash<std::uint64_t>{}(firstId);
            auto h2 = std::hash<std::uint64_t>{}(secondId);
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

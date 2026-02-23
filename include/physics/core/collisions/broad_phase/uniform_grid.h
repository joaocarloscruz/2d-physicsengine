#ifndef UNIFORM_GRID_H
#define UNIFORM_GRID_H

#include "ibroad_phase.h"
#include "../../rigidbody.h"
#include "../../types.h"
#include "../../../math/vector2.h"
#include <vector>
#include <unordered_map>
#include <utility>
#include <memory>

namespace PhysicsEngine {

    // Simple hash structure for integer coordinates
    struct IntPairHash {
        template <class T1, class T2>
        std::size_t operator () (const std::pair<T1,T2> &p) const {
            auto h1 = std::hash<T1>{}(p.first);
            auto h2 = std::hash<T2>{}(p.second);
            // Primarily for reasonable distribution
            return h1 ^ (h2 << 1);
        }
    };

    class UniformGrid : public IBroadPhase {
    public:
        UniformGrid(float cellSize = 100.0f);
        ~UniformGrid() override = default;

        std::vector<CollisionPair> FindPotentialCollisions(const std::vector<RigidBodyPtr>& bodies) override;

        float getCellSize() const;
        void setCellSize(float size);

    private:
        float cellSize;
        
        using CellKey = std::pair<int, int>;
        using GridMap = std::unordered_map<CellKey, std::vector<RigidBodyPtr>, IntPairHash>;

        CellKey GetCellCoords(const Vector2& pos) const;
    };

}

#endif // UNIFORM_GRID_H

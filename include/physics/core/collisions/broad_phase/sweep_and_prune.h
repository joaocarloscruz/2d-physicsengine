#ifndef SWEEP_AND_PRUNE_H
#define SWEEP_AND_PRUNE_H

#include "../../rigidbody.h"
#include "../../types.h"
#include <vector>
#include <utility>

namespace PhysicsEngine {

    class SweepAndPrune {
    public:
        static std::vector<CollisionPair> FindPotentialCollisions(const std::vector<RigidBodyPtr>& bodies);
    };

}

#endif // SWEEP_AND_PRUNE_H

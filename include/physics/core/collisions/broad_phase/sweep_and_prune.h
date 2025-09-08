#ifndef SWEEP_AND_PRUNE_H
#define SWEEP_AND_PRUNE_H

#include "../../rigidbody.h"
#include <vector>
#include <utility>

namespace PhysicsEngine {

    class SweepAndPrune {
    public:
        static std::vector<std::pair<RigidBody*, RigidBody*>> FindPotentialCollisions(const std::vector<RigidBody*>& bodies);
    };

}

#endif // SWEEP_AND_PRUNE_H

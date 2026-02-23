#ifndef SWEEP_AND_PRUNE_H
#define SWEEP_AND_PRUNE_H

#include "ibroad_phase.h"
#include "../../rigidbody.h"
#include "../../types.h"
#include <vector>
#include <utility>

namespace PhysicsEngine {

    class SweepAndPrune : public IBroadPhase {
    public:
        SweepAndPrune() = default;
        ~SweepAndPrune() override = default;

        std::vector<CollisionPair> FindPotentialCollisions(const std::vector<RigidBodyPtr>& bodies) override;
    };

}

#endif // SWEEP_AND_PRUNE_H

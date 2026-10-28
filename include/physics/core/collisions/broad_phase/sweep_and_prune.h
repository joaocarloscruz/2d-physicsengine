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
        std::vector<CollisionPair> FindSpeculativeCollisions(const std::vector<RigidBodyPtr>& bodies,
                                                             const std::vector<float>& margins) override;
        // As above, but reuses bounds the caller already computed: for each i
        // with known[i] set, knownBounds[i] must be bodies[i]->GetAABB().
        std::vector<CollisionPair> FindSpeculativeCollisions(const std::vector<RigidBodyPtr>& bodies,
                                                             const std::vector<float>& margins,
                                                             const std::vector<AABB>& knownBounds,
                                                             const std::vector<char>& known);

    private:
        std::vector<CollisionPair> Find(const std::vector<RigidBodyPtr>& bodies, const std::vector<float>& margins,
                                        const std::vector<AABB>* knownBounds, const std::vector<char>* known);
    };

}

#endif // SWEEP_AND_PRUNE_H

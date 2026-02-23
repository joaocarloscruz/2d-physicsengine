#ifndef IBROAD_PHASE_H
#define IBROAD_PHASE_H

#include "../../rigidbody.h"
#include "../../types.h"
#include <vector>

namespace PhysicsEngine {

class IBroadPhase {
public:
    virtual ~IBroadPhase() = default;

    /**
     * @brief Find potential collisions between a set of bodies.
     * @param bodies The list of bodies in the world.
     * @return A list of pairs of bodies that might be colliding.
     */
    virtual std::vector<CollisionPair> FindPotentialCollisions(const std::vector<RigidBodyPtr>& bodies) = 0;
};

}

#endif // IBROAD_PHASE_H

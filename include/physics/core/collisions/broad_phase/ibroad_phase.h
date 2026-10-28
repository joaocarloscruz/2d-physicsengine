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

    /**
     * @brief Find pairs whose bounds, each grown by its body's margin, overlap.
     * @param margins One non-negative margin per body (same order as bodies).
     *
     * World uses this for speculative contacts. The default ignores the
     * margins, so a custom broad phase still works but only reports
     * overlapping pairs, and its bodies get no early (speculative) contacts.
     */
    virtual std::vector<CollisionPair> FindSpeculativeCollisions(const std::vector<RigidBodyPtr>& bodies,
                                                                 const std::vector<float>& margins) {
        (void)margins;
        return FindPotentialCollisions(bodies);
    }
};

}

#endif // IBROAD_PHASE_H

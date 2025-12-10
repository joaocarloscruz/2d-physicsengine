#include "physics/core/collisions/broad_phase/sweep_and_prune.h"
#include "physics/core/types.h"
#include <algorithm>
#include <list>

namespace PhysicsEngine {

    struct Endpoint {
        RigidBodyPtr body;
        float value;
        bool isStart;

        bool operator<(const Endpoint& other) const {
            return value < other.value;
        }
    };

    std::vector<CollisionPair> SweepAndPrune::FindPotentialCollisions(const std::vector<RigidBodyPtr>& bodies) {
        std::vector<CollisionPair> potentialCollisions;
        if (bodies.size() < 2) {
            return potentialCollisions;
        }

        std::vector<Endpoint> endpoints;
        endpoints.reserve(bodies.size() * 2);
        for (const auto& body : bodies) {
            AABB aabb = body->GetAABB();
            endpoints.push_back({body, aabb.min.x, true});
            endpoints.push_back({body, aabb.max.x, false});
        }

        std::sort(endpoints.begin(), endpoints.end());

        std::list<RigidBodyPtr> activeList;
        for (const auto& endpoint : endpoints) {
            if (endpoint.isStart) {
                for (const auto& activeBody : activeList) {
                    if (endpoint.body->IsStatic() && activeBody->IsStatic()) {
                        continue;
                    }  
                    AABB aabbA = endpoint.body->GetAABB();
                    AABB aabbB = activeBody->GetAABB();
                    if (aabbA.IsOverlapping(aabbB)) {
                        potentialCollisions.push_back({endpoint.body, activeBody});
                    }
                }
                activeList.push_back(endpoint.body);
            } else {
                activeList.remove(endpoint.body);
            }
        }

        return potentialCollisions;
    }
}
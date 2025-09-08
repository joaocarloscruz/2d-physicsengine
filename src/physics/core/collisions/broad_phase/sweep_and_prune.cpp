#include "physics/core/collisions/broad_phase/sweep_and_prune.h"
#include <algorithm>

namespace PhysicsEngine {

    struct Endpoint {
        RigidBody* body;
        float value;
        bool isStart;

        bool operator<(const Endpoint& other) const {
            return value < other.value;
        }
    };

    std::vector<std::pair<RigidBody*, RigidBody*>> SweepAndPrune::FindPotentialCollisions(const std::vector<RigidBody*>& bodies) {
        std::vector<std::pair<RigidBody*, RigidBody*>> potentialCollisions;
        if (bodies.size() < 2) {
            return potentialCollisions;
        }

        std::vector<Endpoint> endpoints;
        endpoints.reserve(bodies.size() * 2);
        for (RigidBody* body : bodies) {
            if (body->IsStatic()) continue;
            AABB aabb = body->GetAABB();
            endpoints.push_back({body, aabb.min.x, true});
            endpoints.push_back({body, aabb.max.x, false});
        }

        std::sort(endpoints.begin(), endpoints.end());

        std::vector<RigidBody*> activeList;
        for (const auto& endpoint : endpoints) {
            if (endpoint.isStart) {
                for (RigidBody* activeBody : activeList) {
                    AABB aabbA = endpoint.body->GetAABB();
                    AABB aabbB = activeBody->GetAABB();
                    if (aabbA.IsOverlapping(aabbB)) {
                        potentialCollisions.push_back({endpoint.body, activeBody});
                    }
                }
                activeList.push_back(endpoint.body);
            } else {
                activeList.erase(std::remove(activeList.begin(), activeList.end(), endpoint.body), activeList.end());
            }
        }

        return potentialCollisions;
    }

}

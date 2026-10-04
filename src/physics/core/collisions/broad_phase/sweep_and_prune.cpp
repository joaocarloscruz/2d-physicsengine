#include "physics/core/collisions/broad_phase/sweep_and_prune.h"
#include "physics/core/types.h"
#include <algorithm>
#include <list>
#include <stdexcept>

namespace PhysicsEngine {

    struct Endpoint {
        std::size_t bodyIndex;
        float value;
        bool isStart;

        bool operator<(const Endpoint& other) const {
            return value < other.value;
        }
    };

    std::vector<CollisionPair> SweepAndPrune::FindPotentialCollisions(const std::vector<RigidBodyPtr>& bodies) {
        std::vector<CollisionPair> potentialCollisions;
        if (bodies.empty()) {
            return potentialCollisions;
        }

        std::vector<Endpoint> endpoints;
        endpoints.reserve(bodies.size() * 2);
        std::vector<AABB> bounds;
        bounds.reserve(bodies.size());
        for (std::size_t i = 0; i < bodies.size(); ++i) {
            if (!bodies[i])
                throw std::invalid_argument("Sweep and prune requires valid rigid bodies.");
            bounds.push_back(bodies[i]->GetAABB());
            endpoints.push_back({i, bounds.back().min.x, true});
            endpoints.push_back({i, bounds.back().max.x, false});
        }

        std::sort(endpoints.begin(), endpoints.end());

        // Geometry is fixed during this call. Reuse its bounds for every pair,
        // but take fresh snapshots next time so moved bodies are never stale.
        std::list<std::size_t> activeList;
        for (const auto& endpoint : endpoints) {
            if (endpoint.isStart) {
                const auto& body = bodies[endpoint.bodyIndex];
                for (const auto activeIndex : activeList) {
                    const auto& activeBody = bodies[activeIndex];
                    if (body->IsStatic() && activeBody->IsStatic()) {
                        continue;
                    }  
                    if (bounds[endpoint.bodyIndex].IsOverlapping(bounds[activeIndex])) {
                        potentialCollisions.push_back({body, activeBody});
                    }
                }
                activeList.push_back(endpoint.bodyIndex);
            } else {
                activeList.remove(endpoint.bodyIndex);
            }
        }

        return potentialCollisions;
    }
}

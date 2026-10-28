#include "physics/core/collisions/broad_phase/sweep_and_prune.h"
#include "physics/core/types.h"
#include <algorithm>
#include <vector>
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
        return FindSpeculativeCollisions(bodies, {});
    }

    std::vector<CollisionPair> SweepAndPrune::FindSpeculativeCollisions(const std::vector<RigidBodyPtr>& bodies,
                                                                        const std::vector<float>& margins) {
        return Find(bodies, margins, nullptr, nullptr);
    }

    std::vector<CollisionPair> SweepAndPrune::FindSpeculativeCollisions(const std::vector<RigidBodyPtr>& bodies,
        const std::vector<float>& margins, const std::vector<AABB>& knownBounds, const std::vector<char>& known) {
        if (knownBounds.size() != bodies.size() || known.size() != bodies.size())
            throw std::invalid_argument("Sweep and prune needs one known bound per body.");
        return Find(bodies, margins, &knownBounds, &known);
    }

    std::vector<CollisionPair> SweepAndPrune::Find(const std::vector<RigidBodyPtr>& bodies, const std::vector<float>& margins,
                                                   const std::vector<AABB>* knownBounds, const std::vector<char>* known) {
        if (!margins.empty() && margins.size() != bodies.size())
            throw std::invalid_argument("Sweep and prune needs one margin per body.");
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
            bounds.push_back(known && (*known)[i] ? (*knownBounds)[i] : bodies[i]->GetAABB());
            if (!margins.empty()) {
                const Vector2 grow{margins[i], margins[i]};
                bounds.back().min = bounds.back().min - grow;
                bounds.back().max = bounds.back().max + grow;
            }
            endpoints.push_back({i, bounds.back().min.x, true});
            endpoints.push_back({i, bounds.back().max.x, false});
        }

        std::sort(endpoints.begin(), endpoints.end());

        // Geometry is fixed during this call. Reuse its bounds for every pair,
        // but take fresh snapshots next time so moved bodies are never stale.
        // Insertion-ordered active set; pairs come out in the same order as
        // they would from a linked list, but scans stay contiguous.
        std::vector<std::size_t> activeList;
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
                const auto found = std::find(activeList.begin(), activeList.end(), endpoint.bodyIndex);
                if (found != activeList.end()) activeList.erase(found);
            }
        }

        return potentialCollisions;
    }
}

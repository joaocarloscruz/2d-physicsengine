#include "physics/physics.h"
#include "physics/core/collisions/broad_phase/sweep_and_prune.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <set>
#include <stdexcept>
#include <utility>
#include <vector>

using namespace PhysicsEngine;

// Fixed work: 500 bodies, 32 vertices, one dense broad-phase call. Timing
// excludes shape setup and candidate verification; no wall-clock pass target.
int main() {
    try {
        constexpr int bodyCount = 500, vertexCount = 32;
        std::vector<Vector2> vertices;
        for (int i = 0; i < vertexCount; ++i) {
            const double angle = 6.283185307179586 * i / vertexCount;
            vertices.push_back({float(4 * std::cos(angle)), float(4 * std::sin(angle))});
        }
        const Polygon shape(vertices);
        std::vector<RigidBodyPtr> bodies;
        for (int i = 0; i < bodyCount; ++i) {
            auto body = std::make_shared<RigidBody>(shape, Material{},
                Vector2{float(i % 25) * .2f, float(i / 25) * .2f});
            body->SetOrientation(float(i % 17) * .03125f);
            bodies.push_back(body);
        }
        SweepAndPrune broad;
        const auto start = std::chrono::steady_clock::now();
        const auto pairs = broad.FindPotentialCollisions(bodies);
        const double elapsed = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - start).count();
        // Every polygon contains a circle of radius 4*cos(pi/32); the entire
        // center footprint is smaller than twice that radius. Every pair overlaps.
        constexpr std::size_t expected = bodyCount * (bodyCount - 1) / 2;
        std::set<std::pair<std::uint64_t, std::uint64_t>> unique;
        for (const auto& pair : pairs) {
            if (!pair.first || !pair.second || pair.first == pair.second)
                throw std::runtime_error("Invalid broad-phase pair.");
            unique.insert(std::minmax(pair.first->GetId(), pair.second->GetId()));
        }
        const bool passed = pairs.size() == expected && unique.size() == expected;
        std::cout << std::setprecision(17) << "{\"bodies\":" << bodyCount
                  << ",\"verticesPerBody\":" << vertexCount
                  << ",\"expectedPairs\":" << expected << ",\"candidatePairs\":" << pairs.size()
                  << ",\"uniquePairs\":" << unique.size() << ",\"elapsedMilliseconds\":" << elapsed
                  << ",\"passed\":" << (passed ? "true" : "false") << "}\n";
        return passed ? 0 : 1;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}

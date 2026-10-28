// Visible collision quality at the default 60 Hz World configuration: resting
// overlap, stack and pyramid drift, and impact penetration of falling boxes.
// Each scene has a pass threshold; the exit code is nonzero if any fails, so
// CTest guards against regressions such as collapsing stacks or deep impacts.
#include "physics/physics.h"
#include "physics/core/collisions/collision_dispatcher.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>
#include <vector>

using namespace PhysicsEngine;

namespace {
struct Measure {
    double drift = 0, tilt = 0, speed = 0, peakPenetration = 0;
};

double Penetration(World& world) {
    double worst = 0;
    const auto& bodies = world.getBodies();
    for (std::size_t i = 0; i < bodies.size(); ++i)
        for (std::size_t j = i + 1; j < bodies.size(); ++j)
            if (!(bodies[i]->IsStatic() && bodies[j]->IsStatic()))
                worst = std::max(worst, double(CheckCollision(bodies[i].get(), bodies[j].get()).penetration));
    return worst;
}

RigidBodyPtr Add(World& world, const Shape& shape, Vector2 position, bool fixed = false, float orientation = 0) {
    auto body = std::make_shared<RigidBody>(shape, Material{1.0f, 0.2f}, position, fixed);
    body->SetOrientation(orientation);
    world.addBody(body);
    return body;
}

void Ground(World& world) {
    Add(world, Polygon::MakeBox(40, 2), {0, -1}, true);
    world.addUniversalForce(std::make_unique<Gravity>(Vector2{0, -9.81f}));
}

// Drift, tilt and speed are measured after settleSeconds against the settled pose.
Measure Run(World& world, double seconds, double settleSeconds) {
    Measure m;
    std::vector<Vector2> positions;
    std::vector<float> angles;
    const int steps = int(seconds * 60), settle = int(settleSeconds * 60);
    for (int step = 0; step < steps; ++step) {
        world.step(1.0f / 60);
        m.peakPenetration = std::max(m.peakPenetration, Penetration(world));
        if (step == settle)
            for (const auto& body : world.getBodies()) {
                positions.push_back(body->position);
                angles.push_back(body->orientation);
            }
        if (step <= settle) continue;
        std::size_t index = 0;
        for (const auto& body : world.getBodies()) {
            if (!body->IsStatic()) {
                m.drift = std::max(m.drift, double((body->position - positions[index]).magnitude()));
                m.tilt = std::max(m.tilt, std::abs(std::remainder(double(body->orientation - angles[index]), 6.283185307179586)));
                m.speed = std::max(m.speed, double(body->velocity.magnitude()));
            }
            ++index;
        }
    }
    return m;
}

int failures = 0;
void Check(const char* scene, const char* metric, double value, double limit) {
    const bool pass = value <= limit;
    failures += !pass;
    std::printf("%-22s %-18s %10.6f <= %-9g %s\n", scene, metric, value, limit, pass ? "ok" : "FAIL");
}
}

int main() {
    {
        World world; Ground(world); Add(world, Polygon::MakeBox(1, 1), {0, 0.5f});
        const auto m = Run(world, 4, 1);
        Check("resting box", "peak penetration", m.peakPenetration, 0.006);
        Check("resting box", "speed", m.speed, 1e-3);
    }
    {
        World world; Ground(world);
        for (int i = 0; i < 10; ++i) Add(world, Polygon::MakeBox(1, 1), {0, 0.5f + i});
        const auto m = Run(world, 10, 1);
        Check("stack of 10", "drift (m)", m.drift, 0.01);
        Check("stack of 10", "tilt (rad)", m.tilt, 0.01);
        Check("stack of 10", "peak penetration", m.peakPenetration, 0.01);
    }
    {
        World world; Ground(world);
        for (int row = 0; row < 5; ++row)
            for (int i = 0; i < 5 - row; ++i)
                Add(world, Polygon::MakeBox(1, 1), {-2.0f + row * 0.5f + i * 1.05f, 0.5f + row});
        const auto m = Run(world, 10, 2);
        Check("pyramid of 15", "drift (m)", m.drift, 0.01);
        Check("pyramid of 15", "peak penetration", m.peakPenetration, 0.01);
    }
    {
        World world; Ground(world);
        Add(world, Polygon::MakeBox(1, 1), {0, 0.5f});
        Add(world, Polygon::MakeBox(1, 1), {0.3f, 4}, false, 0.4f);
        const auto m = Run(world, 6, 3);
        Check("box dropped on box", "peak penetration", m.peakPenetration, 0.012);
        Check("box dropped on box", "settled speed", m.speed, 1e-3);
    }
    {
        // 24 boxes dropped one at a time from 9 m (about 13 m/s at impact).
        World world; Ground(world);
        Add(world, Polygon::MakeBox(2, 14), {-5, 6}, true);
        Add(world, Polygon::MakeBox(2, 14), {5, 6}, true);
        double peak = 0, speed = 0;
        for (int step = 0; step < 60 * 14; ++step) {
            if (step % 20 == 0 && step < 60 * 8)
                Add(world, Polygon::MakeBox(1, 1), {-2.5f + float((step / 20) % 6), 9.0f}, false, 0.3f * float(step / 20));
            world.step(1.0f / 60);
            peak = std::max(peak, Penetration(world));
        }
        for (const auto& body : world.getBodies()) speed = std::max(speed, double(body->velocity.magnitude()));
        Check("box rain", "peak penetration", peak, 0.03);
        Check("box rain", "final max speed", speed, 0.01);
    }
    std::printf("%s\n", failures ? "collision quality: FAIL" : "collision quality: ok");
    return failures ? 1 : 0;
}

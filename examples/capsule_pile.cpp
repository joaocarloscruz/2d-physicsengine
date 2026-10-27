#include "physics/physics.h"
#include "physics/core/collisions/collision_dispatcher.h"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <vector>

// Bounded capsule scene: six capsules drop onto a ground box and a static
// capsule ramp while a CCD circle is fired at the ramp. Capsule CCD supports
// only circles against static capsules, so the bullet's category is masked out
// of the dynamic capsules. After four seconds the example reports the deepest
// remaining contact and the total kinetic energy.
int main() {
    using namespace PhysicsEngine;
    World world;
    world.addUniversalForce(std::make_unique<Gravity>(Vector2{0, -9.81f}));
    auto ground = std::make_shared<RigidBody>(Polygon::MakeBox(30, 1), Material{}, Vector2{0, -0.5f}, true);
    auto ramp = std::make_shared<RigidBody>(Capsule(3, 0.2f), Material{}, Vector2{-6, 1.5f}, true);
    ramp->SetOrientation(-0.4f);
    world.addBody(ground);
    world.addBody(ramp);
    std::vector<RigidBodyPtr> capsules;
    for (int i = 0; i < 6; ++i) {
        auto body = std::make_shared<RigidBody>(Capsule(0.6f, 0.25f), Material{1, 0.2f},
            Vector2{-7.0f + 1.6f * i, 4.0f + 0.7f * i});
        body->SetOrientation(0.5f * i);
        body->SetCollisionMaskBits(~0x2u);
        capsules.push_back(body);
        world.addBody(body);
    }
    auto bullet = std::make_shared<RigidBody>(Circle(0.1f), Material{}, Vector2{-12, 1.0f});
    bullet->SetVelocity({150, 0});
    bullet->SetCcdEnabled(true);
    bullet->SetCollisionCategoryBits(0x2u);
    world.addBody(bullet);

    std::uint32_t ccdImpacts = 0;
    for (int frame = 0; frame < 240; ++frame) {
        world.step(1.0f / 60);
        ccdImpacts += world.getLastStepStatistics().ccdImpactCount;
    }

    double kinetic = 0, deepest = 0;
    bool finite = true;
    const auto& bodies = world.getBodies();
    for (const auto& body : bodies) {
        finite = finite && std::isfinite(body->position.x) && std::isfinite(body->position.y);
        if (!body->IsStatic())
            kinetic += 0.5 * body->GetMass() * body->velocity.magnitudeSquared()
                + 0.5 * body->GetInertia() * body->angularVelocity * body->angularVelocity;
    }
    for (std::size_t i = 0; i < bodies.size(); ++i)
        for (std::size_t j = i + 1; j < bodies.size(); ++j)
            deepest = std::max(deepest, double(CheckCollision(bodies[i].get(), bodies[j].get()).penetration));
    double lowest = 1e30;
    for (const auto& body : capsules) lowest = std::min(lowest, double(body->position.y));
    std::cout << std::setprecision(9) << "capsules=" << capsules.size()
              << " lowest_center=" << lowest << " deepest_contact=" << deepest
              << " kinetic=" << kinetic << " ccd_impacts=" << ccdImpacts << '\n';
    // Centers stay above the ground by about one radius and contacts stay shallow.
    return finite && lowest > 0.2 && deepest < 0.05 && ccdImpacts >= 1 ? 0 : 1;
}

#include "physics/physics.h"
#include <cmath>
#include <cstdio>
#include <memory>

// The getting-started tour (docs/getting-started.md) as one runnable program:
// a static ground, a stack of boxes, a ball that reports when it lands, a
// pendulum on a revolute joint, sleeping, and a ray cast. It exits nonzero if
// the scene does not behave as the guide describes.
using namespace PhysicsEngine;

namespace {
// Collision events arrive once per step, after the solver ran.
class LandingCounter : public ICollisionListener {
public:
    explicit LandingCounter(std::uint64_t ballId) : ballId(ballId) {}
    void onCollisionBegin(const CollisionEvent& event) override {
        if (event.bodyAId == ballId || event.bodyBId == ballId) ++landings;
    }
    int landings = 0;
private:
    std::uint64_t ballId;
};
}

int main() {
    World world;
    SimulationConfig config = world.getSimulationConfig();
    config.enableSleeping = true;
    world.setSimulationConfig(config);
    world.addUniversalForce(std::make_unique<Gravity>(Vector2{0, -9.81f}));

    // Static bodies never move. Shapes are copied, so temporaries are fine.
    auto ground = std::make_shared<RigidBody>(Polygon::MakeBox(20, 1), Material{}, Vector2{0, -0.5f}, true);
    world.addBody(ground);

    // Material is {density kg/m^2, restitution, static friction, dynamic friction}.
    const Material wood{1.0f, 0.1f, 0.6f, 0.4f};
    for (int i = 0; i < 5; ++i)
        world.addBody(std::make_shared<RigidBody>(Polygon::MakeBox(1, 1), wood, Vector2{-3, 0.5f + i}));

    auto ball = std::make_shared<RigidBody>(Circle(0.25f), Material{1.0f, 0.6f, 0.4f, 0.3f}, Vector2{2, 4});
    world.addBody(ball);
    LandingCounter counter(ball->GetId());
    world.addCollisionListener(&counter);

    // A pendulum: a capsule hung from a static anchor by a revolute joint.
    auto pivot = std::make_shared<RigidBody>(Circle(0.1f), Material{}, Vector2{6, 5}, true);
    auto bob = std::make_shared<RigidBody>(Capsule(0.5f, 0.15f), wood, Vector2{7, 5});
    world.addBody(pivot);
    world.addBody(bob);
    world.addJoint(std::make_shared<RevoluteJoint>(pivot, bob, Vector2{0, 0}, Vector2{-1, 0}));

    double lowestBob = bob->position.y;
    for (int frame = 0; frame < 600; ++frame) { // 10 seconds at the default 60 Hz
        world.step();
        lowestBob = std::fmin(lowestBob, bob->position.y);
    }
    world.removeCollisionListener(&counter);

    const auto ray = RayCastNearest(world, Vector2{-3, 10}, Vector2{-3, -10});
    const auto& top = world.getBodies()[5];
    std::printf("stack top y=%.4f (expected 4.5), awake=%d\n", top->position.y, int(top->IsAwake()));
    std::printf("ball y=%.4f, landings=%d\n", ball->position.y, counter.landings);
    std::printf("pendulum lowest y=%.4f (pivot at 5, arm 1)\n", lowestBob);
    if (ray) std::printf("ray from above hits body %llu at y=%.4f\n",
        static_cast<unsigned long long>(ray->body->GetId()), ray->hit.point.y);

    const bool ok = std::fabs(top->position.y - 4.5f) < 0.02f && !top->IsAwake()
        && std::fabs(ball->position.y - 0.25f) < 0.02f && counter.landings >= 1
        && std::fabs(lowestBob - 4.0f) < 0.05f
        && ray && ray->body == top && std::fabs(ray->hit.point.y - 5.0f) < 0.02f;
    std::printf("%s\n", ok ? "getting started: ok" : "getting started: FAIL");
    return ok ? 0 : 1;
}

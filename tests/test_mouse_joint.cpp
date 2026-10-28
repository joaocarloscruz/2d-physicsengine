#include "catch_amalgamated.hpp"
#include "joint_oscillation.h"
#include "physics/physics.h"
#include <limits>

using namespace PhysicsEngine;
namespace {
SimulationConfig FreeConfig() {
    SimulationConfig c;
    c.enableLinearVelocityLimit = false;
    c.enableAngularVelocityLimit = false;
    return c;
}
RigidBodyPtr Anchor() {
    auto ground = std::make_shared<RigidBody>(Circle(0.1f), Material{}, Vector2{0, -50}, true);
    ground->SetCollisionMaskBits(0);
    return ground;
}
RigidBodyPtr Box(Vector2 p, float size, float mass) {
    auto body = std::make_shared<RigidBody>(Polygon::MakeBox(size, size), Material{}, p);
    body->SetMass(mass);
    return body;
}
std::vector<double> Oscillate(float frequency, float damping, float h, int steps) {
    World world(FreeConfig());
    auto ground = Anchor(), body = Box({0.25f, 0}, 1, 2);
    body->SetCollisionMaskBits(0);
    world.addBody(ground);
    world.addBody(body);
    world.addJoint(std::make_shared<MouseJoint>(ground, body, Vector2{}, Vector2{0, 0}, frequency,
                                                damping, 1e6f));
    std::vector<double> x{body->position.x};
    for (int i = 0; i < steps; ++i) {
        world.step(h);
        x.push_back(body->position.x);
        REQUIRE(body->position.y == 0);
    }
    return x;
}
}

TEST_CASE("Mouse joint oscillates at the backward-Euler image of its spring", "[mouse][oracle]") {
    // f = 2 Hz, zeta = 0.1, h = 1/60: backward Euler gives 1.9617 Hz and
    // damping 0.2251; the run must match that oracle, not just the target.
    const double h = 1.0 / 60;
    const auto measured = JointOscillation::Measure(Oscillate(2, 0.1f, float(h), 240), h);
    const auto oracle = JointOscillation::BackwardEuler(2, 0.1, h);
    REQUIRE(measured.peaks >= 4);
    CHECK(measured.frequency == Catch::Approx(oracle.frequency).epsilon(0.005));
    CHECK(measured.dampingRatio == Catch::Approx(oracle.dampingRatio).epsilon(0.01));
    // The requested frequency survives the large step within 2%.
    CHECK(measured.frequency == Catch::Approx(2).epsilon(0.02));
}

TEST_CASE("Mouse joint frequency and damping converge to the requested values", "[mouse][oracle]") {
    // h = 1 ms: numerical damping is about h*omega/2 = 0.006.
    const double h = 1.0 / 1000;
    const auto measured = JointOscillation::Measure(Oscillate(2, 0.1f, float(h), 3000), h);
    REQUIRE(measured.peaks >= 5);
    CHECK(measured.frequency == Catch::Approx(2).epsilon(0.002));
    CHECK(measured.dampingRatio == Catch::Approx(0.1).margin(0.01));
    const double h2 = 1.0 / 1000;
    const auto undamped = JointOscillation::Measure(Oscillate(1, 0, float(h2), 4500), h2);
    CHECK(undamped.frequency == Catch::Approx(1).epsilon(0.002));
    CHECK(undamped.dampingRatio == Catch::Approx(0).margin(0.005));
}

TEST_CASE("Mouse joint force never exceeds its cap", "[mouse]") {
    World world(FreeConfig());
    auto ground = Anchor(), body = Box({0, 0}, 1, 1);
    world.addBody(ground);
    world.addBody(body);
    auto joint = std::make_shared<MouseJoint>(ground, body, Vector2{}, Vector2{10, 0}, 5, 0.7f, 3);
    world.addJoint(joint);
    const float h = 1.f / 60;
    for (int i = 1; i <= 60; ++i) {
        world.step(h);
        const Vector2 force = joint->getReactionForce();
        REQUIRE(std::hypot(force.x, force.y) <= 3 * (1 + 1e-6));
        // Saturated from the first step: constant 3 N on 1 kg.
        REQUIRE(body->velocity.x == Catch::Approx(3.0 * h * i).epsilon(1e-5));
        REQUIRE(body->velocity.y == 0);
    }
    REQUIRE(body->position.x == Catch::Approx(0.5 * 3 * 1.0 * (1 + 1.0 / 60)).epsilon(1e-4));
}

TEST_CASE("Mouse joint holds a hanging body at the static spring sag", "[mouse][oracle]") {
    // Steady state of the soft step: k*x = m*g, so x = g/omega^2 below the
    // target, independent of warm starting.
    for (float factor : {0.0f, 1.0f}) {
        SimulationConfig config = FreeConfig();
        config.warmStartFactor = factor;
        config.solverIterations = 1;
        World world(config);
        auto ground = Anchor(), body = Box({0, 0}, 0.5f, 3);
        body->SetCollisionMaskBits(0);
        world.addBody(ground);
        world.addBody(body);
        world.addUniversalForce(std::make_unique<Gravity>(Vector2{0, -9.81f}));
        auto joint = std::make_shared<MouseJoint>(ground, body, Vector2{}, Vector2{0, 0}, 3, 1, 1000);
        world.addJoint(joint);
        for (int i = 0; i < 600; ++i) world.step(1.f / 60);
        const double omega = 2 * JointOscillation::Pi * 3;
        CHECK(body->position.y == Catch::Approx(-9.81 / (omega * omega)).epsilon(1e-4));
        CHECK(joint->getReactionForce().y == Catch::Approx(3 * 9.81).epsilon(1e-4));
    }
}
TEST_CASE("Mouse joint dragging into a wall neither tunnels nor explodes", "[mouse][contact]") {
    // A 0.5 m, 0.25 kg box dragged at a 0.5 m thick static wall with Box2D's
    // suggested force scale, 1000 * m * g (163 m/s of speed change per 60 Hz
    // step at most), default config: speculative contacts, velocity limits.
    // 1. A soft drag to a target 7 m past the wall.
    // 2. A stiff flick: 30 Hz toward a target 97 m past the wall. The box
    //    reaches the force cap's speed in one step, so it starts inside the
    //    speculative margin late; the contact still stops it in the wall's
    //    outer half and pushes it back out.
    struct Case { float frequency, target; double overlapBound; };
    const float maxForce = 1000 * 0.25f * 9.81f;
    for (const Case c : {Case{5, 10, 0.02}, Case{30, 100, 0.3}}) {
        World world;
        auto ground = Anchor();
        auto wall = std::make_shared<RigidBody>(Polygon::MakeBox(0.5f, 6), Material{}, Vector2{3, 0}, true);
        auto body = Box({0, 0}, 0.5f, 0.25f);
        world.addBody(ground);
        world.addBody(wall);
        world.addBody(body);
        auto joint = std::make_shared<MouseJoint>(ground, body, Vector2{0.1f, 0.1f}, Vector2{c.target, 0},
                                                  c.frequency, 0.7f, maxForce);
        world.addJoint(joint);
        double deepest = 0, fastest = 0;
        for (int i = 0; i < 600; ++i) {
            world.step(1.f / 60);
            REQUIRE(std::isfinite(body->position.x));
            // Right edge of the body (any rotation) against the wall's face at 2.75.
            deepest = std::max(deepest, double(body->GetAABB().max.x) - 2.75);
            fastest = std::max(fastest, double(body->velocity.magnitude()));
            REQUIRE(body->position.x < 2.75); // the center never enters the wall
        }
        INFO("frequency " << c.frequency << " Hz, target " << c.target << ": deepest AABB overlap "
                          << deepest << " m, peak speed " << fastest << " m/s, final speed "
                          << body->velocity.magnitude());
        // AABB overlap bounds penetration from above (rotation inflates it).
        CHECK(deepest < c.overlapBound);
        CHECK(fastest <= maxForce / 0.25 / 60 * 1.001);
        // Pinned against the wall by the capped force, it comes to rest.
        CHECK(body->velocity.magnitude() < 0.05f);
        CHECK(body->GetAABB().max.x - 2.75f < 0.02f);
        joint->setTarget({-2, 0});
        for (int i = 0; i < 300; ++i) world.step(1.f / 60);
        CHECK(body->position.x == Catch::Approx(-2).margin(0.2));
    }
}


TEST_CASE("Mouse joint target changes wake a sleeping body", "[mouse][sleeping]") {
    SimulationConfig config = FreeConfig();
    config.enableSleeping = true;
    World world(config);
    auto ground = Anchor(), body = Box({0, 0}, 1, 1);
    world.addBody(ground);
    world.addBody(body);
    auto joint = std::make_shared<MouseJoint>(ground, body, Vector2{}, Vector2{0, 0}, 4, 1, 100);
    world.addJoint(joint);
    for (int i = 0; i < 120 && body->IsAwake(); ++i) world.step(1.f / 60);
    REQUIRE_FALSE(body->IsAwake());
    joint->setTarget({0, 0}); // unchanged: stays asleep
    REQUIRE_FALSE(body->IsAwake());
    joint->setTarget({1, 0});
    REQUIRE(body->IsAwake());
    for (int i = 0; i < 180; ++i) world.step(1.f / 60);
    CHECK(body->position.x == Catch::Approx(1).margin(0.01));
}

TEST_CASE("Mouse joint validates inputs before changing state", "[mouse][validation]") {
    const float inf = std::numeric_limits<float>::infinity(), nan = std::numeric_limits<float>::quiet_NaN();
    auto ground = Anchor(), body = Box({0, 0}, 1, 1), other = Box({2, 0}, 1, 1);
    REQUIRE_THROWS_AS(MouseJoint(other, body, {}, {}, 5, 0.7f, 10), std::invalid_argument);
    REQUIRE_THROWS_AS(MouseJoint(ground, body, {}, {nan, 0}, 5, 0.7f, 10), std::invalid_argument);
    REQUIRE_THROWS_AS(MouseJoint(ground, body, {inf, 0}, {}, 5, 0.7f, 10), std::invalid_argument);
    REQUIRE_THROWS_AS(MouseJoint(ground, body, {}, {}, 0, 0.7f, 10), std::invalid_argument);
    REQUIRE_THROWS_AS(MouseJoint(ground, body, {}, {}, -1, 0.7f, 10), std::invalid_argument);
    REQUIRE_THROWS_AS(MouseJoint(ground, body, {}, {}, inf, 0.7f, 10), std::invalid_argument);
    REQUIRE_THROWS_AS(MouseJoint(ground, body, {}, {}, 5, -0.1f, 10), std::invalid_argument);
    REQUIRE_THROWS_AS(MouseJoint(ground, body, {}, {}, 5, nan, 10), std::invalid_argument);
    REQUIRE_THROWS_AS(MouseJoint(ground, body, {}, {}, 5, 0.7f, -1), std::invalid_argument);
    REQUIRE_THROWS_AS(MouseJoint(ground, body, {}, {}, 5, 0.7f, inf), std::invalid_argument);
    MouseJoint joint(ground, body, {0.1f, 0}, {1, 2}, 5, 0.7f, 10);
    REQUIRE(joint.getCollideConnected());
    REQUIRE(joint.getBodyA() == ground);
    REQUIRE(joint.getBodyB() == body);
    REQUIRE_THROWS_AS(joint.setTarget({inf, 0}), std::invalid_argument);
    REQUIRE_THROWS_AS(joint.setSpring(0, 1), std::invalid_argument);
    REQUIRE_THROWS_AS(joint.setSpring(2, -1), std::invalid_argument);
    REQUIRE_THROWS_AS(joint.setMaxForce(nan), std::invalid_argument);
    REQUIRE(joint.getTarget() == Vector2{1, 2});
    REQUIRE(joint.getFrequency() == 5);
    REQUIRE(joint.getDampingRatio() == 0.7f);
    REQUIRE(joint.getMaxForce() == 10);
}

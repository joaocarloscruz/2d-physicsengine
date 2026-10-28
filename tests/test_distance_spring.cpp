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
RigidBodyPtr Ball(Vector2 p, bool fixed = false) {
    auto body = std::make_shared<RigidBody>(Circle(0.1f), Material{1, 0}, p, fixed);
    body->SetCollisionMaskBits(0);
    if (!fixed) body->SetMass(1);
    return body;
}
}

TEST_CASE("Distance spring oscillates at the requested frequency", "[distance-spring][oracle]") {
    for (double h : {1.0 / 60, 1.0 / 1000}) {
        World world(FreeConfig());
        auto a = Ball({0, 0}, true), b = Ball({1.1f, 0});
        world.addBody(a);
        world.addBody(b);
        auto joint = std::make_shared<DistanceJoint>(a, b, 1);
        joint->setSpring(2, 0.1f);
        world.addJoint(joint);
        std::vector<double> samples;
        for (int i = 0; i < int(3 / h); ++i) {
            samples.push_back(joint->getCurrentLength() - 1);
            world.step(float(h));
        }
        const auto measured = JointOscillation::Measure(samples, h);
        const auto oracle = JointOscillation::BackwardEuler(2, 0.1, h);
        INFO("h " << h << ": measured " << measured.frequency << " Hz zeta " << measured.dampingRatio
                  << ", oracle " << oracle.frequency << " Hz zeta " << oracle.dampingRatio);
        REQUIRE(measured.peaks >= 3);
        CHECK(measured.frequency == Catch::Approx(oracle.frequency).epsilon(0.005));
        CHECK(measured.dampingRatio == Catch::Approx(oracle.dampingRatio).epsilon(0.02));
        CHECK(measured.frequency == Catch::Approx(2).epsilon(h > 0.01 ? 0.02 : 0.002));
        if (h < 0.01) CHECK(measured.dampingRatio == Catch::Approx(0.1).margin(0.01));
    }
}

TEST_CASE("Distance length range acts as a rope", "[distance-spring]") {
    World world(FreeConfig());
    world.addUniversalForce(std::make_unique<Gravity>(Vector2{0, -9.81f}));
    World reference(FreeConfig());
    reference.addUniversalForce(std::make_unique<Gravity>(Vector2{0, -9.81f}));
    auto a = Ball({0, 0}, true), b = Ball({0, -1}), free = Ball({0, -1});
    world.addBody(a);
    world.addBody(b);
    reference.addBody(free);
    auto joint = std::make_shared<DistanceJoint>(a, b, 1);
    joint->setLengthRange(0, 2);
    world.addJoint(joint);
    // Slack: the bob falls exactly like an unjointed body until the rope is taut.
    int step = 0;
    // Stop before the step whose motion would reach the stop (speculative row).
    for (; free->position.y + (free->velocity.y - 9.81f / 60) / 60 > -1.99f; ++step) {
        world.step(1.f / 60);
        reference.step(1.f / 60);
        REQUIRE(b->position.y == free->position.y);
        REQUIRE(b->velocity.y == free->velocity.y);
    }
    for (int i = 0; i < 240; ++i) {
        world.step(1.f / 60);
        REQUIRE(joint->getCurrentLength() < 2 + 0.006);
    }
    INFO("free fall for " << step << " steps, then length " << joint->getCurrentLength());
    CHECK(joint->getCurrentLength() == Catch::Approx(2).margin(0.006));
    CHECK(std::abs(b->velocity.y) < 1e-3);
}

TEST_CASE("Distance spring with a range hangs on its stop", "[distance-spring]") {
    // A 1 Hz spring would sag g/w^2 = 0.248 m below the rest length; the
    // maximum length 1.1 stops it at 0.1.
    World world(FreeConfig());
    world.addUniversalForce(std::make_unique<Gravity>(Vector2{0, -9.81f}));
    auto a = Ball({0, 0}, true), b = Ball({0, -1});
    world.addBody(a);
    world.addBody(b);
    auto joint = std::make_shared<DistanceJoint>(a, b, 1);
    joint->setSpring(1, 1);
    world.addJoint(joint);
    for (int i = 0; i < 600; ++i) world.step(1.f / 60);
    const double omega = 2 * JointOscillation::Pi;
    CHECK(joint->getCurrentLength() == Catch::Approx(1 + 9.81 / (omega * omega)).epsilon(1e-4));
    joint->setLengthRange(0.5f, 1.1f);
    for (int i = 0; i < 600; ++i) world.step(1.f / 60);
    CHECK(joint->getCurrentLength() == Catch::Approx(1.1).margin(0.006));
    CHECK(std::abs(b->velocity.y) < 1e-3);
}

TEST_CASE("Distance joint defaults stay rigid and bit-identical", "[distance-spring]") {
    // A pendulum with the options set back to their defaults must match an
    // untouched joint exactly.
    World plain(FreeConfig()), reset(FreeConfig());
    std::vector<RigidBodyPtr> bobs;
    for (World* w : {&plain, &reset}) {
        w->addUniversalForce(std::make_unique<Gravity>(Vector2{0, -9.81f}));
        auto a = Ball({0, 0}, true), b = Ball({1, 0});
        w->addBody(a);
        w->addBody(b);
        auto joint = std::make_shared<DistanceJoint>(a, b, 1);
        if (w == &reset) {
            joint->setSpring(3, 0.5f);
            joint->setLengthRange(0.2f, 3);
            w->addJoint(joint);
            for (int i = 0; i < 10; ++i) w->step(0); // soft mode on zero steps leaves no impulse
            joint->setSpring(0, 0);
            joint->setLengthRange(1, 1);
        } else {
            w->addJoint(joint);
        }
        bobs.push_back(b);
    }
    for (int i = 0; i < 600; ++i) {
        plain.step(1.f / 60);
        reset.step(1.f / 60);
        REQUIRE(bobs[0]->position == bobs[1]->position);
        REQUIRE(bobs[0]->velocity == bobs[1]->velocity);
    }
}

TEST_CASE("Distance spring options validate before changing state", "[distance-spring][validation]") {
    const float inf = std::numeric_limits<float>::infinity(), nan = std::numeric_limits<float>::quiet_NaN();
    auto a = Ball({0, 0}), b = Ball({1, 0});
    DistanceJoint joint(a, b, 1);
    REQUIRE(joint.getMinLength() == 1);
    REQUIRE(joint.getMaxLength() == 1);
    REQUIRE(joint.getSpringFrequency() == 0);
    REQUIRE_THROWS_AS(joint.setSpring(-1, 0), std::invalid_argument);
    REQUIRE_THROWS_AS(joint.setSpring(inf, 0), std::invalid_argument);
    REQUIRE_THROWS_AS(joint.setSpring(1, nan), std::invalid_argument);
    REQUIRE_THROWS_AS(joint.setSpring(1, -1), std::invalid_argument);
    REQUIRE_THROWS_AS(joint.setLengthRange(-0.1f, 2), std::invalid_argument);
    REQUIRE_THROWS_AS(joint.setLengthRange(1.5f, 2), std::invalid_argument); // length below range
    REQUIRE_THROWS_AS(joint.setLengthRange(0, 0.5f), std::invalid_argument); // length above range
    REQUIRE_THROWS_AS(joint.setLengthRange(0, inf), std::invalid_argument);
    REQUIRE_THROWS_AS(joint.setLengthRange(nan, 2), std::invalid_argument);
    REQUIRE(joint.getMinLength() == 1);
    REQUIRE(joint.getMaxLength() == 1);
    REQUIRE(joint.getSpringFrequency() == 0);
    joint.setLengthRange(0, 2);
    joint.setSpring(2, 0.3f);
    REQUIRE(joint.getMinLength() == 0);
    REQUIRE(joint.getMaxLength() == 2);
    REQUIRE(joint.getSpringFrequency() == 2);
    REQUIRE(joint.getSpringDampingRatio() == 0.3f);
}

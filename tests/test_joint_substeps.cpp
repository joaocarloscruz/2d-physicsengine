#include "catch_amalgamated.hpp"
#include "physics/core/forces/gravity.h"
#include "physics/core/prismatic_joint.h"
#include "physics/core/world.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

using namespace PhysicsEngine;
namespace {
constexpr float Dt = 1.0f / 60.0f;
constexpr float Pi = 3.14159265f;

SimulationConfig Config(bool jointSubsteps) {
    SimulationConfig config;
    config.enableJointSubsteps = jointSubsteps;
    return config;
}

RigidBodyPtr Add(World& world, const Shape& shape, Vector2 position, bool fixed = false) {
    auto body = std::make_shared<RigidBody>(shape, Material{1, 0.1f}, position, fixed);
    world.addBody(body);
    return body;
}

struct Chain {
    std::vector<RigidBodyPtr> links;
    RigidBodyPtr end;
};

// Ten capsule links hanging from a static anchor at (0, 5), with an optional
// end box `endRatio` times as heavy as one link.
Chain HangingChain(World& world, float endRatio) {
    world.addUniversalForce(std::make_unique<Gravity>(Vector2{0, -9.81f}));
    const float h = 0.15f;
    auto anchor = Add(world, Circle(0.05f), {0, 5}, true);
    Chain chain;
    for (int i = 0; i < 10; ++i) {
        auto link = Add(world, Capsule(h, 0.05f), {0, 5 - (2 * i + 1) * h});
        link->SetOrientation(-Pi / 2);
        chain.links.push_back(link);
    }
    world.addJoint(std::make_shared<RevoluteJoint>(anchor, chain.links[0], Vector2{}, Vector2{-h, 0}));
    for (int i = 0; i + 1 < 10; ++i)
        world.addJoint(std::make_shared<RevoluteJoint>(chain.links[i], chain.links[i + 1], Vector2{h, 0},
                                                       Vector2{-h, 0}));
    if (endRatio > 0) {
        chain.end = Add(world, Polygon::MakeBox(0.4f, 0.4f), {0, 5 - 20 * h - 0.2f});
        chain.end->SetMass(endRatio * chain.links[0]->GetMass());
        world.addJoint(std::make_shared<RevoluteJoint>(chain.links.back(), chain.end, Vector2{h, 0},
                                                       Vector2{0, 0.2f}));
    }
    return chain;
}

double MaxSeparation(const World& world) {
    double worst = 0;
    for (const auto& joint : world.getJoints())
        worst = std::max(worst, double((joint->getAnchorA() - joint->getAnchorB()).magnitude()));
    return worst;
}

double MaxSpeed(const World& world) {
    double fastest = 0;
    for (const auto& body : world.getBodies())
        if (!body->IsStatic()) fastest = std::max(fastest, double(body->velocity.magnitude()));
    return fastest;
}

struct ChainRun {
    double stretch = 0, separation = 0, lateSpeed = 0;
};
ChainRun RunHeavyChain(bool jointSubsteps) {
    World world(Config(jointSubsteps));
    const auto chain = HangingChain(world, 100);
    ChainRun run;
    for (int step = 0; step < 6 * 60; ++step) {
        world.step(Dt);
        run.separation = std::max(run.separation, MaxSeparation(world));
        if (step >= 4 * 60) run.lateSpeed = std::max(run.lateSpeed, MaxSpeed(world));
    }
    // The tip anchor hangs 3 m below the static anchor at rest length.
    run.stretch = (world.getJoints().back()->getAnchorB() - Vector2{0, 5}).magnitude() - 3.0;
    return run;
}
} // namespace

TEST_CASE("Joint substep settings default off and are validated", "[simulation_config][joint-substeps]") {
    SimulationConfig config;
    REQUIRE_FALSE(config.enableJointSubsteps);
    REQUIRE(config.jointHertz == Catch::Approx(60.0f));
    REQUIRE(config.jointDampingRatio == Catch::Approx(2.0f));
    REQUIRE_NOTHROW(config.Validate());
    for (float hertz : {0.0f, -1.0f, std::numeric_limits<float>::infinity(), std::numeric_limits<float>::quiet_NaN()}) {
        SimulationConfig invalid;
        invalid.jointHertz = hertz;
        REQUIRE_THROWS_AS(invalid.Validate(), std::invalid_argument);
    }
    for (float damping : {-0.5f, std::numeric_limits<float>::infinity(), std::numeric_limits<float>::quiet_NaN()}) {
        SimulationConfig invalid;
        invalid.jointDampingRatio = damping;
        REQUIRE_THROWS_AS(invalid.Validate(), std::invalid_argument);
    }
    config.jointDampingRatio = 0;
    REQUIRE_NOTHROW(config.Validate());
    World world;
    SimulationConfig invalid;
    invalid.jointHertz = 0;
    REQUIRE_THROWS_AS(world.setSimulationConfig(invalid), std::invalid_argument);
    REQUIRE(world.getSimulationConfig().jointHertz == Catch::Approx(60.0f));
}

TEST_CASE("Joint substeps hold a chain with a 100x heavier end mass", "[joints][joint-substeps]") {
    const ChainRun rigid = RunHeavyChain(false);
    const ChainRun soft = RunHeavyChain(true);
    INFO("rigid stretch " << rigid.stretch << " separation " << rigid.separation << " late speed " << rigid.lateSpeed);
    INFO("soft stretch " << soft.stretch << " separation " << soft.separation << " late speed " << soft.lateSpeed);
    // Measured: rigid 0.53 m stretch, 2.3 m/s jitter; soft 0.14 m, 0.12 m/s.
    REQUIRE(soft.stretch < 0.2);
    REQUIRE(soft.stretch < 0.5 * rigid.stretch);
    REQUIRE(soft.separation < 0.03);
    REQUIRE(soft.lateSpeed < 0.25);
    REQUIRE(soft.lateSpeed < 0.25 * rigid.lateSpeed);
}

TEST_CASE("A substepped 1:1 chain comes to rest within the soft allowance", "[joints][joint-substeps]") {
    World world(Config(true));
    HangingChain(world, 0);
    double late = 0;
    for (int step = 0; step < 3 * 60; ++step) {
        world.step(Dt);
        if (step >= 2 * 60) late = std::max(late, MaxSpeed(world));
    }
    REQUIRE(late < 1e-3);
    // The soft rows sag under the chain's weight, below half the slop.
    REQUIRE(MaxSeparation(world) < 0.5 * world.getSimulationConfig().penetrationSlop);
}

TEST_CASE("Substepped revolute motors hold their speed and torque cap", "[joints][motor][joint-substeps]") {
    SECTION("speed under a gravity load") {
        World world(Config(true));
        world.addUniversalForce(std::make_unique<Gravity>(Vector2{0, -9.81f}));
        auto base = Add(world, Circle(0.05f), {0, 2}, true);
        auto arm = Add(world, Polygon::MakeBox(1.0f, 0.1f), {0.5f, 2});
        auto motor = std::make_shared<RevoluteJoint>(base, arm, Vector2{}, Vector2{-0.5f, 0});
        motor->setMotor(true, 2.0f, 200);
        world.addJoint(motor);
        double error = 0;
        for (int step = 0; step < 4 * 60; ++step) {
            world.step(Dt);
            if (step >= 60) error = std::max(error, std::abs(double(arm->angularVelocity) - 2.0));
        }
        REQUIRE(error < 2e-3);
        REQUIRE(MaxSeparation(world) < 0.005);
    }
    SECTION("cap below the load") {
        World world(Config(true));
        world.addUniversalForce(std::make_unique<Gravity>(Vector2{0, -9.81f}));
        auto base = Add(world, Circle(0.05f), {0, 2}, true);
        auto arm = Add(world, Polygon::MakeBox(1.0f, 0.1f), {0.5f, 2});
        const double cap = 0.25 * arm->GetMass() * 9.81;
        auto motor = std::make_shared<RevoluteJoint>(base, arm, Vector2{}, Vector2{-0.5f, 0});
        motor->setMotor(true, 1.0f, float(cap));
        world.addJoint(motor);
        double torque = 0;
        for (int step = 0; step < 60; ++step) {
            world.step(Dt);
            torque = std::max(torque, std::abs(motor->getMotorTorque()));
        }
        REQUIRE(torque <= cap * (1 + 1e-6));
        REQUIRE(torque == Catch::Approx(cap).epsilon(1e-6));
        REQUIRE(motor->getAngle() < -1.5);
    }
}

TEST_CASE("Substepped revolute limits hold", "[joints][joint-substeps]") {
    World world(Config(true));
    world.addUniversalForce(std::make_unique<Gravity>(Vector2{0, -9.81f}));
    auto base = Add(world, Circle(0.05f), {0, 2}, true);
    auto arm = Add(world, Polygon::MakeBox(1.0f, 0.1f), {0.5f, 2});
    auto hinge = std::make_shared<RevoluteJoint>(base, arm, Vector2{}, Vector2{-0.5f, 0});
    hinge->setLimits(true, -0.5f, 0.5f);
    world.addJoint(hinge);
    double lowest = 0;
    for (int step = 0; step < 3 * 60; ++step) {
        world.step(Dt);
        lowest = std::min(lowest, hinge->getAngle());
    }
    REQUIRE(lowest > -0.5 - 0.02);
    REQUIRE(hinge->getAngle() == Catch::Approx(-0.5).margin(0.01));
    REQUIRE(MaxSpeed(world) < 1e-2);
}

TEST_CASE("Substepped distance springs and rigid rods keep working", "[joints][joint-substeps]") {
    World world(Config(true));
    world.addUniversalForce(std::make_unique<Gravity>(Vector2{0, -9.81f}));
    auto anchor = Add(world, Circle(0.05f), {0, 5}, true);
    auto rodBob = Add(world, Circle(0.1f), {1, 5});
    auto springBob = Add(world, Circle(0.1f), {0, 4});
    rodBob->SetCollisionMaskBits(0);
    springBob->SetCollisionMaskBits(0);
    auto rod = std::make_shared<DistanceJoint>(anchor, rodBob, 1.0f);
    auto spring = std::make_shared<DistanceJoint>(anchor, springBob, 1.0f);
    spring->setSpring(2.0f, 1.0f);
    world.addJoint(rod);
    world.addJoint(spring);
    double rodError = 0;
    for (int step = 0; step < 6 * 60; ++step) {
        world.step(Dt);
        rodError = std::max(rodError, std::abs(rod->getCurrentLength() - 1.0));
    }
    REQUIRE(rodError < 0.01);
    // Critically damped 2 Hz spring at rest: stretched by g / omega^2.
    const double omega = 2 * 3.14159265358979 * 2.0;
    REQUIRE(spring->getCurrentLength() == Catch::Approx(1.0 + 9.81 / (omega * omega)).margin(2e-3));
}

TEST_CASE("Joint substeps leave other joint types on the rigid solve", "[joints][joint-substeps]") {
    const auto run = [](bool jointSubsteps) {
        World world(Config(jointSubsteps));
        world.addUniversalForce(std::make_unique<Gravity>(Vector2{0, -9.81f}));
        auto ground = Add(world, Polygon::MakeBox(20, 2), {0, -1}, true);
        auto platform = Add(world, Polygon::MakeBox(2, 0.2f), {0, 0.1f});
        auto slider = std::make_shared<PrismaticJoint>(ground, platform, Vector2{0, 1}, Vector2{0, 1.0f}, Vector2{});
        slider->setMotor(true, 1.0f, 2000);
        world.addJoint(slider);
        auto rider = Add(world, Polygon::MakeBox(0.5f, 0.5f), {0.3f, 0.45f});
        // A revolute joint in the same group does not make it substep.
        auto flag = Add(world, Polygon::MakeBox(0.1f, 0.4f), {-0.8f, 0.4f});
        world.addJoint(std::make_shared<RevoluteJoint>(platform, flag, Vector2{-0.8f, 0.1f}, Vector2{0, -0.2f}));
        for (int step = 0; step < 90; ++step) world.step(Dt);
        return std::vector<float>{platform->position.y, platform->velocity.y, rider->position.x, rider->position.y,
                                  flag->position.x, flag->orientation};
    };
    REQUIRE(run(true) == run(false));
}

TEST_CASE("Toggling joint substeps mid-run keeps a loaded chain intact", "[joints][joint-substeps]") {
    World world(Config(false));
    HangingChain(world, 10);
    double separation = 0;
    for (int phase = 0; phase < 4; ++phase) {
        auto config = world.getSimulationConfig();
        config.enableJointSubsteps = phase % 2 == 0;
        world.setSimulationConfig(config);
        for (int step = 0; step < 60; ++step) {
            world.step(Dt);
            separation = std::max(separation, MaxSeparation(world));
        }
    }
    REQUIRE(separation < 0.02);
    REQUIRE(MaxSpeed(world) < 0.05);
}

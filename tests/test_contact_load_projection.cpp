#include "../benchmarks/experimental/contact_load_projection.h"
#include "../benchmarks/rigid_contact_metrics.h"
#include "catch_amalgamated.hpp"

using namespace PhysicsEngine;
namespace {
SimulationConfig LoadConfig() {
    SimulationConfig c;
    c.solverIterations = 64;
    c.velocityTolerance = 0;
    c.positionCorrectionFactor = 0;
    c.warmStartFactor = 0;
    c.enableLinearVelocityLimit = false;
    c.enableAngularVelocityLimit = false;
    c.restitutionVelocityThreshold = 0;
    return c;
}
template <class W> std::pair<RigidBodyPtr, RigidBodyPtr> Floor(W &world, float friction = 1) {
    auto a = std::make_shared<RigidBody>(
        Polygon::MakeBox(20, 1), Material{1, 0, friction, friction}, Vector2{0, -.5f}, true);
    auto b = std::make_shared<RigidBody>(Polygon::MakeBox(1, 1), Material{1, 0, friction, friction},
                                         Vector2{0, .499f});
    b->SetMass(1);
    world.addBody(a);
    world.addBody(b);
    return {a, b};
}
} // namespace
TEST_CASE("Experimental persistent incline equilibrium retains position and balances its load",
          "[contact-load]") {
    ContactLoadExperiment::World world(LoadConfig());
    const auto pair = Floor(world);
    const float theta = 3.14159265358979323846f / 12;
    const Vector2 n{float(-std::sin(double(theta))), float(std::cos(double(theta)))};
    pair.first->SetPosition(n * -.5f);
    pair.first->SetOrientation(theta);
    pair.second->SetPosition(n * .499f);
    pair.second->SetOrientation(theta);
    const auto start = pair.second->position;
    world.addUniversalForce(std::make_unique<Gravity>(Vector2{0, -9.81f}));
    double reactionY = 0;
    for (int i = 0; i < 120; ++i) {
        world.step(1.f / 60);
        reactionY += world.lastExperiment().bodies[1].loadedReactionY -
                     world.lastExperiment().bodies[1].unloadedReactionY;
        REQUIRE(world.lastExperiment().loadedClosingResidual < 1e-14);
        REQUIRE(world.lastExperiment().loadedTangentialSpeed < 1e-14);
    }
    REQUIRE(pair.second->position.x == Catch::Approx(start.x).epsilon(0).margin(2e-6));
    REQUIRE(pair.second->position.y == Catch::Approx(start.y).epsilon(0).margin(2e-6));
    REQUIRE(reactionY == Catch::Approx(double(9.81f) * (1.f / 60) * 120).epsilon(0).margin(1e-12));
}
TEST_CASE("Experimental exact flat touching reports delayed manifold onset without adding overlap",
          "[contact-load]") {
    ContactLoadExperiment::World world(LoadConfig());
    const auto pair = Floor(world);
    pair.second->SetPosition({0, .5f});
    world.addUniversalForce(std::make_unique<Gravity>(Vector2{0, -8}));
    REQUIRE_FALSE(CheckCollision(pair.first.get(), pair.second.get()).hasCollision);
    world.step(.125f);
    REQUIRE(world.lastExperiment().eligibility == "no_start_contacts");
    REQUIRE(world.lastExperiment().scratchSolves == 0);
    REQUIRE(pair.second->position.y == .4375f);
    world.step(.125f);
    REQUIRE(world.lastExperiment().pairs == 1);
    REQUIRE(pair.second->position.y == .4375f);
}
TEST_CASE("Experimental unsupported force sleeping and moving-support policies delegate exactly",
          "[contact-load]") {
    const int mode = GENERATE(0, 1, 2, 3, 4);
    auto config = LoadConfig();
    if (mode == 1)
        config.enableSleeping = true;
    if (mode == 3)
        config.enableLinearVelocityLimit = true;
    ContactLoadExperiment::World experiment(config);
    PhysicsEngine::World baseline(config);
    const auto e = Floor(experiment), b = Floor(baseline);
    struct CountedForce : IForceGenerator {
        int &count;
        explicit CountedForce(int &n) : count(n) {}
        void applyForce(RigidBody *body) override {
            ++count;
            body->ApplyForce({0, -8 * body->mass});
        }
    };
    int ec = 0, bc = 0;
    if (mode == 0) {
        experiment.addUniversalForce(std::make_unique<CountedForce>(ec));
        baseline.addUniversalForce(std::make_unique<CountedForce>(bc));
    } else {
        experiment.addUniversalForce(std::make_unique<Gravity>(Vector2{0, -8}));
        baseline.addUniversalForce(std::make_unique<Gravity>(Vector2{0, -8}));
    }
    if (mode == 2) {
        e.first->SetVelocity({1, 0});
        b.first->SetVelocity({1, 0});
    }
    if (mode == 4)
        for (int i = 0; i < 31; ++i) {
            auto eb =
                std::make_shared<RigidBody>(Circle(.1f), Material{}, Vector2{float(100 + i), 100});
            auto bb =
                std::make_shared<RigidBody>(Circle(.1f), Material{}, Vector2{float(100 + i), 100});
            experiment.addBody(eb);
            baseline.addBody(bb);
        }
    RigidContactDiagnostic::Tracker et, bt;
    experiment.addCollisionListener(&et);
    baseline.addCollisionListener(&bt);
    const char *reasons[] = {"opaque_force", "sleeping", "moving_support", "velocity_caps",
                             "budget"};
    for (int i = 0; i < 8; ++i) {
        experiment.step(.125f);
        baseline.step(.125f);
        REQUIRE(experiment.lastExperiment().eligibility == reasons[mode]);
        REQUIRE(experiment.lastExperiment().scratchSolves == 0);
        REQUIRE(e.second->position == b.second->position);
        REQUIRE(e.second->velocity == b.second->velocity);
        REQUIRE(e.second->angularVelocity == b.second->angularVelocity);
        REQUIRE(e.second->IsAwake() == b.second->IsAwake());
    }
    REQUIRE(ec == bc);
    if (mode == 0)
        REQUIRE(ec == 8); // One awake dynamic body, once per step; static support is skipped.
    REQUIRE(et.begins == bt.begins);
    REQUIRE(et.persists == bt.persists);
    REQUIRE(et.ends == bt.ends);
    experiment.removeCollisionListener(&et);
    baseline.removeCollisionListener(&bt);
}

TEST_CASE(
    "Experimental load projection cancels a persistent support load without consuming it twice",
    "[contact-load]") {
    ContactLoadExperiment::World world(LoadConfig());
    const auto pair = Floor(world);
    const auto start = pair.second->position;
    world.addUniversalForce(std::make_unique<Gravity>(Vector2{0, -8}));
    for (int i = 0; i < 16; ++i) {
        world.step(.125f);
        REQUIRE(pair.second->position == start);
        REQUIRE(pair.second->velocity.magnitude() == Catch::Approx(0).epsilon(0).margin(1e-14));
        const auto &r = world.lastExperiment();
        REQUIRE(r.pairs == 1);
        REQUIRE(r.scratchSolves == 2);
        REQUIRE(r.bodies[1].loadY == -1);
        REQUIRE(r.bodies[1].loadedReactionY == Catch::Approx(1).epsilon(0).margin(1e-14));
        REQUIRE(pair.second->force == Vector2{});
    }
}
TEST_CASE("Experimental kinetic friction integrates its constant deceleration", "[contact-load]") {
    ContactLoadExperiment::World world(LoadConfig());
    const auto pair = Floor(world, .25f);
    pair.second->SetVelocity({2, 0});
    world.addUniversalForce(std::make_unique<Gravity>(Vector2{0, -8}));
    for (int i = 0; i < 4; ++i)
        world.step(.125f);
    // Coulomb a=-mu*g=-2, t=.5: x=.75, v=1, with no stop during the step.
    REQUIRE(pair.second->position.x == Catch::Approx(.75).epsilon(0).margin(2e-6));
    REQUIRE(pair.second->velocity.x == Catch::Approx(1).epsilon(0).margin(2e-6));
    REQUIRE(pair.second->angularVelocity == Catch::Approx(0).epsilon(0).margin(2e-6));
}
TEST_CASE("Experimental separating load releases support and returns to ballistic flight",
          "[contact-load]") {
    ContactLoadExperiment::World world(LoadConfig());
    const auto pair = Floor(world);
    auto gravity = std::make_unique<Gravity>(Vector2{0, -8});
    auto *change = gravity.get();
    world.addUniversalForce(std::move(gravity));
    world.step(.125f);
    const double y = pair.second->position.y;
    change->setGravity({0, 8});
    world.step(.125f);
    REQUIRE(pair.second->position.y == Catch::Approx(y + .0625).epsilon(0).margin(1e-7));
    REQUIRE(pair.second->velocity.y == 1);
    REQUIRE(world.lastExperiment().bodies[1].loadedReactionY ==
            Catch::Approx(0).epsilon(0).margin(1e-14));
    world.step(.125f);
    REQUIRE(world.lastExperiment().eligibility == "no_start_contacts");
    REQUIRE(pair.second->position.y == Catch::Approx(y + .25).epsilon(0).margin(1e-7));
    REQUIRE(pair.second->velocity.y == 2);
}
TEST_CASE("Experimental free flight retains one-shot force and torque accuracy", "[contact-load]") {
    ContactLoadExperiment::World world(LoadConfig());
    auto b = std::make_shared<RigidBody>(Circle(1), Material{});
    b->SetMass(1);
    world.addBody(b);
    b->ApplyForce({8, 0});
    b->ApplyTorque(4);
    world.step(.25f);
    REQUIRE(world.lastExperiment().scratchSolves == 0);
    REQUIRE(b->position.x == .25f);
    REQUIRE(b->velocity.x == 2);
    REQUIRE(b->orientation == .25f);
    REQUIRE(b->angularVelocity == 2);
    REQUIRE(b->force == Vector2{});
    REQUIRE(b->torque == 0);
    world.step(.25f);
    REQUIRE(b->position.x == .75f);
    REQUIRE(b->orientation == .75f);
    REQUIRE(b->velocity.x == 2);
    REQUIRE(b->angularVelocity == 2);
}
TEST_CASE("Experimental new closing impacts are not averaged backward in time", "[contact-load]") {
    ContactLoadExperiment::World experiment(LoadConfig());
    PhysicsEngine::World baseline(LoadConfig());
    const auto e = Floor(experiment), b = Floor(baseline);
    e.second->SetVelocity({0, -2});
    b.second->SetVelocity({0, -2});
    experiment.addUniversalForce(std::make_unique<Gravity>(Vector2{0, -8}));
    baseline.addUniversalForce(std::make_unique<Gravity>(Vector2{0, -8}));
    experiment.step(.125f);
    baseline.step(.125f);
    REQUIRE(experiment.lastExperiment().scratchSolves == 0);
    REQUIRE(e.second->position == b.second->position);
    REQUIRE(e.second->velocity == b.second->velocity);
    REQUIRE(e.second->angularVelocity == b.second->angularVelocity);
}
TEST_CASE("Experimental instantaneous impact momentum angular momentum and energy match unchanged "
          "oracles",
          "[contact-load]") {
    for (const auto &spec : RigidContactDiagnostic::Fixtures(true))
        if (spec.impact) {
            const auto r = RigidContactDiagnostic::Run<ContactLoadExperiment::World>(
                spec, {1.f / 60, 64, 0, 2});
            REQUIRE(r.velocityError < 3e-7);
            // Retain the existing impact-oracle acceptance margins. The m=1000
            // row's rounded body velocities produce ~3.65e-7 momentum error.
            REQUIRE(r.final.px == Catch::Approx(r.initial.px).epsilon(0).margin(3e-6));
            REQUIRE(r.final.py == Catch::Approx(r.initial.py).epsilon(0).margin(3e-6));
            REQUIRE(r.final.angular == Catch::Approx(r.initial.angular).epsilon(0).margin(3e-6));
            REQUIRE(r.final.kinetic == Catch::Approx(r.expectedEnergy).epsilon(0).margin(3e-6));
            REQUIRE(r.positionalProjection == 0);
            const auto baseline = RigidContactDiagnostic::Run(spec, {1.f / 60, 64, 0, 2});
            REQUIRE(r.states == baseline.states);
        }
}
TEST_CASE("Experimental CCD and joint drives use the production step without correction",
          "[contact-load]") {
    const bool ccd = GENERATE(false, true);
    ContactLoadExperiment::World experiment(LoadConfig());
    PhysicsEngine::World baseline(LoadConfig());
    auto e = Floor(experiment), b = Floor(baseline);
    if (ccd) {
        experiment.removeBody(e.second);
        baseline.removeBody(b.second);
        e.second = std::make_shared<RigidBody>(Circle(.5f), Material{1, 0, 1, 1}, Vector2{0, 2});
        b.second = std::make_shared<RigidBody>(Circle(.5f), Material{1, 0, 1, 1}, Vector2{0, 2});
        experiment.addBody(e.second);
        baseline.addBody(b.second);
        e.second->SetCcdEnabled(true);
        b.second->SetCcdEnabled(true);
        e.second->SetPosition({0, 2});
        b.second->SetPosition({0, 2});
        e.second->SetVelocity({0, -10});
        b.second->SetVelocity({0, -10});
    } else {
        e.second->SetCollisionMaskBits(0);
        b.second->SetCollisionMaskBits(0);
        auto ej = std::make_shared<RevoluteJoint>(e.first, e.second);
        auto bj = std::make_shared<RevoluteJoint>(b.first, b.second);
        ej->setMotor(true, 1, 4);
        bj->setMotor(true, 1, 4);
        experiment.addJoint(ej);
        baseline.addJoint(bj);
    }
    experiment.addUniversalForce(std::make_unique<Gravity>(Vector2{0, -8}));
    baseline.addUniversalForce(std::make_unique<Gravity>(Vector2{0, -8}));
    unsigned ccdImpacts = 0;
    for (int i = 0; i < 4; ++i) {
        experiment.step(.125f);
        baseline.step(.125f);
        REQUIRE(experiment.lastExperiment().eligibility == (ccd ? "ccd" : "joints"));
        REQUIRE(e.second->position == b.second->position);
        REQUIRE(e.second->velocity == b.second->velocity);
        REQUIRE(e.second->orientation == b.second->orientation);
        REQUIRE(e.second->angularVelocity == b.second->angularVelocity);
        ccdImpacts += experiment.getLastStepStatistics().ccdImpactCount;
    }
    if (ccd)
        REQUIRE(ccdImpacts > 0);
}

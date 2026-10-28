#include "catch_amalgamated.hpp"
#include "physics/physics.h"
#include <cmath>
#include <memory>

using namespace PhysicsEngine;
using Catch::Approx;

namespace {
constexpr double G = double(9.81f);
SimulationConfig Config(bool correction, int iterations = 10, float warm = 0.8f) {
    SimulationConfig c;
    c.solverIterations = iterations;
    c.warmStartFactor = warm;
    c.enableLinearVelocityLimit = false;
    c.enableAngularVelocityLimit = false;
    c.restitutionVelocityThreshold = 0;
    c.enableFrictionDisplacementCorrection = correction;
    // The correction is a workaround inside the legacy step order.
    c.enableSpeculativeContacts = false;
    return c;
}
struct Incline {
    World world;
    RigidBodyPtr floor, box;
    Vector2 down, normal;
    Incline(double theta, float staticFriction, float dynamicFriction, const SimulationConfig& config)
        : world(config) {
        normal = {float(-std::sin(theta)), float(std::cos(theta))};
        down = {float(-std::cos(theta)), float(-std::sin(theta))};
        const Material material{1, 0, staticFriction, dynamicFriction};
        floor = std::make_shared<RigidBody>(Polygon::MakeBox(40, 1), material, normal * -0.5f, true);
        floor->SetOrientation(float(theta));
        // Start slightly inside the slop so contact exists from the first step.
        box = std::make_shared<RigidBody>(Polygon::MakeBox(1, 1), material, normal * 0.497f);
        box->SetMass(1);
        box->SetOrientation(float(theta));
        world.addBody(floor);
        world.addBody(box);
        world.addUniversalForce(std::make_unique<Gravity>(Vector2{0, -float(G)}));
    }
    double Downhill() const { return double((box->position - normal * 0.497f).dot(down)); }
    void Run(float dt, double duration) {
        for (int i = 0, n = int(std::lround(duration / dt)); i < n; ++i) world.step(dt);
    }
};
}

TEST_CASE("Friction displacement removes static incline creep", "[contact][friction][incline]") {
    const double theta = 3.14159265358979323846 / 12; // Below the 0.8 friction angle.
    for (const float dt : {1.0f / 60, 1.0f / 120, 1.0f / 240}) {
        Incline legacy(theta, 0.8f, 0.6f, Config(false)), corrected(theta, 0.8f, 0.6f, Config(true));
        // Measure the second second, after the start-up contact transient.
        legacy.Run(dt, 1);
        corrected.Run(dt, 1);
        const double legacyStart = legacy.Downhill(), correctedStart = corrected.Downhill();
        legacy.Run(dt, 1);
        corrected.Run(dt, 1);
        // The legacy order integrates the load before the friction impulse,
        // creeping about g sin(theta) dt / 2 per second downhill.
        const double creep = 0.5 * G * std::sin(theta) * dt;
        const double legacyCreep = legacy.Downhill() - legacyStart, correctedCreep = corrected.Downhill() - correctedStart;
        INFO("dt=" << dt << " legacy=" << legacyCreep << " corrected=" << correctedCreep << " total=" << corrected.Downhill());
        REQUIRE(legacyCreep > 0.9 * creep);
        // What remains is the unconverged solver's own residual velocity.
        REQUIRE(std::abs(correctedCreep) < 0.02 * creep);
        REQUIRE(std::abs(corrected.Downhill()) < 0.03 * 2 * creep);
        REQUIRE(corrected.box->velocity.magnitude() < 1e-3f);
        // Initial settling tilts both variants equally; the correction adds none.
        const auto tilt = [&](const Incline& i) {
            return std::abs(std::remainder(double(i.box->orientation) - theta, 6.283185307179586));
        };
        REQUIRE(tilt(corrected) <= tilt(legacy) * 1.01 + 1e-6);
    }
}

TEST_CASE("Friction displacement matches analytic sliding and stick-slip limits", "[contact][friction][incline]") {
    // Friction angle atan(0.5) is about 26.57 degrees.
    const double stick = 25 * 3.14159265358979323846 / 180, slide = 28 * 3.14159265358979323846 / 180;
    const float dt = 1.0f / 120;
    Incline held(stick, 0.5f, 0.4f, Config(true, 20)), sliding(slide, 0.5f, 0.4f, Config(true, 20));
    Incline legacy(slide, 0.5f, 0.4f, Config(false, 20));
    for (auto* incline : {&held, &sliding, &legacy}) incline->Run(dt, 1);
    const double heldStart = held.Downhill(), slideStart = sliding.Downhill(), legacyStart = legacy.Downhill();
    const double slideSpeed = sliding.box->velocity.dot(sliding.down), legacySpeed = legacy.box->velocity.dot(legacy.down);
    for (auto* incline : {&held, &sliding, &legacy}) incline->Run(dt, 1);
    REQUIRE(std::abs(held.Downhill() - heldStart) < 0.01 * 0.5 * G * std::sin(stick) * dt);
    // Kinetic friction gives constant acceleration g (sin - mu_k cos). Over the
    // second second the position must follow s = v t + a t^2 / 2 exactly.
    const double acceleration = G * (std::sin(slide) - double(0.4f) * std::cos(slide));
    const double measured = sliding.box->velocity.dot(sliding.down) - slideSpeed;
    REQUIRE(measured == Approx(acceleration).epsilon(1e-5));
    const double expected = slideSpeed + 0.5 * acceleration, legacyExpected = legacySpeed + 0.5 * acceleration;
    const double correctedError = sliding.Downhill() - slideStart - expected;
    const double legacyError = legacy.Downhill() - legacyStart - legacyExpected;
    INFO("corrected error=" << correctedError << " legacy error=" << legacyError);
    REQUIRE(std::abs(correctedError) < 1e-5);
    // The legacy offset is the O(dt) load-before-impulse term mu_k g cos dt / 2.
    REQUIRE(legacyError == Approx(0.5 * double(0.4f) * G * std::cos(slide) * dt).epsilon(0.02));
}

TEST_CASE("Friction displacement keeps the center of mass on its momentum path", "[contact][friction][momentum]") {
    // Two free boxes pressed together by equal and opposite loads slide against
    // each other; contact corrections are internal, so the mass center moves
    // exactly as its total momentum prescribes.
    struct Push : IForceGenerator {
        Vector2 force;
        explicit Push(Vector2 f) : force(f) {}
        void applyForce(RigidBody* body) override { body->ApplyForce(force); }
    };
    World world(Config(true, 10, 0));
    const Material material{1, 0, 0.6f, 0.4f};
    auto lower = std::make_shared<RigidBody>(Polygon::MakeBox(4, 1), material, Vector2{0, 0});
    auto upper = std::make_shared<RigidBody>(Polygon::MakeBox(1, 1), material, Vector2{0.25f, 0.995f});
    lower->SetMass(3);
    upper->SetMass(1);
    upper->SetVelocity({2, 0});
    world.addBody(lower);
    world.addBody(upper);
    world.addForce(upper, std::make_unique<Push>(Vector2{0, -5}));
    world.addForce(lower, std::make_unique<Push>(Vector2{0, 5}));
    const auto center = [&] { return (lower->position * 3.0f + upper->position) / 4.0f; };
    const Vector2 start = center();
    const double dt = 1.0 / 120;
    for (int i = 0; i < 60; ++i) world.step(float(dt));
    // Initial momentum (2, 0) over total mass 4.
    REQUIRE(double(center().x) == Approx(start.x + 0.5 * 60 * dt).margin(2e-6));
    REQUIRE(double(center().y) == Approx(start.y).margin(2e-6));
    REQUIRE(upper->velocity.x < 2.0f); // Friction did act.
}

TEST_CASE("Friction displacement keeps resting contacts for sleeping and waking", "[contact][friction][sleep]") {
    auto config = Config(true);
    config.enableSleeping = true;
    World world(config);
    auto ball = std::make_shared<RigidBody>(Circle(0.5f), Material{1, 0}, Vector2(0, 0.5f));
    auto floor = std::make_shared<RigidBody>(Polygon::MakeBox(10, 1), Material{1, 0}, Vector2(0, -0.5f), true);
    world.addBody(ball);
    world.addBody(floor);
    world.addUniversalForce(std::make_unique<Gravity>(Vector2(0, -9.81f)));
    for (int i = 0; i < 120; ++i) world.step();
    REQUIRE_FALSE(ball->IsAwake());
    floor->SetPosition({0, -5});
    world.step();
    REQUIRE(ball->IsAwake());
    REQUIRE(ball->velocity.y < 0);
}

TEST_CASE("Friction displacement leaves free flight and disabled steps unchanged", "[contact][friction]") {
    for (const bool correction : {false, true}) {
        World world(Config(correction));
        auto body = std::make_shared<RigidBody>(Circle(0.5f), Material{}, Vector2(0, 10));
        body->SetVelocity({1, 2});
        world.addBody(body);
        world.addUniversalForce(std::make_unique<Gravity>(Vector2(0, -9.81f)));
        for (int i = 0; i < 30; ++i) world.step(1.0f / 60);
        const double t = 0.5;
        REQUIRE(double(body->position.x) == Approx(t).margin(1e-5));
        REQUIRE(double(body->position.y) == Approx(10 + 2 * t - 0.5 * G * t * t).margin(1e-5));
    }
}

// A CCD circle must not pass through a static wall, whatever drives it: a
// joint's velocity solve, a joint's or contact's position correction, a
// force, an impulse or a set velocity. Every case runs in both step orders.
#include "catch_amalgamated.hpp"
#include "physics/physics.h"
#include <algorithm>
#include <cmath>
#include <functional>

using namespace PhysicsEngine;

namespace {
constexpr float Step = 1.0f / 60;

SimulationConfig Order(bool speculative) {
    SimulationConfig config;
    config.enableSpeculativeContacts = speculative;
    return config;
}

// Signed distance from p to an axis-aligned box (negative inside).
double BoxDistance(Vector2 p, Vector2 center, Vector2 half) {
    const double dx = std::abs(double(p.x) - center.x) - half.x, dy = std::abs(double(p.y) - center.y) - half.y;
    const double outside = std::hypot(std::max(dx, 0.0), std::max(dy, 0.0));
    return outside + std::min(std::max(dx, dy), 0.0);
}

RigidBodyPtr DragAnchor() {
    auto ground = std::make_shared<RigidBody>(Circle(0.1f), Material{}, Vector2{0, -50}, true);
    ground->SetCollisionMaskBits(0);
    return ground;
}

// A 0.25 kg CCD circle starting at the origin and a 0.5 m static wall whose
// near face is at x = 2.75. The wall is tall enough that the only way past
// it is through it.
struct WallScene {
    World world;
    RigidBodyPtr ball, wall;
    WallScene(bool speculative, float radius, float startX = 0, float wallHeight = 2000)
        : world(Order(speculative)) {
        wall = std::make_shared<RigidBody>(Polygon::MakeBox(0.5f, wallHeight), Material{}, Vector2{3, 0}, true);
        ball = std::make_shared<RigidBody>(Circle(radius), Material{}, Vector2{startX, 0});
        ball->SetMass(0.25f);
        ball->SetCcdEnabled(true);
        world.addBody(wall);
        world.addBody(ball);
    }
    // Deepest overlap of the ball with the wall over the run, and whether the
    // ball ever reached the far side.
    struct Result { double deepest = -1e9; bool through = false; float peakSpeed = 0; };
    Result run(int steps, const std::function<void(int)>& drive = {}) {
        Result r;
        const float radius = ball->shape->GetRadius();
        for (int i = 0; i < steps; ++i) {
            if (drive) drive(i);
            world.step(Step);
            REQUIRE(std::isfinite(ball->position.x));
            REQUIRE(std::isfinite(ball->position.y));
            r.deepest = std::max(r.deepest, radius - BoxDistance(ball->position, {3, 0}, {0.25f, 1000}));
            r.through = r.through || ball->position.x > 3;
            r.peakSpeed = std::max(r.peakSpeed, ball->velocity.magnitude());
        }
        return r;
    }
};
}

TEST_CASE("Mouse-joint drags never carry a CCD circle through a static wall", "[ccd][ccd-driven][mouse]") {
    // The reported setup: 30 Hz, damping 0.7, at Box2D's force scale
    // (1000 m g, about 163 m/s per step) and at 1e5 N (thousands of m/s: the
    // joint's impulse is applied after the maxLinearSpeed clamp).
    const float mass = 0.25f;
    for (const bool speculative : {true, false})
    for (const float maxForce : {1000 * mass * 9.81f, 1e5f})
    for (const float radius : {0.25f, 0.05f})
    for (const Vector2 anchor : {Vector2{}, Vector2{0.1f, 0.1f}}) {
        WallScene scene(speculative, radius);
        auto ground = DragAnchor();
        scene.world.addBody(ground);
        scene.world.addJoint(std::make_shared<MouseJoint>(ground, scene.ball, anchor, Vector2{100, 0}, 30, 0.7f,
                                                          maxForce));
        const auto r = scene.run(240);
        INFO("speculative " << speculative << ", maxForce " << maxForce << ", radius " << radius << ", anchor "
                            << anchor.x << ": deepest " << r.deepest << ", peak speed " << r.peakSpeed);
        CHECK_FALSE(r.through);
        CHECK(r.deepest <= 0.005); // penetration slop
    }
}

TEST_CASE("A CCD circle fired at a wall with a drag's velocity profile stays in front", "[ccd][ccd-driven]") {
    // The speeds a 1000 m g drag reaches (163, 363 m/s per step) set directly,
    // with the linear limit off so they are not clamped to 200 m/s.
    for (const bool speculative : {true, false})
    for (const float speed : {163.5f, 363.f, 4000.f}) {
        SimulationConfig config = Order(speculative);
        config.enableLinearVelocityLimit = false;
        WallScene scene(speculative, 0.25f);
        scene.world.setSimulationConfig(config);
        const auto r = scene.run(60, [&](int) { scene.ball->SetVelocity({speed, 0}); });
        INFO("speculative " << speculative << ", speed " << speed << ": deepest " << r.deepest);
        CHECK_FALSE(r.through);
        CHECK(r.deepest <= 0.005);
    }
}

TEST_CASE("Forces, impulses and set velocities never carry a CCD circle through a wall", "[ccd][ccd-driven]") {
    for (const bool speculative : {true, false})
    for (const float radius : {0.25f, 0.05f})
    for (const float startX : {0.0f, 2.49f})
    for (int driver = 0; driver < 3; ++driver)
    for (const float scale : {10.f, 1000.f, 1e5f}) {
        WallScene scene(speculative, radius, startX);
        auto& ball = *scene.ball;
        const auto r = scene.run(120, [&](int i) {
            if (driver == 0) ball.ApplyForce({scale * 0.25f * 9.81f, 0});
            if (driver == 1 && i == 0) ball.ApplyImpulse({scale, 0}, {0, 0.1f});
            if (driver == 2) ball.SetVelocity({scale, 0.1f * scale}); // clamped to maxLinearSpeed
        });
        INFO("speculative " << speculative << ", radius " << radius << ", start " << startX << ", driver "
                            << driver << ", scale " << scale << ": deepest " << r.deepest);
        CHECK_FALSE(r.through);
        CHECK(r.deepest <= 0.005);
    }
}

TEST_CASE("Joint position correction never pulls a CCD circle through a static wall", "[ccd][ccd-driven][joint]") {
    // A heavy box that ignores the wall drags a CCD circle on a rigid
    // distance joint, a rope or a revolute joint. The velocity solve is swept,
    // but before the fix the position correction that follows it moved the
    // circle through the wall: with the rigid rod perpendicular to the drag
    // the circle had no velocity at all and still crossed in one step.
    const auto boxClearance = [](Vector2 p) { return BoxDistance(p, {0, 0}, {0.05f, 1000}); };
    const auto circleClearance = [](Vector2 p) { return std::hypot(double(p.x) - 1000.05, double(p.y)) - 1000; };
    const auto capsuleClearance = [](Vector2 p) {
        return std::hypot(double(p.x), std::max(std::abs(double(p.y)) - 1000, 0.0)) - 0.05;
    };
    for (const bool speculative : {true, false})
    for (int wallKind = 0; wallKind < 3; ++wallKind)
    for (int jointKind = 0; jointKind < 3; ++jointKind)
    for (const float speed : {5.f, 50.f, 200.f}) {
        World world(Order(speculative));
        // Each wall's near surface is the line x = -0.05 near the origin.
        RigidBodyPtr wall;
        double (*clearance)(Vector2) = nullptr;
        if (wallKind == 0) {
            wall = std::make_shared<RigidBody>(Polygon::MakeBox(0.1f, 2000), Material{}, Vector2{0, 0}, true);
            clearance = boxClearance;
        } else if (wallKind == 1) {
            wall = std::make_shared<RigidBody>(Circle(1000), Material{}, Vector2{1000.05f, 0}, true);
            clearance = circleClearance;
        } else {
            wall = std::make_shared<RigidBody>(Capsule(1000, 0.05f), Material{}, Vector2{0, 0}, true);
            wall->SetOrientation(1.5707963267948966f);
            clearance = capsuleClearance;
        }
        auto ball = std::make_shared<RigidBody>(Circle(0.05f), Material{}, Vector2{-0.5f, 0});
        ball->SetMass(0.25f);
        ball->SetCcdEnabled(true);
        auto heavy = std::make_shared<RigidBody>(Polygon::MakeBox(0.2f, 0.2f), Material{}, Vector2{-0.5f, 0.5f});
        heavy->SetMass(100);
        heavy->SetCollisionMaskBits(0); // passes the wall freely
        heavy->SetVelocity({speed, 0});
        world.addBody(wall);
        world.addBody(ball);
        world.addBody(heavy);
        if (jointKind == 0) world.addJoint(std::make_shared<DistanceJoint>(heavy, ball, 0.5f));
        if (jointKind == 1) {
            auto rope = std::make_shared<DistanceJoint>(heavy, ball, 0.5f);
            rope->setLengthRange(0, 0.5f);
            world.addJoint(rope);
        }
        if (jointKind == 2) world.addJoint(std::make_shared<RevoluteJoint>(heavy, ball, Vector2{0, -0.5f}, Vector2{}));
        double deepest = -1e9;
        bool through = false;
        for (int i = 0; i < 60; ++i) {
            world.step(Step);
            REQUIRE(std::isfinite(ball->position.x));
            REQUIRE(std::isfinite(ball->position.y));
            deepest = std::max(deepest, 0.05 - clearance(ball->position));
            // The disk wall curves away from the line x = -0.05, so sliding
            // along it reaches x > 0; it is too large to cross in a run.
            through = through || (wallKind == 1 ? clearance(ball->position) < 0 : ball->position.x > 0);
        }
        INFO("speculative " << speculative << ", wall " << wallKind << ", joint " << jointKind << ", speed "
                            << speed << ": deepest " << deepest);
        CHECK_FALSE(through);
        CHECK(deepest <= 0.005);
    }
}

TEST_CASE("A circle dragged along a short wall goes around its end, not through it", "[ccd][ccd-driven][mouse]") {
    // The original report's geometry: a 6 m wall and an off-center anchor.
    // The spun circle slides up the face and leaves over the top, which an
    // x-only check reads as tunnelling. It never overlaps the wall.
    for (const bool speculative : {true, false})
    for (const float maxForce : {1000 * 0.25f * 9.81f, 1e5f}) {
        WallScene scene(speculative, 0.25f, 0, 6);
        auto ground = DragAnchor();
        scene.world.addBody(ground);
        scene.world.addJoint(std::make_shared<MouseJoint>(ground, scene.ball, Vector2{0.1f, 0.1f}, Vector2{100, 0},
                                                          30, 0.7f, maxForce));
        double deepest = -1e9;
        for (int i = 0; i < 120; ++i) {
            const Vector2 before = scene.ball->position;
            scene.world.step(Step);
            const Vector2 after = scene.ball->position;
            deepest = std::max(deepest, 0.25 - BoxDistance(after, {3, 0}, {0.25f, 3}));
            // A step that crosses from the near side to the far side does so
            // above or below the wall.
            if (before.x < 2.5f && after.x > 3.5f) {
                const double t = (2.75 - before.x) / (double(after.x) - before.x);
                CHECK(std::abs(before.y + t * (double(after.y) - before.y)) > 3);
            }
        }
        INFO("speculative " << speculative << ", maxForce " << maxForce << ": deepest " << deepest);
        CHECK(deepest <= 0.005);
    }
}

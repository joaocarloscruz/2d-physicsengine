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
RigidBodyPtr Box(Vector2 p, float w, float h, bool fixed = false) {
    auto body = std::make_shared<RigidBody>(Polygon::MakeBox(w, h), Material{1, 0}, p, fixed);
    body->SetCollisionMaskBits(0);
    return body;
}
Vector2 Tip(const RigidBody& body, Vector2 local) {
    const double c = std::cos(double(body.orientation)), s = std::sin(double(body.orientation));
    return {float(body.position.x + c * local.x - s * local.y), float(body.position.y + s * local.x + c * local.y)};
}
struct Sag { double worst = 0, final = 0, tipSpeed = 0, worstAngle = 0, worstSeparation = 0; };
Sag Cantilever(int segments, int iterations, float warmStart) {
    SimulationConfig config = FreeConfig();
    config.solverIterations = iterations;
    config.warmStartFactor = warmStart;
    World world(config);
    world.addUniversalForce(std::make_unique<Gravity>(Vector2{0, -9.81f}));
    auto wall = Box({0, 0}, 0.2f, 2, true);
    world.addBody(wall);
    RigidBodyPtr previous = wall;
    Vector2 previousAnchor{0.1f, 0};
    RigidBodyPtr tip;
    std::vector<std::shared_ptr<WeldJoint>> welds;
    const float length = 4.f / segments;
    for (int i = 0; i < segments; ++i) {
        auto piece = Box({0.1f + length * (i + 0.5f), 0}, length, 0.2f);
        world.addBody(piece);
        welds.push_back(std::make_shared<WeldJoint>(previous, piece, previousAnchor, Vector2{-length / 2, 0}));
        world.addJoint(welds.back());
        previous = tip = piece;
        previousAnchor = {length / 2, 0};
    }
    const Vector2 tipLocal{length / 2, 0};
    const float startY = Tip(*tip, tipLocal).y;
    Sag sag;
    for (int i = 0; i < 600; ++i) {
        world.step(1.f / 60);
        sag.final = startY - Tip(*tip, tipLocal).y;
        sag.worst = std::max(sag.worst, std::abs(sag.final));
    }
    sag.tipSpeed = tip->velocity.magnitude();
    for (const auto& weld : welds) {
        sag.worstAngle = std::max(sag.worstAngle, std::abs(weld->getAngle()));
        sag.worstSeparation = std::max(sag.worstSeparation, weld->getSeparation());
    }
    return sag;
}
}

TEST_CASE("Welded cantilever holds its pose under gravity", "[weld]") {
    // A 4 m x 0.2 m beam welded at its left end to a static wall, as one body
    // and as four 1 m segments (one wall weld and three beam welds). Sag is the
    // tip drop from the construction pose over 10 s at 60 Hz.
    struct Case { int segments, iterations; double worstBound; };
    for (const Case c : {Case{1, 10, 0.005}, Case{4, 10, 0.15}, Case{4, 30, 0.03}}) {
        const Sag sag = Cantilever(c.segments, c.iterations, 1);
        INFO(c.segments << " segment(s), " << c.iterations << " iterations: worst tip sag " << sag.worst
                        << " m, after 10 s " << sag.final << " m, tip speed " << sag.tipSpeed
                        << ", worst weld angle " << sag.worstAngle << ", separation " << sag.worstSeparation);
        // One weld is solved exactly (3x3 block); what remains is the per-step
        // gravity drift g*h^2/2 = 1.4 mm, which projection removes. A chain is
        // Gauss-Seidel over the welds, so on first load it rings like a stiff
        // beam until warm starting carries the load.
        CHECK(sag.worst < c.worstBound);
        CHECK(std::abs(sag.final) < 0.001);
        CHECK(sag.tipSpeed < 0.01);
        CHECK(sag.worstAngle < 0.005);
        CHECK(sag.worstSeparation < 0.005);
    }
    // Without warm starting the chain rebuilds its load from zero each step
    // and hangs visibly lower.
    const Sag cold = Cantilever(4, 10, 0), warm = Cantilever(4, 10, 1);
    INFO("cold-start sag after 10 s " << cold.final << " m vs warm " << warm.final << " m");
    CHECK(cold.final > 10 * std::abs(warm.final));
}

TEST_CASE("Two welded boxes fly like the equivalent single body", "[weld][oracle]") {
    // Two 1x1 boxes side by side have the mass (2) and inertia (5/6) of one
    // 2x1 box, so with the same launch they share one rigid trajectory.
    // Oracle for the difference: each half moves on a chord of its arc and
    // projection pulls it back radially, which scales the halves' orbital
    // angular momentum by 1 - (w*h)^2/2 per step. With orbital share
    // 0.25/(5/12) = 0.6 the spin falls by 0.6 * w^3 * h * T / 2, first order
    // in h. The test checks that model and its convergence.
    const double omega = 2, duration = 2, share = 0.6;
    double previousSpinError = 0;
    for (double h : {1.0 / 60, 1.0 / 240}) {
        World pair(FreeConfig()), single(FreeConfig());
        for (World* w : {&pair, &single}) w->addUniversalForce(std::make_unique<Gravity>(Vector2{0, -9.81f}));
        auto left = Box({-0.5f, 0}, 1, 1), right = Box({0.5f, 0}, 1, 1), whole = Box({0, 0}, 2, 1);
        REQUIRE(whole->mass == Catch::Approx(left->mass + right->mass));
        REQUIRE(whole->inertia == Catch::Approx(left->inertia + right->inertia + 0.25 * (left->mass + right->mass)));
        const float vx = 3, vy = 8, w = float(omega);
        // Rigid launch: v + omega x r for each half.
        left->SetVelocity({vx, vy - w * 0.5f});
        right->SetVelocity({vx, vy + w * 0.5f});
        left->SetAngularVelocity(w);
        right->SetAngularVelocity(w);
        whole->SetVelocity({vx, vy});
        whole->SetAngularVelocity(w);
        pair.addBody(left);
        pair.addBody(right);
        single.addBody(whole);
        pair.addJoint(std::make_shared<WeldJoint>(left, right, Vector2{0.5f, 0}, Vector2{-0.5f, 0}));
        double worstCenter = 0;
        const int steps = int(std::lround(duration / h));
        for (int i = 0; i < steps; ++i) {
            pair.step(float(h));
            single.step(float(h));
            const double cx = 0.5 * (double(left->position.x) + right->position.x);
            const double cy = 0.5 * (double(left->position.y) + right->position.y);
            worstCenter = std::max(worstCenter, std::hypot(cx - whole->position.x, cy - whole->position.y));
            REQUIRE(left->angularVelocity == Catch::Approx(right->angularVelocity).margin(1e-5));
        }
        const double spinError = double(whole->angularVelocity) - left->angularVelocity;
        const double angleError = std::remainder(double(whole->orientation) - left->orientation, 6.283185307179586);
        const double predicted = share * omega * omega * omega * h * duration / 2;
        INFO("h " << h << " over 2 s: spin loss " << spinError << " rad/s (model " << predicted
                  << "), angle lag " << angleError << " rad (model " << predicted * duration / 2
                  << "), worst center error " << worstCenter << " m");
        CHECK(spinError == Catch::Approx(predicted).epsilon(0.15));
        CHECK(angleError == Catch::Approx(predicted * duration / 2).epsilon(0.15));
        // Momentum is exact; the center differs only through the World's
        // per-body blend of solver velocity changes into positions.
        CHECK(worstCenter < 0.1 * h);
        if (previousSpinError > 0) CHECK(spinError / previousSpinError == Catch::Approx(0.25).epsilon(0.1));
        previousSpinError = spinError;
    }
}

TEST_CASE("Weld softness oscillates at the requested frequency", "[weld][oracle]") {
    for (bool angular : {true, false}) {
        for (double h : {1.0 / 60, 1.0 / 1000}) {
            World world(FreeConfig());
            auto base = Box({0, 0}, 0.2f, 0.2f, true), body = Box({1, 0}, 1, 0.5f);
            world.addBody(base);
            world.addBody(body);
            // Anchor at B's center decouples the two rows: the angular row's
            // mass is B's inertia and the linear row's is B's mass.
            auto joint = std::make_shared<WeldJoint>(base, body, Vector2{1, 0}, Vector2{});
            if (angular) joint->setAngularSpring(2, 0.1f); else joint->setLinearSpring(2, 0.1f);
            world.addJoint(joint);
            if (angular) body->SetOrientation(0.2f); else body->SetPosition({1.25f, 0});
            std::vector<double> samples;
            const int steps = int(3 / h);
            for (int i = 0; i < steps; ++i) {
                samples.push_back(angular ? joint->getAngle() : double(body->position.x) - 1);
                world.step(float(h));
                REQUIRE(std::abs(angular ? double(body->position.x) - 1 : joint->getAngle()) < 1e-5);
            }
            const auto measured = JointOscillation::Measure(samples, h);
            const auto oracle = JointOscillation::BackwardEuler(2, 0.1, h);
            INFO((angular ? "angular" : "linear") << " h " << h << ": measured " << measured.frequency
                 << " Hz zeta " << measured.dampingRatio << ", oracle " << oracle.frequency << " Hz zeta "
                 << oracle.dampingRatio);
            REQUIRE(measured.peaks >= 3);
            CHECK(measured.frequency == Catch::Approx(oracle.frequency).epsilon(0.005));
            CHECK(measured.dampingRatio == Catch::Approx(oracle.dampingRatio).epsilon(0.02));
            CHECK(measured.frequency == Catch::Approx(2).epsilon(h > 0.01 ? 0.02 : 0.002));
            if (h < 0.01) CHECK(measured.dampingRatio == Catch::Approx(0.1).margin(0.01));
        }
    }
}

TEST_CASE("Rigid weld removes relative motion in one iteration", "[weld]") {
    SimulationConfig config = FreeConfig();
    config.solverIterations = 1;
    World world(config);
    auto a = Box({-1, 0}, 1, 1), b = Box({1, 0.5f}, 1, 2);
    a->SetMass(2); b->SetMass(5);
    a->SetVelocity({3, -1}); b->SetVelocity({-2, 4});
    a->SetAngularVelocity(1.3f); b->SetAngularVelocity(-0.7f);
    world.addBody(a);
    world.addBody(b);
    world.addJoint(std::make_shared<WeldJoint>(a, b, Vector2{1, 0}, Vector2{-1, -0.5f}));
    const double px = double(a->mass) * a->velocity.x + double(b->mass) * b->velocity.x;
    const double py = double(a->mass) * a->velocity.y + double(b->mass) * b->velocity.y;
    world.step(0);
    CHECK(a->angularVelocity == Catch::Approx(b->angularVelocity).margin(1e-6));
    // Anchor velocities agree.
    const double w = a->angularVelocity;
    CHECK(a->velocity.x == Catch::Approx(b->velocity.x + w * 0.5).margin(1e-5));
    CHECK(a->velocity.y + w * 1 == Catch::Approx(b->velocity.y - w * 1).margin(1e-5));
    CHECK(double(a->mass) * a->velocity.x + double(b->mass) * b->velocity.x == Catch::Approx(px).margin(1e-5));
    CHECK(double(a->mass) * a->velocity.y + double(b->mass) * b->velocity.y == Catch::Approx(py).margin(1e-5));
}

TEST_CASE("Welded overhang resting on the ground holds and falls asleep", "[weld][sleeping]") {
    SimulationConfig config;
    config.enableSleeping = true;
    World world(config);
    world.addUniversalForce(std::make_unique<Gravity>(Vector2{0, -9.81f}));
    auto ground = std::make_shared<RigidBody>(Polygon::MakeBox(20, 1), Material{}, Vector2{0, -0.5f}, true);
    auto base = std::make_shared<RigidBody>(Polygon::MakeBox(2, 1), Material{}, Vector2{0, 0.5f});
    auto overhang = std::make_shared<RigidBody>(Polygon::MakeBox(1, 1), Material{}, Vector2{1.5f, 1.5f});
    world.addBody(ground);
    world.addBody(base);
    world.addBody(overhang);
    // The overhang touches the base only at its corner (1, 1); the weld
    // carries its whole weight and moment. Combined center x = 0.5 is over
    // the base's support [-1, 1].
    world.addJoint(std::make_shared<WeldJoint>(base, overhang, Vector2{1, 0.5f}, Vector2{-0.5f, -0.5f}));
    int asleepAt = -1;
    for (int i = 0; i < 600 && asleepAt < 0; ++i) {
        world.step(1.f / 60);
        if (!base->IsAwake() && !overhang->IsAwake()) asleepAt = i;
    }
    INFO("asleep after " << asleepAt << " steps; overhang at " << overhang->position.x << ", "
                         << overhang->position.y << " angle " << overhang->orientation);
    CHECK(asleepAt > 0);
    CHECK(overhang->position.y == Catch::Approx(1.5).margin(0.01));
    CHECK(overhang->position.x == Catch::Approx(1.5).margin(0.01));
    CHECK(std::abs(overhang->orientation) < 0.005);
}

TEST_CASE("Weld joint validates inputs before changing state", "[weld][validation]") {
    const float inf = std::numeric_limits<float>::infinity(), nan = std::numeric_limits<float>::quiet_NaN();
    auto a = Box({0, 0}, 1, 1), b = Box({1, 0}, 1, 1), s = Box({2, 0}, 1, 1, true), t = Box({3, 0}, 1, 1, true);
    REQUIRE_THROWS_AS(WeldJoint(a, a), std::invalid_argument);
    REQUIRE_THROWS_AS(WeldJoint(s, t), std::invalid_argument);
    REQUIRE_THROWS_AS(WeldJoint(a, nullptr), std::invalid_argument);
    REQUIRE_THROWS_AS(WeldJoint(a, b, {nan, 0}), std::invalid_argument);
    b->SetOrientation(0.5f);
    WeldJoint joint(a, b);
    REQUIRE(joint.getReferenceAngle() == Catch::Approx(0.5));
    REQUIRE(joint.getAngle() == 0);
    REQUIRE_FALSE(joint.getCollideConnected());
    REQUIRE_THROWS_AS(joint.setLinearSpring(-1, 0), std::invalid_argument);
    REQUIRE_THROWS_AS(joint.setLinearSpring(inf, 0), std::invalid_argument);
    REQUIRE_THROWS_AS(joint.setLinearSpring(1, -0.5f), std::invalid_argument);
    REQUIRE_THROWS_AS(joint.setAngularSpring(nan, 0), std::invalid_argument);
    REQUIRE_THROWS_AS(joint.setAngularSpring(1, inf), std::invalid_argument);
    REQUIRE(joint.getLinearFrequency() == 0);
    REQUIRE(joint.getAngularFrequency() == 0);
    joint.setLinearSpring(3, 0.5f);
    REQUIRE(joint.getLinearFrequency() == 3);
    REQUIRE(joint.getLinearDampingRatio() == 0.5f);
    a->velocity = {std::numeric_limits<float>::quiet_NaN(), 0};
    REQUIRE_THROWS_AS(joint.getAngle(), std::invalid_argument);
}

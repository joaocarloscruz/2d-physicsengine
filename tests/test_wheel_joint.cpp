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
RigidBodyPtr Wheel(Vector2 p, float radius, float mass) {
    auto body = std::make_shared<RigidBody>(Circle(radius), Material{1, 0, 1.0f, 0.9f}, p);
    body->SetMass(mass);
    return body;
}
RigidBodyPtr Mount() {
    auto body = std::make_shared<RigidBody>(Circle(0.1f), Material{}, Vector2{}, true);
    body->SetCollisionMaskBits(0);
    return body;
}
}

TEST_CASE("Wheel suspension oscillates at the requested frequency", "[wheel][oracle]") {
    for (double h : {1.0 / 60, 1.0 / 1000}) {
        World world(FreeConfig());
        auto mount = Mount(), wheel = Wheel({0, -0.1f}, 0.3f, 1.5f);
        wheel->SetCollisionMaskBits(0);
        world.addBody(mount);
        world.addBody(wheel);
        auto joint = std::make_shared<WheelJoint>(mount, wheel, Vector2{0, 1});
        joint->setSpring(3, 0.2f);
        world.addJoint(joint);
        wheel->SetAngularVelocity(5); // spins freely meanwhile
        std::vector<double> samples;
        for (int i = 0; i < int(2 / h); ++i) {
            samples.push_back(-joint->getTranslation()); // starts at +0.1
            world.step(float(h));
            REQUIRE(std::abs(joint->getTransverseError()) < 1e-6);
            REQUIRE(wheel->angularVelocity == 5);
        }
        const auto measured = JointOscillation::Measure(samples, h);
        const auto oracle = JointOscillation::BackwardEuler(3, 0.2, h);
        INFO("h " << h << ": measured " << measured.frequency << " Hz zeta " << measured.dampingRatio
                  << ", oracle " << oracle.frequency << " Hz zeta " << oracle.dampingRatio);
        REQUIRE(measured.peaks >= 3);
        CHECK(measured.frequency == Catch::Approx(oracle.frequency).epsilon(0.005));
        CHECK(measured.dampingRatio == Catch::Approx(oracle.dampingRatio).epsilon(0.02));
        CHECK(measured.frequency == Catch::Approx(3).epsilon(h > 0.01 ? 0.05 : 0.002));
        if (h < 0.01) CHECK(measured.dampingRatio == Catch::Approx(0.2).margin(0.01));
    }
}

TEST_CASE("Wheel motor reaches its speed under the torque cap", "[wheel][motor]") {
    World world(FreeConfig());
    auto mount = Mount(), wheel = Wheel({0, 0}, 0.5f, 2);
    wheel->SetCollisionMaskBits(0);
    world.addBody(mount);
    world.addBody(wheel);
    auto joint = std::make_shared<WheelJoint>(mount, wheel);
    joint->setSpring(4, 0.7f);
    joint->setMotor(true, 10, 2);
    world.addJoint(joint);
    const double inertia = wheel->inertia; // 0.5 m r^2 = 0.25
    REQUIRE(inertia == Catch::Approx(0.25));
    const double h = 1.0 / 60, alpha = 2 / inertia; // 8 rad/s^2: 1.25 s to 10 rad/s
    int reached = -1;
    for (int i = 1; i <= 120; ++i) {
        world.step(float(h));
        REQUIRE(std::abs(joint->getMotorTorque()) <= 2 * (1 + 1e-6));
        const double expected = std::min(alpha * h * i, 10.0);
        REQUIRE(joint->getAngularSpeed() == Catch::Approx(expected).epsilon(1e-5));
        if (reached < 0 && joint->getAngularSpeed() >= 10 - 1e-4) reached = i;
    }
    INFO("reached 10 rad/s after " << reached * h << " s (cap allows 1.25 s)");
    CHECK(reached == 75);
    // Holding speed needs no torque on a free wheel.
    CHECK(joint->getMotorTorque() == Catch::Approx(0).margin(1e-4));
}

TEST_CASE("Wheel translation limits stop the suspension", "[wheel][limits]") {
    World world(FreeConfig());
    world.addUniversalForce(std::make_unique<Gravity>(Vector2{0, -9.81f}));
    auto mount = Mount(), wheel = Wheel({0, 0}, 0.3f, 1);
    wheel->SetCollisionMaskBits(0);
    world.addBody(mount);
    world.addBody(wheel);
    auto joint = std::make_shared<WheelJoint>(mount, wheel);
    joint->setLimits(true, -0.2f, 0.1f);
    world.addJoint(joint);
    double lowest = 0;
    for (int i = 0; i < 240; ++i) {
        world.step(1.f / 60);
        lowest = std::min(lowest, joint->getTranslation());
    }
    INFO("lowest translation " << lowest);
    CHECK(lowest > -0.2 - 0.006);
    CHECK(joint->getTranslation() == Catch::Approx(-0.2).margin(0.006));
    CHECK(std::abs(wheel->velocity.y) < 1e-3);
}

TEST_CASE("Two-wheel car settles, accelerates, brakes and sleeps", "[wheel][car]") {
    SimulationConfig config;
    config.enableSleeping = true;
    World world(config);
    world.addUniversalForce(std::make_unique<Gravity>(Vector2{0, -9.81f}));
    auto ground = std::make_shared<RigidBody>(Polygon::MakeBox(400, 1), Material{1, 0, 1.0f, 0.9f},
                                              Vector2{150, -0.5f}, true);
    auto chassis = std::make_shared<RigidBody>(Polygon::MakeBox(2, 0.5f), Material{}, Vector2{0, 1});
    chassis->SetMass(20);
    const float radius = 0.4f;
    auto rear = Wheel({-0.7f, radius}, radius, 2), front = Wheel({0.7f, radius}, radius, 2);
    for (const auto& body : {ground, chassis, rear, front}) world.addBody(body);
    std::vector<std::shared_ptr<WheelJoint>> axles;
    for (const auto& wheel : {rear, front}) {
        const Vector2 local{wheel->position.x, radius - 1};
        auto axle = std::make_shared<WheelJoint>(chassis, wheel, Vector2{0, 1}, local, Vector2{});
        axle->setSpring(4, 0.7f);
        world.addJoint(axle);
        axles.push_back(axle);
    }
    // Settle with the motor off.
    for (int i = 0; i < 180; ++i) world.step(1.f / 60);
    const double settledY = chassis->position.y;
    // Static sag: each spring carries half the chassis weight with stiffness
    // k = (2 pi f)^2 * axial effective mass, 1/(1/M + 1/m + 0.7^2/I).
    const double axialMass = 1 / (1 / 20.0 + 1 / 2.0 + 0.49 / double(chassis->inertia));
    const double stiffness = std::pow(2 * JointOscillation::Pi * 4, 2) * axialMass;
    const double expectedSag = 10 * 9.81 / stiffness;
    INFO("settled chassis y " << settledY << ", sag " << axles[0]->getTranslation() << " / "
                              << axles[1]->getTranslation() << " m, static model " << expectedSag);
    CHECK(std::abs(chassis->velocity.y) < 0.01);
    CHECK(std::abs(chassis->orientation) < 0.01);
    // Compression is positive: A's anchor drops toward the wheel center.
    CHECK(axles[0]->getTranslation() == Catch::Approx(expectedSag).epsilon(0.03));
    CHECK(axles[1]->getTranslation() == Catch::Approx(expectedSag).epsilon(0.03));
    CHECK(axles[0]->getSpringForce() == Catch::Approx(-10 * 9.81).epsilon(0.03));

    // Rear-wheel drive, clockwise spin drives +x: target ground speed w*r = 6 m/s.
    const float speed = -15, torque = 40;
    axles[0]->setMotor(true, speed, torque);
    double previous = chassis->velocity.x, worstTilt = 0;
    for (int i = 0; i < 300; ++i) {
        world.step(1.f / 60);
        worstTilt = std::max(worstTilt, double(std::abs(chassis->orientation)));
        REQUIRE(std::abs(axles[0]->getMotorTorque()) <= torque * (1 + 1e-6));
        REQUIRE(std::abs(axles[0]->getTransverseError()) < 0.01);
        if (i < 30) CHECK(chassis->velocity.x >= previous - 1e-3); // accelerating from rest
        previous = chassis->velocity.x;
    }
    INFO("after 5 s of drive: vx " << chassis->velocity.x << " m/s (w*r = 6), worst tilt " << worstTilt
                                   << " rad, rear wheel speed " << axles[0]->getAngularSpeed());
    CHECK(chassis->velocity.x == Catch::Approx(-speed * radius).epsilon(0.05));
    CHECK(worstTilt < 0.2);
    CHECK(rear->position.y == Catch::Approx(radius).margin(0.05));
    CHECK(front->position.y == Catch::Approx(radius).margin(0.05));
    CHECK(chassis->position.y == Catch::Approx(settledY).margin(0.05));

    // Brake: zero-speed motor on both wheels. A zero-speed motor lets the car sleep.
    for (const auto& axle : axles) axle->setMotor(true, 0, 200);
    int asleepAt = -1;
    for (int i = 0; i < 900 && asleepAt < 0; ++i) {
        world.step(1.f / 60);
        if (!chassis->IsAwake()) asleepAt = i;
    }
    INFO("asleep " << asleepAt << " steps after braking at x " << chassis->position.x);
    CHECK(asleepAt > 0);
    // A driving motor wakes it and keeps it awake.
    axles[0]->setMotor(true, speed, torque);
    CHECK(chassis->IsAwake());
}

TEST_CASE("Wheel joint validates inputs before changing state", "[wheel][validation]") {
    const float inf = std::numeric_limits<float>::infinity(), nan = std::numeric_limits<float>::quiet_NaN();
    auto mount = Mount(), wheel = Wheel({0, 0}, 0.3f, 1);
    REQUIRE_THROWS_AS(WheelJoint(mount, wheel, {0, 0}), std::invalid_argument);
    REQUIRE_THROWS_AS(WheelJoint(mount, wheel, {inf, 1}), std::invalid_argument);
    REQUIRE_THROWS_AS(WheelJoint(mount, wheel, {0, 1}, {nan, 0}), std::invalid_argument);
    REQUIRE_THROWS_AS(WheelJoint(wheel, wheel), std::invalid_argument);
    WheelJoint joint(mount, wheel, {0, 2});
    REQUIRE(joint.getLocalAxis() == Vector2{0, 1});
    REQUIRE_THROWS_AS(joint.setSpring(-1, 0), std::invalid_argument);
    REQUIRE_THROWS_AS(joint.setSpring(1, nan), std::invalid_argument);
    REQUIRE_THROWS_AS(joint.setMotor(true, nan, 1), std::invalid_argument);
    REQUIRE_THROWS_AS(joint.setMotor(true, 1, -1), std::invalid_argument);
    REQUIRE_THROWS_AS(joint.setLimits(true, 1, 0), std::invalid_argument);
    REQUIRE_THROWS_AS(joint.setLimits(true, -inf, 0), std::invalid_argument);
    REQUIRE(joint.getSpringFrequency() == 0);
    REQUIRE_FALSE(joint.isMotorEnabled());
    REQUIRE_FALSE(joint.areLimitsEnabled());
    joint.setMotor(true, 3, 4);
    REQUIRE(joint.getMotorSpeed() == 3);
    REQUIRE(joint.getMaxMotorTorque() == 4);
}

#include "physics/physics.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <memory>

// A car built from a chassis box and two wheels on WheelJoints (see
// docs/joints-mouse-weld-wheel.md). Each joint has a suspension spring and
// travel limits; the rear wheel has a motor. The car drives over three bumps
// and up a ramp onto a plateau, then the motor becomes a brake. The program
// exits nonzero if the car does not reach the target, the suspension leaves
// its limits, the chassis touches the ground, or the car does not stop.
using namespace PhysicsEngine;

namespace {
constexpr float kLowerLimit = -0.25f, kUpperLimit = 0.25f; // suspension travel in metres
constexpr float kTargetX = 32.0f;                          // on the plateau

// Counts solid contacts of one body, so the chassis can prove it never touched.
class ContactCounter : public ICollisionListener {
public:
    explicit ContactCounter(std::uint64_t id) : id(id) {}
    void onCollisionBegin(const CollisionEvent& event) override {
        if (!event.sensor && (event.bodyAId == id || event.bodyBId == id)) ++begins;
    }
    int begins = 0;
private:
    std::uint64_t id;
};

std::shared_ptr<RigidBody> makeWheel(float x, std::shared_ptr<RigidBody> chassis, World& world,
                                     std::shared_ptr<WheelJoint>& jointOut) {
    // Density 5 kg/m^2 gives a 1.9 kg wheel under a 20 kg chassis. High friction grips the ground.
    auto wheel = std::make_shared<RigidBody>(Circle(0.35f), Material{5.0f, 0.0f, 1.0f, 0.9f}, Vector2{x, 0.35f});
    world.addBody(wheel);
    // Axis {0,1} is "up" in the chassis frame; the anchors start together at the wheel center.
    jointOut = std::make_shared<WheelJoint>(chassis, wheel, Vector2{0, 1}, Vector2{x - chassis->position.x, -0.25f}, Vector2{});
    jointOut->setSpring(4.0f, 0.7f);
    jointOut->setLimits(true, kLowerLimit, kUpperLimit);
    world.addJoint(jointOut);
    return wheel;
}
}

int main() {
    World world;
    world.addUniversalForce(std::make_unique<Gravity>(Vector2{0, -9.81f}));

    const Material road{1.0f, 0.0f, 0.9f, 0.9f};
    world.addBody(std::make_shared<RigidBody>(Polygon::MakeBox(60, 1), road, Vector2{20, -0.5f}, true)); // floor, top y = 0
    for (const float x : {7.0f, 10.0f, 13.0f}) // bumps: circles 0.12 m high, buried below the road surface
        world.addBody(std::make_shared<RigidBody>(Circle(0.2f), road, Vector2{x, -0.08f}, true));
    const Polygon ramp({{0, 0}, {8, 0}, {8, 1.6f}});               // 8 m long, 1.6 m rise
    world.addBody(std::make_shared<RigidBody>(ramp.Recentered(), road, Vector2{18, 0} + ramp.GetCentroid(), true));
    world.addBody(std::make_shared<RigidBody>(Polygon::MakeBox(30, 1.6f), road, Vector2{41, 0.8f}, true)); // plateau from x = 26

    auto chassis = std::make_shared<RigidBody>(Polygon::MakeBox(2.0f, 0.5f), Material{20.0f, 0.0f, 0.5f, 0.4f}, Vector2{0, 0.6f});
    world.addBody(chassis);
    std::shared_ptr<WheelJoint> rearJoint, frontJoint;
    auto rear = makeWheel(-0.8f, chassis, world, rearJoint);
    auto front = makeWheel(0.8f, chassis, world, frontJoint);

    ContactCounter chassisContacts(chassis->GetId());
    world.addCollisionListener(&chassisContacts);

    double lowest = 1e9, highest = -1e9, worstTilt = 0, maxSpeed = 0;
    int brakeStep = -1, stoppedStep = -1;
    float brakeX = 0;
    for (int frame = 0; frame < 2400; ++frame) {
        if (frame == 30) rearJoint->setMotor(true, -12.0f, 40.0f); // clockwise, so the car moves +x
        if (brakeStep < 0 && chassis->position.x >= kTargetX) {
            // A zero-speed motor is a brake: it resists any wheel rotation.
            rearJoint->setMotor(true, 0.0f, 40.0f);
            frontJoint->setMotor(true, 0.0f, 40.0f);
            brakeStep = frame;
            brakeX = chassis->position.x;
        }
        world.step();
        for (const auto& joint : {rearJoint, frontJoint}) {
            lowest = std::min(lowest, joint->getTranslation());
            highest = std::max(highest, joint->getTranslation());
        }
        worstTilt = std::max(worstTilt, double(std::fabs(chassis->orientation)));
        maxSpeed = std::max(maxSpeed, double(chassis->velocity.x));
        if (brakeStep >= 0 && stoppedStep < 0 && std::hypot(chassis->velocity.x, chassis->velocity.y) < 0.02f
            && std::fabs(rear->angularVelocity) < 0.1f && std::fabs(front->angularVelocity) < 0.1f)
            stoppedStep = frame;
    }
    world.removeCollisionListener(&chassisContacts);

    std::printf("braking started at step %d, x=%.2f (target %.1f); top speed %.2f m/s\n", brakeStep, brakeX, kTargetX, maxSpeed);
    std::printf("stopped at step %d, x=%.2f y=%.3f (plateau top 1.6, wheel radius 0.35)\n", stoppedStep, chassis->position.x, chassis->position.y);
    std::printf("suspension translation %.3f to %.3f (limits %.2f to %.2f), worst chassis tilt %.3f rad\n",
        lowest, highest, double(kLowerLimit), double(kUpperLimit), worstTilt);
    std::printf("chassis contact begin events: %d\n", chassisContacts.begins);

    const double slack = 0.01; // limits are projected, so allow a hair of overshoot
    const bool ok = brakeStep > 0 && brakeX >= kTargetX
        && lowest >= kLowerLimit - slack && highest <= kUpperLimit + slack
        && chassisContacts.begins == 0 && worstTilt < 0.5
        && stoppedStep > brakeStep && stoppedStep < brakeStep + 240
        && chassis->position.x - brakeX < 3.0f && std::fabs(rear->position.y - 1.95f) < 0.02f;
    std::printf("%s\n", ok ? "vehicle: ok" : "vehicle: FAIL");
    return ok ? 0 : 1;
}

#include "physics/physics.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>
#include <vector>

// A ragdoll of capsules joined by revolute joints with angle limits, thrown
// down a staircase. Joints keep collideConnected off (the default), so joined
// limbs, which overlap at their shared anchor, do not push each other apart.
// Sleeping is enabled so the ragdoll stops simulating once it settles. The
// program exits nonzero if a limit is broken by more than a tolerance, a joint
// separates by more than a bound, the ragdoll explodes, or it never sleeps.
using namespace PhysicsEngine;

namespace {
constexpr float kHalfPi = 1.57079632679f;
constexpr float kLimitTolerance = 0.1f;      // radians a limit may be exceeded under impact
constexpr float kSeparationBound = 0.05f;    // metres a joint's two anchors may drift apart

struct Rig {
    World& world;
    std::vector<std::shared_ptr<RevoluteJoint>> joints;
    std::vector<std::shared_ptr<RigidBody>> parts;

    // A vertical capsule: the capsule axis is local x, so rotate it by 90 degrees.
    std::shared_ptr<RigidBody> limb(float halfLength, float radius, Vector2 center) {
        auto body = std::make_shared<RigidBody>(Capsule(halfLength, radius), Material{1.0f, 0.0f, 0.6f, 0.5f}, center);
        body->SetOrientation(kHalfPi);
        add(body);
        return body;
    }
    void add(const std::shared_ptr<RigidBody>& body) { world.addBody(body); parts.push_back(body); }

    static Vector2 toLocal(const RigidBody& body, Vector2 world) {
        const Vector2 d = world - body.position;
        const float c = std::cos(body.orientation), s = std::sin(body.orientation);
        return {c * d.x + s * d.y, -s * d.x + c * d.y};
    }
    // Pins `child` to `parent` at a world point. Limits are relative angles of
    // the child to the parent, measured from this pose.
    void pin(const std::shared_ptr<RigidBody>& parent, const std::shared_ptr<RigidBody>& child,
             Vector2 anchor, float lower, float upper) {
        auto joint = std::make_shared<RevoluteJoint>(parent, child, toLocal(*parent, anchor), toLocal(*child, anchor));
        joint->setLimits(true, lower, upper);
        world.addJoint(joint);
        joints.push_back(joint);
    }
};
}

int main() {
    World world;
    SimulationConfig config = world.getSimulationConfig();
    config.enableSleeping = true;
    world.setSimulationConfig(config);
    world.addUniversalForce(std::make_unique<Gravity>(Vector2{0, -9.81f}));

    // Eight steps going down to the right, 0.4 m high and 0.8 m wide, then a floor.
    const Material stone{1.0f, 0.0f, 0.6f, 0.5f};
    for (int i = 0; i < 8; ++i) {
        const float height = 0.4f * (8 - i);
        world.addBody(std::make_shared<RigidBody>(Polygon::MakeBox(0.8f, height), stone, Vector2{0.4f + 0.8f * i, height / 2}, true));
    }
    world.addBody(std::make_shared<RigidBody>(Polygon::MakeBox(20, 1), stone, Vector2{16.4f, -0.5f}, true));

    // Standing pose above the top step, about 2.4 m tall.
    const float top = 3.2f + 0.05f; // feet start 5 cm above the top step
    Rig rig{world, {}, {}};
    auto torso = rig.limb(0.3f, 0.15f, {0.4f, top + 1.3f});
    auto head = std::make_shared<RigidBody>(Circle(0.17f), stone, Vector2{0.4f, top + 2.05f});
    rig.add(head);
    const float shoulderY = top + 1.65f, hipY = top + 1.05f, kneeY = top + 0.55f, neckY = top + 1.85f;
    rig.pin(torso, head, {0.4f, neckY}, -0.6f, 0.6f);     // neck
    for (const float side : {-1.0f, 1.0f}) {
        auto arm = rig.limb(0.2f, 0.06f, {0.4f + 0.22f * side, top + 1.45f});
        auto thigh = rig.limb(0.25f, 0.08f, {0.4f + 0.12f * side, top + 0.8f});
        auto shin = rig.limb(0.25f, 0.08f, {0.4f + 0.12f * side, top + 0.3f});
        rig.pin(torso, arm, {0.4f + 0.22f * side, shoulderY}, -2.5f, 2.5f);   // shoulder
        rig.pin(torso, thigh, {0.4f + 0.12f * side, hipY}, -1.2f, 1.2f);      // hip
        rig.pin(thigh, shin, {0.4f + 0.12f * side, kneeY}, -2.2f, 0.0f);      // knee bends one way only
    }

    // Throw it: a push toward the stairs and a spin.
    for (const auto& part : rig.parts) {
        part->SetVelocity({4.5f, 0});
        part->SetAngularVelocity(-2.0f);
    }

    double worstLimit = 0, worstSeparation = 0, fastest = 0;
    int asleepAt = -1;
    bool finite = true;
    for (int frame = 0; frame < 1500; ++frame) {
        world.step();
        for (const auto& joint : rig.joints) {
            const double angle = joint->getAngle();
            worstLimit = std::max({worstLimit, angle - joint->getUpperLimit(), joint->getLowerLimit() - angle});
            const Vector2 gap = joint->getAnchorA() - joint->getAnchorB();
            worstSeparation = std::max(worstSeparation, std::hypot(double(gap.x), double(gap.y)));
        }
        for (const auto& part : rig.parts) {
            fastest = std::max(fastest, double(std::hypot(part->velocity.x, part->velocity.y)));
            finite = finite && std::isfinite(part->position.x) && std::isfinite(part->position.y);
        }
        const bool allAsleep = std::none_of(rig.parts.begin(), rig.parts.end(), [](const auto& p) { return p->IsAwake(); });
        if (allAsleep && asleepAt < 0) asleepAt = frame;
    }

    std::printf("torso: start (0.40, %.2f), rest (%.2f, %.2f); head rest (%.2f, %.2f)\n",
        top + 1.3f, torso->position.x, torso->position.y, head->position.x, head->position.y);
    std::printf("worst limit violation %.4f rad (tolerance %.2f), worst joint separation %.4f m (bound %.2f)\n",
        worstLimit, double(kLimitTolerance), worstSeparation, double(kSeparationBound));
    std::printf("fastest part %.2f m/s, whole ragdoll asleep at step %d\n", fastest, asleepAt);

    // Resting on the floor past the last step, not inside it and not flying off.
    const bool inBounds = torso->position.x > 6.4f && torso->position.x < 19.0f && torso->position.y > 0.0f && torso->position.y < 0.6f;
    const bool ok = finite && inBounds && worstLimit < kLimitTolerance && worstSeparation < kSeparationBound
        && fastest < 40.0 && asleepAt >= 0 && asleepAt < 1400;
    std::printf("%s\n", ok ? "ragdoll: ok" : "ragdoll: FAIL");
    return ok ? 0 : 1;
}

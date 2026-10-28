#include "scenes.h"
#include <cmath>
#include <cstdint>
#include <memory>
#include <vector>

namespace PhysicsEngine {
namespace Visualization {

namespace {

constexpr float kPi = 3.14159265358979f;
constexpr float kFloorTop = -5.0f;
constexpr float kGravity = -9.81f;
// Chain parts live in category 2 and ignore it, so neighbours linked by a joint
// do not fight their own contact while still colliding with everything else.
constexpr std::uint32_t kChainCategory = 2u;
constexpr std::uint32_t kNoChainMask = ~kChainCategory;
// Slider stroke in the Machines scene. It lives at namespace scope because the
// slider's controller lambda reads it, and MSVC rejects uncaptured constexpr
// float locals inside lambdas.
constexpr float kSliderTravel = 6.0f;

const Material kStone{1.0f, 0.1f, 0.6f, 0.4f};

RigidBodyPtr add(World& world, const Shape& shape, const Material& material,
                 Vector2 position, bool isStatic = false, float angle = 0.0f) {
    auto body = std::make_shared<RigidBody>(shape, material, position, isStatic);
    if (angle != 0.0f) body->SetOrientation(angle);
    world.addBody(body);
    return body;
}

RigidBodyPtr addBox(World& world, float width, float height, Vector2 position,
                    const Material& material = kStone, bool isStatic = false, float angle = 0.0f) {
    return add(world, Polygon::MakeBox(width, height), material, position, isStatic, angle);
}

RigidBodyPtr addBall(World& world, float radius, Vector2 position, const Material& material = kStone) {
    return add(world, Circle(radius), material, position);
}

RigidBodyPtr addCapsule(World& world, float halfLength, float radius, Vector2 position,
                        const Material& material = kStone, bool isStatic = false, float angle = 0.0f) {
    return add(world, Capsule(halfLength, radius), material, position, isStatic, angle);
}

// A static body that collides with nothing: a pivot or rail anchor for a joint.
RigidBodyPtr addAnchor(World& world, Vector2 position, float size = 0.3f) {
    auto anchor = addBox(world, size, size, position, kStone, true);
    anchor->SetCollisionMaskBits(0u);
    return anchor;
}

void addArena(World& world) {
    const Material ground{1.0f, 0.1f, 0.7f, 0.5f};
    addBox(world, 20.0f, 2.0f, Vector2(0.0f, -6.0f), ground, true);
    addBox(world, 2.0f, 14.0f, Vector2(-11.0f, 0.0f), ground, true);
    addBox(world, 2.0f, 14.0f, Vector2(11.0f, 0.0f), ground, true);
    world.addUniversalForce(std::make_unique<Gravity>(Vector2(0.0f, kGravity)));
}

SimulationConfig stackedConfig() {
    SimulationConfig config;
    config.solverIterations = 20;
    return config;
}

// 1. Playground: the original sandbox.
SceneRuntime buildPlayground(World& world) {
    addArena(world);
    addBox(world, 1.0f, 1.0f, Vector2(-3.0f, -3.0f));
    addBox(world, 1.5f, 1.0f, Vector2(2.0f, -3.0f));
    addBall(world, 0.5f, Vector2(0.0f, 0.0f));
    return {};
}

// 2. Tower: twelve boxes, each nudged a few centimetres off the last.
SceneRuntime buildTower(World& world) {
    addArena(world);
    constexpr int count = 12;
    constexpr float height = 0.8f;
    for (int i = 0; i < count; ++i) {
        const float jitter = 0.03f * std::sin(1.7f * static_cast<float>(i));
        addBox(world, 1.2f, height,
               Vector2(jitter, kFloorTop + height * (static_cast<float>(i) + 0.5f) + 0.002f * i));
    }
    return {};
}

// 3. Pyramid: eight boxes at the base, one at the top.
SceneRuntime buildPyramid(World& world) {
    addArena(world);
    constexpr int base = 8;
    constexpr float width = 1.1f;
    constexpr float height = 0.7f;
    for (int row = 0; row < base; ++row) {
        const int columns = base - row;
        for (int column = 0; column < columns; ++column) {
            const float x = (static_cast<float>(column) - 0.5f * (columns - 1)) * (width + 0.01f);
            const float y = kFloorTop + height * (static_cast<float>(row) + 0.5f) + 0.002f * row;
            addBox(world, width, height, Vector2(x, y));
        }
    }
    return {};
}

// 4. Dominoes: twenty thin boxes and a ball fired at the first.
SceneRuntime buildDominoes(World& world) {
    addArena(world);
    constexpr int count = 20;
    constexpr float width = 0.22f;
    constexpr float height = 1.6f;
    constexpr float spacing = 0.82f;
    for (int i = 0; i < count; ++i) {
        const float x = (static_cast<float>(i) - 0.5f * (count - 1)) * spacing;
        addBox(world, width, height, Vector2(x, kFloorTop + 0.5f * height + 0.001f));
    }
    auto ball = addBall(world, 0.5f, Vector2(-9.4f, kFloorTop + 0.51f), Material{4.0f, 0.1f, 0.6f, 0.4f});
    ball->SetVelocity(Vector2(6.0f, 0.0f));
    return {};
}

// 5. Chain and pendulum: a hanging revolute chain beside a distance-joint
// pendulum that swings into a stack of boxes.
SceneRuntime buildChainAndPendulum(World& world) {
    addArena(world);

    constexpr float halfLength = 0.4f;
    constexpr float radius = 0.2f;
    constexpr float tip = halfLength + radius;
    constexpr int links = 5;
    const Vector2 pivot(-8.5f, 3.0f);
    auto previous = addAnchor(world, pivot);
    Vector2 previousTip(0.0f, 0.0f);
    for (int i = 0; i < links; ++i) {
        auto link = addCapsule(world, halfLength, radius,
                               Vector2(pivot.x + tip + 2.0f * tip * i, pivot.y),
                               Material{1.0f, 0.1f, 0.5f, 0.4f});
        link->SetCollisionCategoryBits(kChainCategory);
        link->SetCollisionMaskBits(kNoChainMask);
        world.addJoint(std::make_shared<RevoluteJoint>(previous, link, previousTip, Vector2(-tip, 0.0f)));
        previous = link;
        previousTip = Vector2(tip, 0.0f);
    }
    auto weight = addBall(world, 0.45f, Vector2(pivot.x + 2.0f * tip * links + 0.45f, pivot.y),
                          Material{3.0f, 0.1f, 0.5f, 0.4f});
    weight->SetCollisionCategoryBits(kChainCategory);
    weight->SetCollisionMaskBits(kNoChainMask);
    world.addJoint(std::make_shared<RevoluteJoint>(previous, weight, previousTip, Vector2(-0.45f, 0.0f)));

    // A stack of ten boxes for the pendulum to hit.
    for (int row = 0; row < 5; ++row) {
        for (int column = 0; column < 2; ++column) {
            addBox(world, 0.98f, 0.98f,
                   Vector2(0.5f + static_cast<float>(column), kFloorTop + 0.5f + 0.99f * row + 0.002f));
        }
    }

    const Vector2 hinge(3.0f, 3.0f);
    constexpr float length = 6.0f;
    constexpr float angle = 70.0f * kPi / 180.0f;
    auto support = addAnchor(world, hinge);
    auto bob = addBall(world, 0.6f,
                       hinge + Vector2(length * std::sin(angle), -length * std::cos(angle)),
                       Material{6.0f, 0.1f, 0.5f, 0.4f});
    world.addJoint(std::make_shared<DistanceJoint>(support, bob, length));
    return {};
}

// 6. Capsules and ramps: capsules and balls slide down static polygon and
// capsule ramps in a zig-zag.
SceneRuntime buildRamps(World& world) {
    addArena(world);
    const Material slick{1.0f, 0.15f, 0.3f, 0.2f};
    constexpr float steep = 0.4f;
    constexpr float shallow = 0.35f;
    const Vector2 firstRamp(-4.5f, 2.0f);
    const Vector2 capsuleRamp(6.4f, 3.6f);

    addBox(world, 8.0f, 0.4f, firstRamp, slick, true, -steep);
    addBox(world, 8.0f, 0.4f, Vector2(3.0f, -1.5f), slick, true, steep);
    addCapsule(world, 2.5f, 0.3f, capsuleRamp, slick, true, shallow);
    // A back stop at the left end of the first ramp.
    addBox(world, 0.3f, 1.2f, Vector2(-8.4f, 3.5f), slick, true);

    // A point `along` a ramp's axis and `lift` above its centre line.
    auto onRamp = [](Vector2 center, float angle, float along, float lift) {
        return center + Vector2(std::cos(angle), std::sin(angle)) * along
                      + Vector2(-std::sin(angle), std::cos(angle)) * lift;
    };

    addCapsule(world, 0.5f, 0.2f, onRamp(firstRamp, -steep, -3.2f, 0.42f), slick, false, -steep);
    addBall(world, 0.4f, onRamp(firstRamp, -steep, -1.9f, 0.62f));
    addCapsule(world, 0.5f, 0.2f, onRamp(firstRamp, -steep, -0.6f, 0.42f), slick, false, -steep);
    addBall(world, 0.3f, onRamp(firstRamp, -steep, 0.8f, 0.52f));

    addCapsule(world, 0.45f, 0.2f, onRamp(capsuleRamp, shallow, 1.4f, 0.52f), slick, false, shallow);
    addBall(world, 0.35f, onRamp(capsuleRamp, shallow, 0.0f, 0.67f));
    addCapsule(world, 0.45f, 0.2f, onRamp(capsuleRamp, shallow, -1.3f, 0.52f), slick, false, shallow);
    return {};
}

// 7. Seesaw / mass ratio: a heavy ball drops on one end of a plank on a hinge
// and throws a light box from the other end (a mass ratio of roughly 25 to 1).
SceneRuntime buildSeesaw(World& world) {
    addArena(world);

    constexpr float halfPlank = 6.0f;
    constexpr float tilt = -0.3f;
    const Vector2 hinge(0.0f, -3.0f);
    auto support = addAnchor(world, hinge);
    // A decorative post under the hinge that collides with nothing.
    auto post = addBox(world, 0.3f, 2.0f, hinge + Vector2(0.0f, -1.0f), kStone, true);
    post->SetCollisionMaskBits(0u);

    auto plank = addBox(world, 2.0f * halfPlank, 0.3f, hinge, Material{1.0f, 0.1f, 0.7f, 0.5f}, false, tilt);
    auto joint = std::make_shared<RevoluteJoint>(support, plank);
    joint->setLimits(true, 0.0f, 0.6f);
    world.addJoint(joint);

    const Vector2 along(std::cos(tilt), std::sin(tilt));
    const Vector2 up(-std::sin(tilt), std::cos(tilt));
    const Material light{0.8f, 0.1f, 0.7f, 0.5f};
    addBox(world, 0.7f, 0.7f, hinge + along * (halfPlank - 0.7f) + up * 0.51f, light, false, tilt);
    addBox(world, 0.5f, 0.5f, hinge + along * (halfPlank - 1.6f) + up * 0.41f, light, false, tilt);

    addBall(world, 0.6f, Vector2(-5.0f, 3.5f), Material{9.0f, 0.1f, 0.7f, 0.5f});
    return {};
}

// 8. Machines: a motorized paddle wheel, a driven roller and a prismatic slider
// that shuttles back and forth pushing a stack.
SceneRuntime buildMachines(World& world) {
    addArena(world);

    // Paddle wheel: a beam on a motorized revolute joint sweeping a pile.
    const Vector2 hub(-6.5f, -1.5f);
    auto hubBody = addAnchor(world, hub, 0.4f);
    auto beam = addBox(world, 6.0f, 0.3f, hub, Material{2.0f, 0.1f, 0.5f, 0.4f});
    auto paddle = std::make_shared<RevoluteJoint>(hubBody, beam);
    paddle->setMotor(true, 1.6f, 5000.0f);
    world.addJoint(paddle);
    for (int i = 0; i < 3; ++i) {
        addBox(world, 0.9f, 0.9f, Vector2(-8.5f + 1.1f * i, kFloorTop + 0.46f));
    }
    addBox(world, 0.9f, 0.9f, Vector2(-7.9f, kFloorTop + 1.4f));

    // Roller: a high-friction wheel on a motor; balls dropped on it get carried off.
    const Vector2 axle(-1.0f, -3.4f);
    auto axleBody = addAnchor(world, axle, 0.3f);
    auto wheel = addBall(world, 1.55f, axle, Material{1.0f, 0.1f, 1.2f, 1.0f});
    auto roller = std::make_shared<RevoluteJoint>(axleBody, wheel);
    roller->setMotor(true, -4.0f, 8000.0f);
    world.addJoint(roller);
    for (int i = 0; i < 4; ++i) {
        addBall(world, 0.35f, Vector2(-1.6f + 0.55f * i, 1.0f + 1.0f * i), Material{1.0f, 0.2f, 0.6f, 0.4f});
    }

    // Slider: a carriage on a prismatic joint, driven along a rail.
    constexpr float railY = kFloorTop + 0.95f;
    const Vector2 start(9.0f, railY);
    auto rail = addBox(world, kSliderTravel + 1.0f, 0.1f, Vector2(start.x - 0.5f * kSliderTravel, railY), kStone, true);
    rail->SetCollisionMaskBits(0u);
    auto railAnchor = addAnchor(world, start, 0.1f);
    auto carriage = addBox(world, 1.0f, 1.8f, start, Material{5.0f, 0.1f, 0.4f, 0.3f});
    auto slider = std::make_shared<PrismaticJoint>(railAnchor, carriage, Vector2(-1.0f, 0.0f));
    slider->setLimits(true, 0.0f, kSliderTravel);
    slider->setMotor(true, 1.5f, 3000.0f);
    world.addJoint(slider);
    for (int row = 0; row < 3; ++row) {
        addBox(world, 0.9f, 0.9f, Vector2(5.0f, kFloorTop + 0.46f + 0.92f * row));
    }
    addBox(world, 0.9f, 0.9f, Vector2(4.0f, kFloorTop + 0.46f));

    SceneRuntime runtime;
    runtime.beforeStep = [slider](World&, float) {
        const double position = slider->getTranslation();
        const float speed = slider->getMotorSpeed();
        if (position >= kSliderTravel - 0.05 && speed > 0.0f) slider->setMotor(true, -1.5f, 3000.0f);
        else if (position <= 0.05 && speed < 0.0f) slider->setMotor(true, 1.5f, 3000.0f);
    };
    return runtime;
}

std::vector<SceneInfo> makeScenes() {
    const SimulationConfig defaults;
    return {
        {"Playground", "Two boxes and a ball; spawn more and drag them.", 40.0f, Vector2(0, 0), defaults, buildPlayground},
        {"Tower", "Twelve boxes stacked with small offsets.", 40.0f, Vector2(0, 0), stackedConfig(), buildTower},
        {"Pyramid", "A 36-box pyramid held up by friction.", 40.0f, Vector2(0, 0), stackedConfig(), buildPyramid},
        {"Dominoes", "Twenty thin boxes set off by a rolling ball.", 40.0f, Vector2(0, 0), stackedConfig(), buildDominoes},
        {"Chain and pendulum", "Revolute capsule chain and a distance-joint pendulum hitting a stack.", 40.0f, Vector2(0, 0), stackedConfig(), buildChainAndPendulum},
        {"Capsules and ramps", "Capsules and balls sliding down static ramps.", 40.0f, Vector2(0, 0), defaults, buildRamps},
        {"Seesaw and mass ratio", "A heavy ball launches a light box from a hinged plank.", 40.0f, Vector2(0, 0), stackedConfig(), buildSeesaw},
        {"Machines", "A motorized paddle and roller, and a prismatic slider.", 40.0f, Vector2(0, 0), stackedConfig(), buildMachines},
    };
}

} // namespace

const std::vector<SceneInfo>& sceneList() {
    static const std::vector<SceneInfo> scenes = makeScenes();
    return scenes;
}

} // namespace Visualization
} // namespace PhysicsEngine

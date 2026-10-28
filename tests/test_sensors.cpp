#include "catch_amalgamated.hpp"
#include "physics/physics.h"
#include <cstring>
#include <limits>
#include <string>
#include <tuple>
#include <vector>

using namespace PhysicsEngine;

namespace {
struct Recorder : ICollisionListener {
    struct Entry { char phase; bool sensor; std::uint64_t a, b; std::size_t step; };
    std::vector<Entry> entries;
    std::size_t step = 0;
    std::size_t manifolds = 0;
    void onCollisionBegin(const CollisionEvent& e) override { entries.push_back({'B', e.sensor, e.bodyAId, e.bodyBId, step}); }
    void onCollisionPersist(const CollisionEvent& e) override { entries.push_back({'P', e.sensor, e.bodyAId, e.bodyBId, step}); }
    void onCollisionEnd(const CollisionEvent& e) override { entries.push_back({'E', e.sensor, e.bodyAId, e.bodyBId, step}); }
    void onCollision(const CollisionManifold&) override { ++manifolds; }
    // Phases of the events between bodies x and y, in order.
    std::string phases(std::uint64_t x, std::uint64_t y, bool sensor) const {
        std::string out;
        for (const auto& e : entries)
            if (e.sensor == sensor && ((e.a == x && e.b == y) || (e.a == y && e.b == x))) out += e.phase;
        return out;
    }
    std::size_t count(bool sensor) const {
        std::size_t n = 0;
        for (const auto& e : entries) n += e.sensor == sensor;
        return n;
    }
};

// Exact bit pattern of everything a step can change on a body.
struct BodyState {
    std::array<float, 8> values;
    bool awake;
    explicit BodyState(const RigidBody& b)
        : values{b.position.x, b.position.y, b.orientation, b.velocity.x, b.velocity.y,
                 b.angularVelocity, b.force.x, b.force.y}, awake(b.IsAwake()) {}
    bool operator==(const BodyState& other) const {
        return std::memcmp(values.data(), other.values.data(), sizeof(values)) == 0 && awake == other.awake;
    }
};

SimulationConfig Config(bool speculative, bool sleeping = false) {
    SimulationConfig config;
    config.enableSpeculativeContacts = speculative;
    config.enableSleeping = sleeping;
    return config;
}

RigidBodyPtr Ball(Vector2 position, float radius = 0.25f) {
    return std::make_shared<RigidBody>(Circle(radius), Material{}, position);
}
RigidBodyPtr Box(Vector2 position, float w, float h, bool isStatic) {
    return std::make_shared<RigidBody>(Polygon::MakeBox(w, h), Material{}, position, isStatic);
}
void AddGravity(World& world) { world.addUniversalForce(std::make_unique<Gravity>(Vector2{0, -9.81f})); }
}

TEST_CASE("Sensor and user data flags default off and round-trip", "[sensor]") {
    auto body = Ball({0, 0});
    REQUIRE_FALSE(body->IsSensor());
    REQUIRE(body->GetUserData() == 0);
    body->SetSensor(true);
    REQUIRE(body->IsSensor());
    body->SetUserData(std::numeric_limits<std::uintptr_t>::max());
    REQUIRE(body->GetUserData() == std::numeric_limits<std::uintptr_t>::max());
    int payload = 7;
    body->SetUserData(reinterpret_cast<std::uintptr_t>(&payload));
    REQUIRE(*reinterpret_cast<int*>(body->GetUserData()) == 7);
    body->SetSensor(false);
    REQUIRE_FALSE(body->IsSensor());

    World world;
    world.addBody(body);
    body->SetUserData(42);
    for (int i = 0; i < 10; ++i) world.step(1.0f / 60);
    REQUIRE(body->GetUserData() == 42);
}

TEST_CASE("A ball falls through a static sensor on its free-fall path", "[sensor]") {
    const bool speculative = GENERATE(true, false);
    CAPTURE(speculative);
    World plain(Config(speculative)), sensed(Config(speculative));
    auto plainBall = Ball({0, 5});
    auto ball = Ball({0, 5});
    auto sensor = Box({0, 2}, 2, 1, true);
    sensor->SetSensor(true);
    plain.addBody(plainBall); AddGravity(plain);
    sensed.addBody(sensor); sensed.addBody(ball); AddGravity(sensed);
    Recorder events;
    sensed.addCollisionListener(&events);
    bool overlapped = false;
    for (int i = 0; i < 120; ++i) {
        events.step = i;
        plain.step(1.0f / 60); sensed.step(1.0f / 60);
        REQUIRE(BodyState(*ball) == BodyState(*plainBall));
        const auto& s = sensed.getLastStepStatistics();
        REQUIRE(s.resolvedContactCount == 0);
        REQUIRE(s.activeContactCount == 0);
        REQUIRE(s.solvedConstraintCount == 0);
        REQUIRE(sensed.getPersistentContactCount() == 0);
        overlapped = overlapped || events.entries.size() > 0;
        // While they overlap, the pair is listed as a candidate.
        const bool inside = !events.entries.empty() && events.entries.back().step == std::size_t(i)
            && events.entries.back().phase != 'E';
        if (inside) REQUIRE(sensed.getPotentialCollisions().size() == 1);
    }
    REQUIRE(overlapped);
    REQUIRE(ball->position.y < 0);
    const std::string phases = events.phases(sensor->GetId(), ball->GetId(), true);
    REQUIRE(phases.size() >= 3);
    REQUIRE(phases.front() == 'B');
    REQUIRE(phases.back() == 'E');
    REQUIRE(phases.substr(1, phases.size() - 2) == std::string(phases.size() - 2, 'P'));
    REQUIRE(events.count(false) == 0);
    REQUIRE(events.manifolds == phases.size() - 1); // onCollision accompanies Begin/Persist.
}

TEST_CASE("A static sensor inside a pile changes no body's state", "[sensor]") {
    const bool speculative = GENERATE(true, false);
    const bool sleeping = GENERATE(false, true);
    CAPTURE(speculative, sleeping);
    const auto build = [](World& world, bool withSensor, RigidBodyPtr& sensor) {
        // The sensor goes first so that every other body's index shifts.
        if (withSensor) {
            sensor = Box({0.1f, 1.2f}, 3, 1.5f, true);
            sensor->SetSensor(true);
            world.addBody(sensor);
        }
        world.addBody(Box({0, -0.5f}, 20, 1, true));
        for (int i = 0; i < 4; ++i) world.addBody(Box({0.05f * i, 0.5f + 1.0f * i}, 1, 1, false));
        world.addBody(Ball({1.5f, 0.3f}));
        AddGravity(world);
    };
    World plain(Config(speculative, sleeping)), sensed(Config(speculative, sleeping));
    RigidBodyPtr unused, sensor;
    build(plain, false, unused);
    build(sensed, true, sensor);
    Recorder events;
    sensed.addCollisionListener(&events);
    for (int i = 0; i < 400; ++i) {
        events.step = i;
        plain.step(1.0f / 60); sensed.step(1.0f / 60);
        const auto& a = plain.getBodies();
        const auto& b = sensed.getBodies();
        REQUIRE(a.size() + 1 == b.size());
        for (std::size_t k = 0; k < a.size(); ++k) REQUIRE(BodyState(*a[k]) == BodyState(*b[k + 1]));
        const auto& pa = plain.getLastStepStatistics();
        const auto& pb = sensed.getLastStepStatistics();
        REQUIRE(pa.resolvedContactCount == pb.resolvedContactCount);
        REQUIRE(pa.activeContactCount == pb.activeContactCount);
        REQUIRE(pa.solvedConstraintCount == pb.solvedConstraintCount);
        REQUIRE(pa.islandCount == pb.islandCount);
        REQUIRE(pa.sleepingBodyCount == pb.sleepingBodyCount);
        REQUIRE(plain.getPersistentContactCount() == sensed.getPersistentContactCount());
    }
    if (sleeping) REQUIRE(sensed.getLastStepStatistics().sleepingBodyCount > 0);
    // The sensor overlaps the two lower boxes and the ball for the whole run.
    std::size_t begins = 0;
    for (const auto& e : events.entries) if (e.sensor && e.phase == 'B') ++begins;
    REQUIRE(begins >= 3);
    for (const auto& e : events.entries)
        if (e.sensor) REQUIRE((e.a == sensor->GetId() || e.b == sensor->GetId()));
}

TEST_CASE("Sensor pairs obey filtering and sensors ignore each other", "[sensor]") {
    const bool speculative = GENERATE(true, false);
    CAPTURE(speculative);
    World world(Config(speculative));
    auto sensor = Box({0, 0}, 2, 2, true);
    sensor->SetSensor(true);
    sensor->SetCollisionCategoryBits(0x2);
    sensor->SetCollisionMaskBits(0x4);          // detects category 4 only
    auto seen = Ball({0, 0});    seen->SetCollisionCategoryBits(0x4);
    auto ignored = Ball({0.2f, 0}); ignored->SetCollisionCategoryBits(0x8);
    auto otherSensor = Ball({-0.2f, 0}); otherSensor->SetCollisionCategoryBits(0x4); otherSensor->SetSensor(true);
    auto jointed = Ball({0, 0.2f}); jointed->SetCollisionCategoryBits(0x4);
    for (const auto& b : {sensor, seen, ignored, otherSensor, jointed}) world.addBody(b);
    world.addJoint(std::make_shared<DistanceJoint>(sensor, jointed, 0.2f, Vector2{}, Vector2{}));
    Recorder events;
    world.addCollisionListener(&events);
    world.step(0); world.step(0);
    REQUIRE(events.phases(sensor->GetId(), seen->GetId(), true) == "BP");
    REQUIRE(events.phases(sensor->GetId(), ignored->GetId(), true).empty());
    REQUIRE(events.phases(sensor->GetId(), otherSensor->GetId(), true).empty());
    REQUIRE(events.phases(sensor->GetId(), jointed->GetId(), true).empty());
    // otherSensor (dynamic) still detects the balls it overlaps.
    REQUIRE(events.phases(otherSensor->GetId(), seen->GetId(), true) == "BP");
    // Changing the mask ends the overlap like any filtered contact.
    sensor->SetCollisionMaskBits(0);
    world.step(0);
    REQUIRE(events.phases(sensor->GetId(), seen->GetId(), true) == "BPE");
}

TEST_CASE("Sensors neither wake nor keep awake sleeping bodies", "[sensor]") {
    const bool speculative = GENERATE(true, false);
    CAPTURE(speculative);
    World world(Config(speculative, true));
    world.addBody(Box({0, -0.5f}, 20, 1, true));
    auto box = Box({0, 0.5f}, 1, 1, false);
    world.addBody(box);
    AddGravity(world);
    for (int i = 0; i < 300 && box->IsAwake(); ++i) world.step(1.0f / 60);
    REQUIRE_FALSE(box->IsAwake());
    const BodyState asleep(*box);

    Recorder events;
    world.addCollisionListener(&events);
    // A static sensor moved onto the sleeper (a kinematic-style trigger).
    auto trigger = Box({5, 0.5f}, 1.5f, 1.5f, true);
    trigger->SetSensor(true);
    world.addBody(trigger);
    world.step(1.0f / 60);
    trigger->SetPosition({0, 0.5f});
    for (int i = 0; i < 5; ++i) world.step(1.0f / 60);
    REQUIRE(events.phases(trigger->GetId(), box->GetId(), true) == "BPPPP");
    REQUIRE(BodyState(*box) == asleep);

    // A dynamic sensor falls through the sleeper without waking it.
    auto probe = Ball({0, 3}, 0.2f);
    probe->SetSensor(true);
    world.addBody(probe);
    for (int i = 0; i < 90; ++i) world.step(1.0f / 60);
    const std::string probePhases = events.phases(probe->GetId(), box->GetId(), true);
    REQUIRE(probePhases.front() == 'B');
    REQUIRE(probePhases.back() == 'E');
    REQUIRE(probe->position.y < -2);
    REQUIRE(BodyState(*box) == asleep);

    // Removing the sensor mid-overlap ends the event and still wakes nobody.
    world.removeBody(trigger);
    REQUIRE(events.phases(trigger->GetId(), box->GetId(), true).back() == 'E');
    world.step(1.0f / 60);
    REQUIRE(BodyState(*box) == asleep);
}

TEST_CASE("Removing a sensor or its visitor mid-overlap raises one End", "[sensor]") {
    const bool speculative = GENERATE(true, false);
    const bool removeSensor = GENERATE(true, false);
    CAPTURE(speculative, removeSensor);
    World world(Config(speculative));
    auto sensor = Box({0, 0}, 2, 2, false);
    sensor->SetSensor(true);
    auto ball = Ball({0, 0});
    world.addBody(sensor); world.addBody(ball);
    Recorder events;
    world.addCollisionListener(&events);
    world.step(0); world.step(0);
    world.removeBody(removeSensor ? sensor : ball);
    REQUIRE(events.phases(sensor->GetId(), ball->GetId(), true) == "BPE");
    world.step(0);
    REQUIRE(events.entries.size() == 3);
}

TEST_CASE("A dynamic sensor integrates gravity and falls through everything", "[sensor]") {
    const bool speculative = GENERATE(true, false);
    CAPTURE(speculative);
    // alone: the sensor's shape falling alone; scene: the same scene without the sensor.
    World alone(Config(speculative)), scene(Config(speculative)), world(Config(speculative));
    auto reference = Box({0.2f, 2}, 0.5f, 0.5f, false);
    reference->SetAngularVelocity(1);
    auto sensor = Box({0.2f, 2}, 0.5f, 0.5f, false);
    sensor->SetAngularVelocity(1);
    sensor->SetSensor(true);
    auto ground = Box({0, -0.5f}, 20, 1, true), sceneGround = Box({0, -0.5f}, 20, 1, true);
    auto resting = Box({0, 0.5f}, 1, 1, false), sceneResting = Box({0, 0.5f}, 1, 1, false);
    alone.addBody(reference); AddGravity(alone);
    scene.addBody(sceneGround); scene.addBody(sceneResting); AddGravity(scene);
    world.addBody(ground); world.addBody(resting); world.addBody(sensor); AddGravity(world);
    Recorder events;
    world.addCollisionListener(&events);
    for (int i = 0; i < 90; ++i) {
        alone.step(1.0f / 60); scene.step(1.0f / 60); world.step(1.0f / 60);
        REQUIRE(BodyState(*sensor) == BodyState(*reference));
        REQUIRE(BodyState(*resting) == BodyState(*sceneResting));
    }
    REQUIRE(sensor->position.y < -3);
    REQUIRE(events.phases(sensor->GetId(), resting->GetId(), true).front() == 'B');
    REQUIRE(events.phases(sensor->GetId(), resting->GetId(), true).back() == 'E');
    REQUIRE(events.phases(sensor->GetId(), ground->GetId(), true).front() == 'B');
    REQUIRE(events.phases(sensor->GetId(), ground->GetId(), true).back() == 'E');
}

TEST_CASE("Only real overlaps raise sensor events", "[sensor]") {
    const bool speculative = GENERATE(true, false);
    CAPTURE(speculative);
    World world(Config(speculative));
    auto sensor = Box({0, 0}, 2, 2, true);
    sensor->SetSensor(true);
    // 0.1 m apart, closing 0.075 m per step: inside the speculative margin
    // after the first step, overlapping after the second.
    auto ball = Ball({1.35f, 0}, 0.25f);
    ball->SetVelocity({-4.5f, 0});
    world.addBody(sensor); world.addBody(ball);
    Recorder events;
    world.addCollisionListener(&events);
    world.step(1.0f / 60);
    REQUIRE(ball->position.x > 1.25f);
    REQUIRE(events.entries.empty());
    world.step(1.0f / 60);
    REQUIRE(ball->position.x < 1.25f);
    REQUIRE(events.phases(sensor->GetId(), ball->GetId(), true) == "B");
}

TEST_CASE("Sensors are not swept by CCD", "[sensor]") {
    const bool speculative = GENERATE(true, false);
    CAPTURE(speculative);
    World plain(Config(speculative)), world(Config(speculative));
    auto wall = Box({0, 0}, 0.1f, 4, true);
    wall->SetSensor(true);
    auto ball = Ball({-2, 0}, 0.1f), reference = Ball({-2, 0}, 0.1f);
    for (const auto& b : {ball, reference}) { b->SetCcdEnabled(true); b->SetVelocity({300, 0}); }
    world.addBody(wall); world.addBody(ball);
    plain.addBody(reference);
    Recorder events;
    world.addCollisionListener(&events);
    for (int i = 0; i < 3; ++i) {
        plain.step(1.0f / 60); world.step(1.0f / 60);
        REQUIRE(world.getLastStepStatistics().ccdImpactCount == 0);
        REQUIRE(BodyState(*ball) == BodyState(*reference));
    }
    REQUIRE(ball->position.x > 2);
    // The step jumps over the thin sensor, so no overlap is ever seen.
    REQUIRE(events.entries.empty());
}

TEST_CASE("Toggling a sensor ends one lifecycle and begins the other", "[sensor]") {
    const bool speculative = GENERATE(true, false);
    CAPTURE(speculative);
    World world(Config(speculative, true));
    auto floor = Box({0, -0.5f}, 20, 1, true);
    auto box = Box({0, 0.5f}, 1, 1, false);
    world.addBody(floor); world.addBody(box);
    AddGravity(world);
    for (int i = 0; i < 300 && box->IsAwake(); ++i) world.step(1.0f / 60);
    REQUIRE_FALSE(box->IsAwake());
    Recorder events;
    world.addCollisionListener(&events);
    world.step(1.0f / 60);
    floor->SetSensor(true);
    world.step(1.0f / 60);
    REQUIRE(box->IsAwake());       // The floor gave way: the box must fall.
    const auto n = events.entries.size();
    REQUIRE(n >= 3);
    REQUIRE((events.entries[n - 2].phase == 'E' && !events.entries[n - 2].sensor));
    REQUIRE((events.entries[n - 1].phase == 'B' && events.entries[n - 1].sensor));
    for (int i = 0; i < 60; ++i) world.step(1.0f / 60);
    REQUIRE(box->position.y < -1);
    REQUIRE(events.phases(floor->GetId(), box->GetId(), true).back() == 'E');

    // A sensor made solid wakes the body asleep inside it, which is pushed out.
    auto zone = Box({5, 0}, 2, 2, true);
    zone->SetSensor(true);
    auto ground = Box({5, -1.5f}, 6, 1, true);
    auto guest = Box({5, -0.5f}, 1, 1, false);
    World second(Config(speculative, true));
    second.addBody(zone); second.addBody(ground); second.addBody(guest);
    AddGravity(second);
    for (int i = 0; i < 300 && guest->IsAwake(); ++i) second.step(1.0f / 60);
    REQUIRE_FALSE(guest->IsAwake());
    zone->SetSensor(false);
    second.step(1.0f / 60);
    REQUIRE(guest->IsAwake());
}

TEST_CASE("Sensor events are deterministic and follow contact events", "[sensor]") {
    const bool speculative = GENERATE(true, false);
    CAPTURE(speculative);
    using Trace = std::vector<std::tuple<std::size_t, char, bool, std::size_t, std::size_t>>;
    const auto run = [speculative]() {
        World world(Config(speculative));
        std::vector<RigidBodyPtr> bodies;
        bodies.push_back(Box({0, -0.5f}, 20, 1, true));
        auto sensor = Box({0, 1}, 6, 1, true);
        sensor->SetSensor(true);
        bodies.push_back(sensor);
        for (int i = 0; i < 5; ++i) bodies.push_back(Ball({-2.0f + i, 3.0f + 0.3f * i}));
        for (const auto& b : bodies) world.addBody(b);
        AddGravity(world);
        Recorder events;
        world.addCollisionListener(&events);
        for (int i = 0; i < 150; ++i) { events.step = i; world.step(1.0f / 60); }
        const auto index = [&](std::uint64_t id) {
            for (std::size_t k = 0; k < bodies.size(); ++k) if (bodies[k]->GetId() == id) return k;
            return bodies.size();
        };
        Trace trace;
        for (const auto& e : events.entries) trace.emplace_back(e.step, e.phase, e.sensor, index(e.a), index(e.b));
        return trace;
    };
    const Trace first = run(), second = run();
    REQUIRE(first.size() == second.size());
    for (std::size_t k = 0; k < first.size(); ++k) {
        CAPTURE(k, std::get<0>(first[k]), std::get<1>(first[k]), std::get<2>(first[k]), std::get<3>(first[k]), std::get<4>(first[k]));
        CAPTURE(std::get<0>(second[k]), std::get<1>(second[k]), std::get<2>(second[k]), std::get<3>(second[k]), std::get<4>(second[k]));
        REQUIRE(first[k] == second[k]);
    }
    std::size_t sensorBegins = 0, contactBegins = 0;
    for (std::size_t k = 0; k < first.size(); ++k) {
        const auto& [step, phase, sensor, a, b] = first[k];
        sensorBegins += sensor && phase == 'B';
        contactBegins += !sensor && phase == 'B';
        REQUIRE(a < b); // canonical: lower id first
        // Within a step, Begin/Persist contacts come before Begin/Persist sensor
        // events, and every End comes last.
        if (k + 1 < first.size() && std::get<0>(first[k + 1]) == step) {
            const char nextPhase = std::get<1>(first[k + 1]);
            const bool nextSensor = std::get<2>(first[k + 1]);
            if (phase == 'E') REQUIRE(nextPhase == 'E');
            if (sensor && phase != 'E' && nextPhase != 'E') REQUIRE(nextSensor);
        }
    }
    REQUIRE(sensorBegins >= 5);   // every ball passes into the sensor
    REQUIRE(contactBegins >= 5);  // and lands on the ground
}

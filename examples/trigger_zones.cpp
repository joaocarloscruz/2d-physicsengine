#include "physics/physics.h"
#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>
#include <unordered_map>

// Trigger zones: bodies enter and leave several sensor zones, a listener counts
// the entries and exits per zone and prints an event log. Collision filtering
// decides which body is reported by which zone. The program exits nonzero if
// any count differs from the expected one.
//
//   zone A (x=4)  accepts every category
//   zone B (x=8)  accepts only the red category
//   zone C (x=12) accepts only the blue category
//   red   goes right through all three zones, turns around and comes back
//   blue  goes right through all three zones and stops
//   ghost goes right too, but its mask excludes the zones' category, so no
//         zone ever reports it
using namespace PhysicsEngine;

namespace {
constexpr std::uint32_t kZoneBit = 0x1, kRedBit = 0x2, kBlueBit = 0x4, kGhostBit = 0x8;

// Counts sensor overlaps per (zone, body). Zones and bodies are identified by
// the ids the application registers, not by pointers.
class ZoneCounter : public ICollisionListener {
public:
    void addZone(const RigidBody& zone, const char* name) { zones[zone.GetId()] = name; }
    void addBody(const RigidBody& body, const char* name) { bodies[body.GetId()] = name; }

    void onCollisionBegin(const CollisionEvent& event) override { record(event, true); }
    void onCollisionEnd(const CollisionEvent& event) override { record(event, false); }

    int entries(const std::string& zone, const std::string& body) const { return count(entered, zone, body); }
    int exits(const std::string& zone, const std::string& body) const { return count(left, zone, body); }
    int step = 0; // the program sets this so the log shows when events happen

private:
    void record(const CollisionEvent& event, bool begin) {
        if (!event.sensor) return; // a solid contact, not a zone overlap
        // bodyAId is the lower id of the pair, so look at both ids.
        const bool aIsZone = zones.count(event.bodyAId) != 0;
        const std::string& zone = zones[aIsZone ? event.bodyAId : event.bodyBId];
        const std::string& body = bodies[aIsZone ? event.bodyBId : event.bodyAId];
        (begin ? entered : left)[zone + "/" + body]++;
        std::printf("  step %3d: %-5s %s zone %s\n", step, body.c_str(), begin ? "entered" : "left   ", zone.c_str());
    }
    static int count(const std::unordered_map<std::string, int>& map, const std::string& zone, const std::string& body) {
        const auto it = map.find(zone + "/" + body);
        return it == map.end() ? 0 : it->second;
    }
    std::unordered_map<std::uint64_t, std::string> zones, bodies;
    std::unordered_map<std::string, int> entered, left;
};

std::shared_ptr<RigidBody> makeZone(float x, std::uint32_t mask, std::uintptr_t id) {
    auto zone = std::make_shared<RigidBody>(Polygon::MakeBox(2, 6), Material{}, Vector2{x, 0}, true);
    zone->SetSensor(true);
    zone->SetUserData(id);
    zone->SetCollisionCategoryBits(kZoneBit);
    zone->SetCollisionMaskBits(mask); // a zone reports a body only if the body's category is in this mask
    return zone;
}

std::shared_ptr<RigidBody> makeBall(float y, std::uint32_t category, std::uint32_t mask) {
    auto ball = std::make_shared<RigidBody>(Circle(0.2f), Material{}, Vector2{0, y});
    ball->SetCollisionCategoryBits(category);
    ball->SetCollisionMaskBits(mask);
    return ball;
}
}

int main() {
    // No gravity is registered, so every ball keeps the velocity it is given.
    World world;
    auto zoneA = makeZone(4, 0xFFFFFFFF, 1);
    auto zoneB = makeZone(8, kRedBit, 2);
    auto zoneC = makeZone(12, kBlueBit, 3);
    auto red = makeBall(-1.0f, kRedBit, 0xFFFFFFFF);
    auto blue = makeBall(0.0f, kBlueBit, 0xFFFFFFFF);
    auto ghost = makeBall(1.0f, kGhostBit, ~kZoneBit);
    for (const auto& body : {zoneA, zoneB, zoneC, red, blue, ghost}) world.addBody(body);
    for (const auto& ball : {red, blue, ghost}) ball->SetVelocity({8, 0});

    ZoneCounter counter;
    counter.addZone(*zoneA, "A");
    counter.addZone(*zoneB, "B");
    counter.addZone(*zoneC, "C");
    counter.addBody(*red, "red");
    counter.addBody(*blue, "blue");
    counter.addBody(*ghost, "ghost");
    world.addCollisionListener(&counter);

    std::printf("event log:\n");
    bool turned = false;
    for (int frame = 0; frame < 420; ++frame) {
        counter.step = frame;
        world.step();
        if (red->position.x >= 16 && !turned) { // the application steers: red comes back
            red->SetVelocity({-8, 0});
            turned = true;
        }
        if (blue->position.x >= 16) blue->SetVelocity({0, 0}); // blue parks outside every zone
        if (ghost->position.x >= 16) ghost->SetVelocity({0, 0});
    }
    world.removeCollisionListener(&counter);

    struct Expect { const char* zone; const char* body; int count; };
    const Expect expected[] = {
        {"A", "red", 2}, {"A", "blue", 1}, {"A", "ghost", 0},
        {"B", "red", 2}, {"B", "blue", 0}, {"B", "ghost", 0},
        {"C", "red", 0}, {"C", "blue", 1}, {"C", "ghost", 0},
    };
    bool ok = turned;
    std::printf("zone/body  entries exits (expected)\n");
    for (const auto& e : expected) {
        const int in = counter.entries(e.zone, e.body), out = counter.exits(e.zone, e.body);
        std::printf("  %s/%-5s  %d       %d     (%d)\n", e.zone, e.body, in, out, e.count);
        ok = ok && in == e.count && out == e.count;
    }
    ok = ok && zoneC->GetUserData() == 3;
    std::printf("%s\n", ok ? "trigger zones: ok" : "trigger zones: FAIL");
    return ok ? 0 : 1;
}

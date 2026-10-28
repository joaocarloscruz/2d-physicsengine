#include "collision_events.h"
#include "checked_indices.h"
#include <cstdint>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

namespace PhysicsEngine::Wasm {
namespace {
enum class CollisionPhase { Begin, Persist, End };

struct CollisionEventRecord {
    CollisionPhase phase = CollisionPhase::Begin;
    std::uint64_t bodyAId = 0, bodyBId = 0;
    Vector2 normal;
    float penetration = 0.0f;
    std::uint8_t contactCount = 0;
    Vector2 point0, point1;
    bool sensor = false;
};

constexpr std::size_t kDefaultCapacity = 1024;
constexpr std::size_t kMaxCapacity = std::size_t{1} << 20;

// Copies each event World delivers into a bounded buffer. It never calls back
// into JS, so stepping stays free of JS re-entry.
class Recorder final : public ICollisionListener {
public:
    void onCollisionBegin(const CollisionEvent& e) override { Record(CollisionPhase::Begin, e); }
    void onCollisionPersist(const CollisionEvent& e) override { Record(CollisionPhase::Persist, e); }
    void onCollisionEnd(const CollisionEvent& e) override { Record(CollisionPhase::End, e); }

    bool enabled = false;
    std::size_t capacity = kDefaultCapacity;
    std::size_t dropped = 0;
    std::vector<CollisionEventRecord> events;

    void Clear() { events.clear(); dropped = 0; }
    void SetCapacity(std::size_t value) {
        if (value < 1 || value > kMaxCapacity)
            throw std::invalid_argument("Collision event capacity must be between 1 and 1048576");
        capacity = value;
        if (events.size() > capacity) { dropped += events.size() - capacity; events.resize(capacity); }
    }

private:
    void Record(CollisionPhase phase, const CollisionEvent& e) {
        // A full buffer keeps the oldest events (a consistent prefix of the
        // lifecycle) and counts what it had to drop.
        if (events.size() >= capacity) { ++dropped; return; }
        CollisionEventRecord r;
        r.phase = phase;
        r.bodyAId = e.bodyAId; r.bodyBId = e.bodyBId;
        r.normal = e.normal; r.penetration = e.penetration;
        r.contactCount = e.contactCount;
        r.point0 = e.contacts[0].position; r.point1 = e.contacts[1].position;
        r.sensor = e.sensor;
        events.push_back(r);
    }
};

class EventList {
public:
    EventList(std::vector<CollisionEventRecord> values, std::size_t droppedCount)
        : events(std::move(values)), dropped(droppedCount) {}
    std::size_t size() const { return events.size(); }
    CollisionEventRecord getEvent(double index) const { return events.at(Index(index, events.size())); }
    bool overflowed() const { return dropped > 0; }
    double getDroppedCount() const { return static_cast<double>(dropped); }
private:
    std::vector<CollisionEventRecord> events;
    std::size_t dropped;
};

struct Holder {
    // Declared first so it is destroyed last: nothing the engine does while
    // being destroyed can reach a freed listener.
    Recorder recorder;
    Engine engine;
    ~Holder();
};
std::unordered_map<const Engine*, Holder*>& Registry() {
    static std::unordered_map<const Engine*, Holder*> registry;
    return registry;
}
Holder::~Holder() {
    engine.removeCollisionListener(&recorder);
    Registry().erase(&engine);
}
Recorder& RecorderOf(const Engine& engine) {
    const auto found = Registry().find(&engine);
    if (found == Registry().end()) throw std::logic_error("Engine was not created by the bindings");
    return found->second->recorder;
}

void Enable(Engine& engine, bool enabled) {
    auto& recorder = RecorderOf(engine);
    if (enabled == recorder.enabled) return;
    if (enabled) engine.addCollisionListener(&recorder);
    else engine.removeCollisionListener(&recorder);
    recorder.enabled = enabled;
}
EventList* Snapshot(const Engine& engine) {
    const auto& recorder = RecorderOf(engine);
    return new EventList(recorder.events, recorder.dropped);
}
EventList* Take(Engine& engine) {
    auto& recorder = RecorderOf(engine);
    auto* list = new EventList(std::move(recorder.events), recorder.dropped);
    recorder.Clear();
    return list;
}
}

std::shared_ptr<Engine> CreateEngine() {
    auto holder = std::make_shared<Holder>();
    Registry().emplace(&holder->engine, holder.get());
    return std::shared_ptr<Engine>(holder, &holder->engine);
}

void AddCollisionEventMethods(emscripten::class_<Engine>& engineClass) {
    using namespace emscripten;
    engineClass
        .function("enableCollisionEvents", &Enable)
        .function("isCollisionEventsEnabled", optional_override([](const Engine& e) { return RecorderOf(e).enabled; }))
        .function("setCollisionEventCapacity", optional_override([](Engine& e, double n) { RecorderOf(e).SetCapacity(Count(n)); }))
        .function("getCollisionEventCapacity", optional_override([](const Engine& e) { return static_cast<double>(RecorderOf(e).capacity); }))
        .function("getCollisionEvents", &Snapshot, allow_raw_pointers())
        .function("takeCollisionEvents", &Take, allow_raw_pointers())
        .function("clearCollisionEvents", optional_override([](Engine& e) { RecorderOf(e).Clear(); }));
}
}

EMSCRIPTEN_BINDINGS(collision_events) {
    using namespace emscripten;
    using namespace PhysicsEngine;
    using namespace PhysicsEngine::Wasm;
    enum_<CollisionPhase>("CollisionPhase")
        .value("Begin", CollisionPhase::Begin)
        .value("Persist", CollisionPhase::Persist)
        .value("End", CollisionPhase::End);
    value_object<CollisionEventRecord>("CollisionEvent")
        .field("phase", &CollisionEventRecord::phase)
        .field("bodyAId", &CollisionEventRecord::bodyAId).field("bodyBId", &CollisionEventRecord::bodyBId)
        .field("normal", &CollisionEventRecord::normal).field("penetration", &CollisionEventRecord::penetration)
        .field("contactCount", &CollisionEventRecord::contactCount)
        .field("point0", &CollisionEventRecord::point0).field("point1", &CollisionEventRecord::point1)
        .field("sensor", &CollisionEventRecord::sensor);
    class_<EventList>("CollisionEventList")
        .function("size", &EventList::size).function("getEvent", &EventList::getEvent)
        .function("overflowed", &EventList::overflowed).function("getDroppedCount", &EventList::getDroppedCount);
}

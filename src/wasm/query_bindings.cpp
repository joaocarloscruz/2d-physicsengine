#include <emscripten/bind.h>
#include "engine.h"
#include "physics/core/spatial_queries.h"
#include "checked_indices.h"
#include <limits>
#include <utility>

namespace PhysicsEngine::Wasm {
namespace {
struct QueryFilterInput { double categoryBits, maskBits; };
std::uint32_t Bits(double value) {
    const auto count = Count(value);
    if (count > std::numeric_limits<std::uint32_t>::max())
        throw std::invalid_argument("Query filter bits must fit uint32");
    return static_cast<std::uint32_t>(count);
}
QueryFilter Native(QueryFilterInput input) { return {Bits(input.categoryBits), Bits(input.maskBits)}; }
const RigidBodyPtr& Body(const RigidBodyPtr& entry) { return entry; }
const RigidBodyPtr& Body(const WorldRayHit& entry) { return entry.body; }
const RigidBodyPtr& Body(const WorldSweptCircleHit& entry) { return entry.body; }
template<class Entry> struct Results {
    std::vector<Entry> entries;
    std::size_t size() const { return entries.size(); }
    const Entry& at(double index) const { return entries.at(Index(index, entries.size())); }
    std::uint64_t getBodyId(double index) const { return Body(at(index))->GetId(); }
    RigidBodyPtr getBody(double index) const { return Body(at(index)); }
};
using BodyResults = Results<RigidBodyPtr>;
using RayResults = Results<WorldRayHit>;
using SweepResults = Results<WorldSweptCircleHit>;
template<class Entry> Results<Entry>* Own(std::vector<Entry> values) {
    return new Results<Entry>{std::move(values)};
}
template<class Entry> Results<Entry>* Own(std::optional<Entry> value) {
    std::vector<Entry> values;
    if (value) values.push_back(std::move(*value));
    return Own(std::move(values));
}
}
}

EMSCRIPTEN_BINDINGS(spatial_queries) {
    using namespace emscripten;
    using namespace PhysicsEngine;
    using namespace PhysicsEngine::Wasm;
    value_object<QueryFilterInput>("QueryFilter")
        .field("categoryBits", &QueryFilterInput::categoryBits).field("maskBits", &QueryFilterInput::maskBits);
    value_object<RayHit>("RayHit")
        .field("fraction", &RayHit::fraction).field("point", &RayHit::point).field("normal", &RayHit::normal);
    value_object<SweptCircleHit>("SweptCircleHit")
        .field("fraction", &SweptCircleHit::fraction).field("center", &SweptCircleHit::center)
        .field("contactPoint", &SweptCircleHit::contactPoint).field("normal", &SweptCircleHit::normal);
    class_<BodyResults>("BodyQueryResults")
        .function("size", &BodyResults::size).function("getBodyId", &BodyResults::getBodyId)
        .function("getBody", &BodyResults::getBody);
    class_<RayResults>("RayQueryResults")
        .function("size", &RayResults::size).function("getBodyId", &RayResults::getBodyId)
        .function("getBody", &RayResults::getBody)
        .function("getHit", optional_override([](const RayResults& r, double i) { return r.at(i).hit; }));
    class_<SweepResults>("CircleSweepResults")
        .function("size", &SweepResults::size).function("getBodyId", &SweepResults::getBodyId)
        .function("getBody", &SweepResults::getBody)
        .function("getHit", optional_override([](const SweepResults& r, double i) { return r.at(i).hit; }));
    function("queryPoint", optional_override([](const Engine& e, Vector2 p) { return Own(QueryPoint(e,p)); }), allow_raw_pointers());
    function("queryPoint", optional_override([](const Engine& e, Vector2 p, QueryFilterInput f) { return Own(QueryPoint(e,p,Native(f))); }), allow_raw_pointers());
    function("queryCircle", optional_override([](const Engine& e, Vector2 p, float radius) { return Own(QueryCircle(e,p,radius)); }), allow_raw_pointers());
    function("queryCircle", optional_override([](const Engine& e, Vector2 p, float radius, QueryFilterInput f) { return Own(QueryCircle(e,p,radius,Native(f))); }), allow_raw_pointers());
    function("rayCastAll", optional_override([](const Engine& e, Vector2 a, Vector2 b) { return Own(RayCastAll(e,a,b)); }), allow_raw_pointers());
    function("rayCastAll", optional_override([](const Engine& e, Vector2 a, Vector2 b, QueryFilterInput f) { return Own(RayCastAll(e,a,b,Native(f))); }), allow_raw_pointers());
    function("rayCastNearest", optional_override([](const Engine& e, Vector2 a, Vector2 b) { return Own(RayCastNearest(e,a,b)); }), allow_raw_pointers());
    function("rayCastNearest", optional_override([](const Engine& e, Vector2 a, Vector2 b, QueryFilterInput f) { return Own(RayCastNearest(e,a,b,Native(f))); }), allow_raw_pointers());
    function("sweepCircleAll", optional_override([](const Engine& e, Vector2 a, Vector2 b, float radius) { return Own(SweepCircleAll(e,a,b,radius)); }), allow_raw_pointers());
    function("sweepCircleAll", optional_override([](const Engine& e, Vector2 a, Vector2 b, float radius, QueryFilterInput f) { return Own(SweepCircleAll(e,a,b,radius,Native(f))); }), allow_raw_pointers());
    function("sweepCircleNearest", optional_override([](const Engine& e, Vector2 a, Vector2 b, float radius) { return Own(SweepCircleNearest(e,a,b,radius)); }), allow_raw_pointers());
    function("sweepCircleNearest", optional_override([](const Engine& e, Vector2 a, Vector2 b, float radius, QueryFilterInput f) { return Own(SweepCircleNearest(e,a,b,radius,Native(f))); }), allow_raw_pointers());
}

// Test-only bindings, absent from production modules.
#include <cstdint>
#include <emscripten/bind.h>
#include <exception>
#include <malloc.h>
#include <stdexcept>
#if __has_feature(address_sanitizer)
#include <sanitizer/allocator_interface.h>
#endif

namespace {
int liveValues = 0, liveObjects = 0;
struct Value {
    double number = 0;
    Value() { ++liveValues; }
    explicit Value(double n) : number(n) { ++liveValues; }
    Value(const Value &other) : number(other.number) { ++liveValues; }
    ~Value() { --liveValues; }
};
struct Probe {
    explicit Probe(const Value &value) {
        if (value.number < 0)
            throw std::invalid_argument("probe constructor");
        ++liveObjects;
    }
    ~Probe() { --liveObjects; }
    Probe(const Probe &, const Value &value) : Probe(value) {}
    Value get() const { throw std::runtime_error("probe getter"); }
    void set(const Value &) { throw std::runtime_error("probe setter"); }
    void method(const Value &) const { throw std::runtime_error("probe method"); }
    double method() const { return 7; }
    static void function(const Value &) { throw std::runtime_error("probe static"); }
    static Value copy(const Value &value) { return Value(value.number); }
    float scalarFloat(float value) const { return value; }
    int scalarInteger(int value) const { return value; }
    std::int64_t scalarInt64(std::int64_t value) const { return value; }
    std::uint64_t scalarUint64(std::uint64_t value) const { return value; }
    static Value staticValue;
};
Value Probe::staticValue;
struct Stats {
    double heap;
    int uncaught, values, objects;
};
Stats stats() {
#if __has_feature(address_sanitizer)
    const auto heap = __sanitizer_get_current_allocated_bytes();
#else
    const auto heap = mallinfo().uordblks;
#endif
    return {static_cast<double>(heap), std::uncaught_exceptions(), liveValues, liveObjects};
}
} // namespace

EMSCRIPTEN_BINDINGS(physics_boundary_test_probes) {
    using namespace emscripten;
    value_object<Value>("BoundaryTestValue").field("number", &Value::number);
    value_object<Stats>("BoundaryTestStats")
        .field("heap", &Stats::heap)
        .field("uncaught", &Stats::uncaught)
        .field("values", &Stats::values)
        .field("objects", &Stats::objects);
    class_<Probe>("BoundaryTestProbe")
        .constructor<const Value &>()
        .constructor<const Probe &, const Value &>()
        .property("value", &Probe::get, &Probe::set)
        .class_property("staticValue", &Probe::staticValue)
        .function("method", select_overload<void(const Value &) const>(&Probe::method))
        .function("method", select_overload<double() const>(&Probe::method))
        .function("scalarFloat", &Probe::scalarFloat)
        .function("scalarInteger", &Probe::scalarInteger)
        .function("scalarInt64", &Probe::scalarInt64)
        .function("scalarUint64", &Probe::scalarUint64)
        .class_function("function", &Probe::function)
        .class_function("copy", &Probe::copy);
    function("boundaryTestStats", &stats);
    function("boundaryTestFunction", &Probe::function);
}

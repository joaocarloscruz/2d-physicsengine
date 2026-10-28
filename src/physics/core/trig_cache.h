#ifndef PHYSICS_TRIG_CACHE_H
#define PHYSICS_TRIG_CACHE_H

#include <cmath>
#include <cstdint>
#include <cstring>

namespace PhysicsEngine {
namespace Detail {

struct CosSin {
    double cosine, sine;
};

// std::cos and std::sin of one angle, memoized per thread. A body's
// orientation stays fixed through detection and each velocity iteration, so
// the solver and narrow phase ask for the same angle many times per step.
// Every value returned is exactly what std::cos/std::sin give for that bit
// pattern; the table only skips recomputing it.
inline CosSin CachedCosSin(double angle) {
    struct Entry {
        std::uint64_t key; // angle bits + 1; zero marks an empty slot
        double cosine, sine;
    };
    constexpr unsigned kBits = 12;
    thread_local Entry table[1u << kBits] = {};
    std::uint64_t bits;
    std::memcpy(&bits, &angle, sizeof bits);
    // All-ones bits (a NaN, the only pattern whose key would wrap to zero) and
    // other NaNs are not cached; validation upstream normally rejects them.
    if (angle != angle) return {std::cos(angle), std::sin(angle)};
    // Angles usually come from floats, so the low mantissa bits are zero; a
    // multiplicative hash mixes the high bits into the index.
    Entry& entry = table[(bits * 0x9E3779B97F4A7C15ull) >> (64 - kBits)];
    if (entry.key != bits + 1) {
        entry.cosine = std::cos(angle);
        entry.sine = std::sin(angle);
        entry.key = bits + 1;
    }
    return {entry.cosine, entry.sine};
}

} // namespace Detail
} // namespace PhysicsEngine

#endif // PHYSICS_TRIG_CACHE_H

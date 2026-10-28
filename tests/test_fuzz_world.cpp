// Seeded World robustness fuzz: random mixed scenes in a sealed static box,
// with joints, impulses, teleports, sleeping toggled mid-run, CCD bullets and
// both step orders. Every failure names the seed, the step order and the step,
// and the scene is rebuilt from the seed alone.
#include "catch_amalgamated.hpp"
#include "fuzz_support.h"

#include "physics/physics.h"

#include <cstring>
#include <sstream>

using namespace PhysicsEngine;
using namespace FuzzSupport;

namespace {
constexpr float kHalf = 15;      // interior half-size of the sealed box
constexpr float kWall = 2;       // wall thickness
constexpr float kGravity = 9.81f;
constexpr std::uint32_t kWallCategory = 1, kCapsuleCategory = 2, kBodyCategory = 4, kBulletCategory = 8;

struct SceneOptions {
    std::uint64_t seed = 1;
    bool speculative = true;
    int steps = 600;
    int bodies = 0; // 0: random in [50, 200]
    // Polygon outlines: 0 well-conditioned, 1 random angles with aspect >= 1/20,
    // 2 also slivers and near-coincident vertices (see the known-bug cases).
    int outlines = 0;
    // Light bullets cannot push other bodies past the documented speed.
    bool lightBullets = true;
    // CCD bullets also hit other dynamic bodies (see the known-bug cases).
    bool bulletsHitBodies = false;
};

struct Scene {
    std::unique_ptr<World> world;
    std::vector<RigidBodyPtr> dynamic;
    std::vector<double> extent;
    std::vector<bool> jointed, bullet;
    std::vector<ShapeSpec> specs;
};

ShapeSpec RandomWorldShape(Rng& rng, double target, int outlines) {
    const int kind = rng.Integer(0, 2);
    ShapeSpec spec;
    if (kind == 1 && outlines == 0) {
        spec.type = ShapeType::POLYGON;
        spec.vertices = RegularOutline(rng);
    } else {
        spec = RandomShape(rng, kind == 0 ? ShapeType::CIRCLE : kind == 1 ? ShapeType::POLYGON : ShapeType::CAPSULE, outlines == 2);
    }
    // Rescale to the cell size; the generator's own scales span 0.05 to 3.
    const double scale = target / spec.Extent();
    spec.radius = float(spec.radius * scale);
    spec.halfLength = float(spec.halfLength * scale);
    for (auto& v : spec.vertices) v = {float(v.x * scale), float(v.y * scale)};
    if (spec.type == ShapeType::POLYGON) {
        try { Polygon::ValidateOutline(spec.vertices); } catch (const std::invalid_argument&) { return RandomWorldShape(rng, target, outlines); }
    }
    return spec;
}

Material RandomMaterial(Rng& rng) {
    Material m;
    m.density = float(rng.LogRange(0.2, 5));
    m.restitution = float(rng.Range(0, 0.95));
    m.staticFriction = float(rng.Range(0, 1));
    m.dynamicFriction = float(m.staticFriction * rng.Range(0.3, 1));
    return m;
}

Vector2 LocalAnchor(const RigidBody& body, Vector2 world) {
    const double c = std::cos(double(body.orientation)), s = std::sin(double(body.orientation));
    const double dx = double(world.x) - body.position.x, dy = double(world.y) - body.position.y;
    return {float(c * dx + s * dy), float(-s * dx + c * dy)};
}

Scene BuildScene(const SceneOptions& options) {
    Rng rng(options.seed);
    Scene scene;
    SimulationConfig config;
    config.enableSpeculativeContacts = options.speculative;
    config.enableSleeping = rng.Chance(0.5);
    scene.world = std::make_unique<World>(config);
    scene.world->addUniversalForce(std::make_unique<Gravity>(Vector2{0, -kGravity}));
    const float outer = kHalf + kWall / 2;
    for (int side = 0; side < 4; ++side) {
        const bool vertical = side < 2;
        const float sign = side % 2 ? 1.0f : -1.0f;
        const Vector2 at = vertical ? Vector2{sign * outer, 0} : Vector2{0, sign * outer};
        const Polygon wall = vertical ? Polygon::MakeBox(kWall, 2 * (kHalf + kWall)) : Polygon::MakeBox(2 * (kHalf + kWall), kWall);
        scene.world->addBody(std::make_shared<RigidBody>(wall, RandomMaterial(rng), at, true));
    }
    const int count = options.bodies ? options.bodies : rng.Integer(50, 200);
    constexpr int cells = 15;
    const float cell = 2 * kHalf / cells;
    std::vector<int> slots(cells * cells);
    for (int i = 0; i < int(slots.size()); ++i) slots[i] = i;
    for (int i = int(slots.size()) - 1; i > 0; --i) std::swap(slots[i], slots[rng.Integer(0, i)]);
    for (int i = 0; i < count && i < int(slots.size()); ++i) {
        const double target = rng.Range(0.15, 0.45 * cell);
        const ShapeSpec spec = RandomWorldShape(rng, target, options.outlines);
        const Vector2 at{-kHalf + cell * (float(slots[i] % cells) + 0.5f), -kHalf + cell * (float(slots[i] / cells) + 0.5f)};
        auto body = std::make_shared<RigidBody>(*spec.Make(), RandomMaterial(rng), at);
        body->SetOrientation(float(rng.Range(-Pi, Pi)));
        body->SetVelocity({float(rng.Range(-3, 3)), float(rng.Range(-3, 3))});
        body->SetAngularVelocity(float(rng.Range(-2, 2)));
        if (spec.type == ShapeType::CAPSULE) body->SetCollisionCategoryBits(kCapsuleCategory);
        // CCD circles may not meet dynamic capsules (docs/capsules.md), so
        // they filter capsules out. Unless bullets hit bodies, they meet only
        // the walls (category 1) and each other.
        const bool ccd = spec.type == ShapeType::CIRCLE && rng.Chance(0.25);
        if (!options.bulletsHitBodies && spec.type != ShapeType::CAPSULE) body->SetCollisionCategoryBits(kBodyCategory);
        if (ccd) {
            body->SetCcdEnabled(true);
            body->SetCollisionMaskBits(options.bulletsHitBodies ? ~kCapsuleCategory : kWallCategory | kBulletCategory);
            if (!options.bulletsHitBodies) body->SetCollisionCategoryBits(kBulletCategory);
        }
        scene.world->addBody(body);
        scene.dynamic.push_back(body);
        scene.extent.push_back(spec.Extent());
        scene.jointed.push_back(false);
        scene.bullet.push_back(ccd);
        scene.specs.push_back(spec);
    }
    if (options.lightBullets) {
        float lightest = std::numeric_limits<float>::infinity();
        for (std::size_t i = 0; i < scene.dynamic.size(); ++i)
            if (!scene.bullet[i]) lightest = std::min(lightest, scene.dynamic[i]->GetMass());
        // A 150 m/s bullet of 0.15 times the lightest mass gives any body at
        // most 2 * 0.15 / 1.15 * 150 = 39 m/s, under the 60 m/s limit below.
        for (std::size_t i = 0; i < scene.dynamic.size(); ++i)
            if (scene.bullet[i]) scene.dynamic[i]->SetMass(0.15f * lightest);
    }
    // Joints between bodies in neighbouring cells, built at their current poses.
    const int jointCount = count / 10;
    for (int j = 0; j < jointCount; ++j) {
        const int i = rng.Integer(0, int(scene.dynamic.size()) - 1);
        int k = -1;
        for (int m = 0; m < int(scene.dynamic.size()); ++m) {
            if (m == i) continue;
            const Vector2 d = scene.dynamic[m]->position - scene.dynamic[i]->position;
            if (std::abs(d.x) <= cell * 1.01f && std::abs(d.y) <= cell * 1.01f) { k = m; break; }
        }
        if (k < 0) continue;
        const auto& a = scene.dynamic[i];
        const auto& b = scene.dynamic[k];
        const Vector2 mid = (a->position + b->position) * 0.5f;
        const int type = rng.Integer(0, 2);
        JointPtr joint;
        if (type == 0) {
            joint = std::make_shared<RevoluteJoint>(a, b, LocalAnchor(*a, mid), LocalAnchor(*b, mid));
        } else if (type == 1) {
            const Vector2 d = b->position - a->position;
            joint = std::make_shared<DistanceJoint>(a, b, std::hypot(d.x, d.y));
        } else {
            const Vector2 d = b->position - a->position;
            joint = std::make_shared<PrismaticJoint>(a, b, LocalAnchor(*a, a->position + d) - LocalAnchor(*a, a->position),
                                                     LocalAnchor(*a, mid), LocalAnchor(*b, mid));
        }
        scene.world->addJoint(joint);
        scene.jointed[i] = scene.jointed[k] = true;
    }
    return scene;
}

double Energy(const Scene& scene) {
    double total = 0;
    for (const auto& b : scene.dynamic) {
        const double v2 = double(b->velocity.x) * b->velocity.x + double(b->velocity.y) * b->velocity.y;
        total += 0.5 * b->GetMass() * v2 + 0.5 * b->GetInertia() * double(b->angularVelocity) * b->angularVelocity
            + b->GetMass() * kGravity * (double(b->position.y) + kHalf);
    }
    return total;
}

std::uint64_t Hash(const Scene& scene) {
    std::uint64_t h = 1469598103934665603ull;
    for (const auto& b : scene.dynamic)
        for (float value : {b->position.x, b->position.y, b->orientation, b->velocity.x, b->velocity.y, b->angularVelocity}) {
            std::uint32_t bits;
            std::memcpy(&bits, &value, sizeof bits);
            h = (h ^ bits) * 1099511628211ull;
        }
    return h;
}

struct RunResult {
    std::vector<std::uint64_t> hashes;
    Failures failures;
    double peakEnergyRatio = 0;
    int fastEscapes = 0;
};

RunResult RunScene(const SceneOptions& options) {
    RunResult result;
    Scene scene = BuildScene(options);
    Rng rng(options.seed ^ 0xA5A5A5A5u);
    const auto where = [&](int step) {
        std::ostringstream text;
        text << "seed=" << options.seed << " order=" << (options.speculative ? "speculative" : "legacy")
             << " bodies=" << scene.dynamic.size() << " step=" << step;
        return text.str();
    };
    // Mechanical energy may only fall, apart from what the test injects.
    double budget = Energy(scene);
    const double floorEnergy = 0.5; // absolute slack for position correction on resting piles
    std::vector<bool> fast(scene.dynamic.size()), escaped(scene.dynamic.size());
    int stalled = 0;
    for (int step = 0; step < options.steps; ++step) {
        if (step > 0 && step % 40 == 0) {
            const double before = Energy(scene);
            for (int k = 0; k < 5; ++k) {
                const int i = rng.Integer(0, int(scene.dynamic.size()) - 1);
                auto& body = *scene.dynamic[i];
                const double dv = rng.Range(0, 8), a = rng.Range(-Pi, Pi);
                const Vector2 lever{float(rng.Range(-0.5, 0.5) * scene.extent[i]), float(rng.Range(-0.5, 0.5) * scene.extent[i])};
                body.ApplyImpulse({float(body.GetMass() * dv * std::cos(a)), float(body.GetMass() * dv * std::sin(a))}, lever);
            }
            // Bullets: CCD circles fired at the walls at 150 m/s, below the
            // default 200 m/s speed limit and 2.5 m per step against 2 m walls.
            for (int i = 0; i < int(scene.dynamic.size()); ++i)
                if (scene.bullet[i] && rng.Chance(0.3)) {
                    const double a = rng.Range(-Pi, Pi);
                    scene.dynamic[i]->SetVelocity({float(150 * std::cos(a)), float(150 * std::sin(a))});
                }
            // Teleports into free space, for unjointed bodies.
            for (int k = 0; k < 2; ++k) {
                const int i = rng.Integer(0, int(scene.dynamic.size()) - 1);
                if (scene.jointed[i]) continue;
                const float reach = float(kHalf - scene.extent[i] - 0.1);
                const Vector2 to{float(rng.Range(-reach, reach)), float(rng.Range(-reach, reach))};
                if (QueryCircle(*scene.world, to, float(scene.extent[i] + 0.1)).empty()) scene.dynamic[i]->SetPosition(to);
            }
            budget += std::max(0.0, Energy(scene) - before);
        }
        if (step == options.steps / 2) {
            SimulationConfig config = scene.world->getSimulationConfig();
            config.enableSleeping = !config.enableSleeping;
            scene.world->setSimulationConfig(config);
        }
        try {
            scene.world->step();
        } catch (const std::exception& error) {
            result.failures.Add(std::string("exception: ") + error.what() + "\n" + where(step));
            return result;
        }
        // An exhausted CCD budget stops every body at the last safe time
        // while velocities keep integrating (collision-lifecycle-and-ccd.md).
        // Once is a documented limit; step after step freezes the world.
        stalled = scene.world->getLastStepStatistics().ccdIterationLimitReached ? stalled + 1 : 0;
        if (stalled == 5) result.failures.Add("CCD impact budget exhausted on 5 consecutive steps\n" + where(step));
        bool finite = true;
        for (std::size_t i = 0; i < scene.dynamic.size(); ++i) {
            const auto& b = *scene.dynamic[i];
            for (float value : {b.position.x, b.position.y, b.orientation, b.velocity.x, b.velocity.y, b.angularVelocity})
                finite = finite && std::isfinite(value);
            // The documented range for a body without CCD: it covers at most
            // half a wall per step (getting-started.md, "Fast bodies").
            const double speed = std::hypot(double(b.velocity.x), double(b.velocity.y));
            if (!scene.bullet[i] && speed * scene.world->getSimulationConfig().fixedTimeStep > kWall / 2)
                fast[i] = true;
            // A centre past the middle of a wall has tunnelled.
            const float limit = kHalf + kWall / 2;
            if (!escaped[i] && std::isfinite(b.position.x) && (std::abs(b.position.x) > limit || std::abs(b.position.y) > limit)) {
                escaped[i] = true;
                if (fast[i]) {
                    ++result.fastEscapes; // beyond the documented speed range, a known limit
                    continue;
                }
                std::ostringstream text;
                text << "escaped the sealed box: body " << i << (scene.bullet[i] ? " (CCD bullet)" : "") << " at ("
                     << b.position.x << ", " << b.position.y << ") velocity (" << b.velocity.x << ", " << b.velocity.y << ")\n"
                     << where(step);
                result.failures.Add(text.str());
            }
        }
        if (!finite) {
            result.failures.Add("non-finite body state\n" + where(step));
            return result;
        }
        const double energy = Energy(scene);
        result.peakEnergyRatio = std::max(result.peakEnergyRatio, energy / budget);
        if (energy > 1.1 * budget + floorEnergy) {
            std::ostringstream text;
            text << "mechanical energy " << energy << " exceeds the injected budget " << budget << "\n" << where(step);
            result.failures.Add(text.str());
            return result;
        }
        result.hashes.push_back(Hash(scene));
    }
    return result;
}

void CheckScenes(std::uint64_t firstSeed, int seeds, int steps) {
    for (std::uint64_t seed = firstSeed; seed < firstSeed + std::uint64_t(seeds); ++seed)
        for (const bool speculative : {true, false}) {
            SceneOptions options;
            options.seed = seed;
            options.speculative = speculative;
            options.steps = steps;
            const RunResult first = RunScene(options);
            const RunResult second = RunScene(options);
            INFO("seed " << seed << (speculative ? " speculative" : " legacy") << " peak energy/budget "
                         << first.peakEnergyRatio << ", escapes above the documented speed " << first.fastEscapes
                         << "\n" << first.failures.Summary());
            CHECK(first.failures.total == 0);
            // Bit-identical repeat runs, step by step.
            std::size_t mismatch = 0;
            while (mismatch < std::min(first.hashes.size(), second.hashes.size())
                   && first.hashes[mismatch] == second.hashes[mismatch]) ++mismatch;
            INFO("first differing step " << mismatch);
            CHECK(first.hashes == second.hashes);
        }
}
} // namespace

TEST_CASE("Random mixed scenes stay finite, bounded, sealed and deterministic", "[fuzz][world]") {
    CheckScenes(20261028, 2, 600);
}

// Fails on CCD bullet escapes, a jointed slow body escaping and repeated CCD
// budget exhaustion; see the known-bug cases below and docs/numerical-validation.md.
TEST_CASE("Random mixed scenes stay finite, bounded, sealed and deterministic (long)", "[.fuzz-long][world]") {
    CheckScenes(1, 12, 1200);
}

// ---- Known World bugs (outside the narrow phase; reproduced, not fixed) ----

TEST_CASE("Known bug: World CCD rejects a valid polygon whose float world outline is not convex",
          "[.known-bug][world][ccd]") {
    // World::advanceCcd rotates the local outline into float world vertices
    // and passes them to SweepCirclePolygon, which validates strict convexity
    // again. Rounding can make a valid outline with two close vertices fail,
    // so step() throws for every CCD circle that can meet this polygon.
    const Polygon polygon({{-0x1.2922fep-5f, 0x1.9f8e10p-5f}, {-0x1.2d1b12p-5f, 0x1.9e559ap-5f},
                           {-0x1.f179dep-5f, 0x1.60a23ep-5f}, {0x1.eff2f8p-5f, 0x1.467130p-4f},
                           {-0x1.1bd1eap-5f, 0x1.a3a5a0p-5f}});
    const Vector2 at{0x1.caap+2f, -0x1.bf7p+2f};
    World world;
    auto target = std::make_shared<RigidBody>(polygon, Material{}, at, true);
    target->SetOrientation(-0x1.5d8b78p-2f);
    auto bullet = std::make_shared<RigidBody>(Circle(0.1f), Material{}, Vector2{at.x, at.y + 3});
    bullet->SetCcdEnabled(true);
    bullet->SetVelocity({0, -100});
    world.addBody(target);
    world.addBody(bullet);
    CHECK_NOTHROW(world.step());
}

TEST_CASE("Known bug: wedged CCD bullets exhaust the impact budget every step and freeze the world",
          "[.known-bug][world][ccd]") {
    // From step 61 every step reports 32 CCD impacts and ccdIterationLimitReached.
    // No body moves, yet gravity keeps integrating velocities: body 62 sits
    // at (7.666, -6.817) while its velocity grows from -9.8 to -12.1 m/s.
    SceneOptions options;
    options.seed = 20261029;
    options.speculative = true;
    options.bulletsHitBodies = true;
    options.steps = 120;
    const RunResult result = RunScene(options);
    INFO(result.failures.Summary());
    CHECK(result.failures.total == 0);
}

TEST_CASE("Known bug: a CCD bullet impact launches a light circle to 1289 m/s",
          "[.known-bug][world][ccd]") {
    // Heavy CCD bullets that also hit bodies, random outlines (mode 1). At
    // step 332 body 113 (Circle 0x1.3fc3a6p-2f, mass 0.076) beside the right
    // wall goes from (38, -149) m/s to (-1289, -87) m/s and 32462 rad/s, past
    // the 200 m/s and 30 rad/s limits; mechanical energy jumps from 2.9e4 to
    // 2.1e6 J. The CCD budget was exhausted on many earlier steps. Every
    // narrow-phase manifold at the start of that step matches the oracle.
    SceneOptions options;
    options.seed = 20261028;
    options.speculative = true;
    options.outlines = 1;
    options.lightBullets = false;
    options.bulletsHitBodies = true;
    options.steps = 333;
    const RunResult result = RunScene(options);
    INFO(result.failures.Summary());
    CHECK(result.failures.total == 0);
}

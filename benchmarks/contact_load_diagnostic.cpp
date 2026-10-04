#include "experimental/contact_load_projection.h"
#include "rigid_contact_metrics.h"
#include <iomanip>
#include <iostream>

using namespace PhysicsEngine;
template <class W>
void Observe(std::ostream &out, const char *name, int steps, bool slide, bool release,
             float angle) {
    SimulationConfig c;
    c.solverIterations = 64;
    c.velocityTolerance = 0;
    c.positionCorrectionFactor = 0;
    c.warmStartFactor = 0;
    c.enableLinearVelocityLimit = false;
    c.enableAngularVelocityLimit = false;
    W world(c);
    const double s = std::sin(double(angle)), co = std::cos(double(angle));
    const Vector2 n{float(-s), float(co)};
    const Material mat{1, 0, slide ? .03f : 1.f, slide ? .03f : 1.f};
    auto floor = std::make_shared<RigidBody>(Polygon::MakeBox(20, 1), mat, n * -.5f, true);
    auto box = std::make_shared<RigidBody>(Polygon::MakeBox(1, 1), mat, n * .5f);
    floor->SetOrientation(angle);
    box->SetOrientation(angle);
    box->SetMass(1);
    if (slide)
        box->SetVelocity({float(-co), float(-s)});
    const auto start = box->position;
    world.addBody(floor);
    world.addBody(box);
    auto gravity = std::make_unique<Gravity>(Vector2{0, -9.81f});
    auto *change = gravity.get();
    world.addUniversalForce(std::move(gravity));
    const float dt = 2.f / steps;
    double peak = 0, spin = 0, loadX = 0, loadY = 0, reactionX = 0, reactionY = 0;
    double unloadedTangent = 0, loadedTangent = 0, reactionSpin = 0, incrementalSpin = 0,
           correctionAngle = 0;
    double incrementalX = 0, incrementalY = 0, correctionX = 0, correctionY = 0, unloaded = 0,
           loaded = 0;
    std::uint64_t pairs = 0, scratches = 0, misses = 0;
    int onset = -1;
    RigidContactDiagnostic::Tracker tracker;
    world.addCollisionListener(&tracker);
    for (int k = 0; k < steps; ++k) {
        if (release && k == steps / 2)
            change->setGravity(n * 9.81f);
        const auto before = box->velocity;
        const auto beforeSpin = box->angularVelocity;
        const Vector2 g = release && k >= steps / 2 ? n * 9.81f : Vector2{0, -9.81f};
        const double fx = double(g.x) * box->mass * box->inverseMass,
                     fy = double(g.y) * box->mass * box->inverseMass;
        world.step(dt);
        loadX += box->mass * fx * dt;
        loadY += box->mass * fy * dt;
        reactionX += box->mass * (double(box->velocity.x) - before.x - fx * dt);
        reactionY += box->mass * (double(box->velocity.y) - before.y - fy * dt);
        reactionSpin += box->inertia * (double(box->angularVelocity) - beforeSpin);
        peak = std::max(peak, std::hypot(double(box->velocity.x), box->velocity.y));
        spin = std::max(spin, std::abs(double(box->angularVelocity)));
        if (world.getPersistentContactCount() && onset < 0)
            onset = k;
        if constexpr (std::is_same<W, ContactLoadExperiment::World>::value) {
            const auto &r = world.lastExperiment();
            pairs += r.pairs;
            scratches += r.scratchSolves;
            if (!r.pairs)
                ++misses;
            unloaded = std::max(unloaded, r.unloadedClosingResidual);
            loaded = std::max(loaded, r.loadedClosingResidual);
            unloadedTangent = std::max(unloadedTangent, r.unloadedTangentialSpeed);
            loadedTangent = std::max(loadedTangent, r.loadedTangentialSpeed);
            if (r.bodies.size() > 1) {
                const auto &b = r.bodies[1];
                incrementalX += b.loadedReactionX - b.unloadedReactionX;
                incrementalY += b.loadedReactionY - b.unloadedReactionY;
                correctionX += b.correctionX;
                correctionY += b.correctionY;
                incrementalSpin += b.loadedReactionSpin - b.unloadedReactionSpin;
                correctionAngle += b.correctionAngle;
            }
        }
    }
    const double dx = double(box->position.x) - start.x, dy = double(box->position.y) - start.y;
    const double downhill = -co * dx - s * dy, normal = -s * dx + co * dy;
    const double acceleration = 9.81f * (s - .03f * co), t = double(dt) * steps;
    for (double v : {downhill, normal, peak, spin, loadX, loadY, reactionX, reactionY, incrementalX,
                     incrementalY, correctionX, correctionY, unloaded, loaded, unloadedTangent,
                     loadedTangent, reactionSpin, incrementalSpin, correctionAngle})
        if (!std::isfinite(v))
            throw std::runtime_error("Nonfinite load diagnostic");
    out << "{\"method\":\"" << name << "\",\"fixture\":\""
        << (release      ? "incline_release"
            : slide      ? "kinetic_slide"
            : angle == 0 ? "flat_rest"
                         : "incline_stick")
        << "\",\"steps\":" << steps << ",\"dt\":" << double(dt) << ",\"duration\":" << t
        << ",\"angle\":" << double(angle) << ",\"downhillDisplacement\":" << downhill
        << ",\"normalDisplacement\":" << normal << ",\"peakSpeed\":" << peak
        << ",\"peakSpin\":" << spin
        << ",\"finalDownhillSpeed\":" << -co * box->velocity.x - s * box->velocity.y
        << ",\"expectedSlideDisplacement\":" << (slide ? t + .5 * acceleration * t * t : 0)
        << ",\"expectedSlideSpeed\":" << (slide ? 1 + acceleration * t : 0)
        << ",\"contactOnsetStep\":" << onset << ",\"eligiblePairVisits\":" << pairs
        << ",\"scratchSolves\":" << scratches << ",\"stepsWithoutEligibleContact\":" << misses
        << ",\"peakUnloadedClosingResidual\":" << unloaded
        << ",\"peakLoadedClosingResidual\":" << loaded << ",\"loadImpulse\":[" << loadX << ','
        << loadY << "],\"actualBodyContactImpulse\":[" << reactionX << ',' << reactionY << ','
        << reactionSpin << "],\"scratchIncrementalReactionImpulse\":[" << incrementalX << ','
        << incrementalY << ',' << incrementalSpin << "],\"summedPositionCorrection\":["
        << correctionX << ',' << correctionY << ',' << correctionAngle
        << "],\"peakUnloadedTangentialSpeed\":" << unloadedTangent
        << ",\"peakLoadedTangentialSpeed\":" << loadedTangent << "],\"events\":[" << tracker.begins
        << ',' << tracker.persists << ',' << tracker.ends << "]}";
    world.removeCollisionListener(&tracker);
}
template <class W> void ObserveStop(std::ostream &out, const char *name, float dt) {
    SimulationConfig c;
    c.solverIterations = 64;
    c.velocityTolerance = 0;
    c.positionCorrectionFactor = 0;
    c.warmStartFactor = 0;
    c.enableLinearVelocityLimit = false;
    c.enableAngularVelocityLimit = false;
    W world(c);
    const Material mat{1, 0, .25f, .25f};
    auto floor = std::make_shared<RigidBody>(Polygon::MakeBox(20, 1), mat, Vector2{0, -.5f}, true);
    auto box = std::make_shared<RigidBody>(Polygon::MakeBox(1, 1), mat, Vector2{0, .499f});
    box->SetMass(1);
    box->SetVelocity({.2f, 0});
    world.addBody(floor);
    world.addBody(box);
    world.addUniversalForce(std::make_unique<Gravity>(Vector2{0, -8}));
    const int steps = int(.5f / dt);
    for (int k = 0; k < steps; ++k)
        world.step(dt);
    const double expected = double(.2f) * .2f / (2 * .25 * 8);
    out << "{\"method\":\"" << name << "\",\"fixture\":\"within_step_stop\",\"dt\":" << double(dt)
        << ",\"steps\":" << steps << ",\"duration\":" << double(dt) * steps
        << ",\"initialSpeed\":" << double(.2f)
        << ",\"gravity\":8,\"friction\":0.25,\"initialCenterY\":" << double(.499f)
        << ",\"expectedStoppingTime\":" << double(.2f) / 2
        << ",\"expectedStoppingDisplacement\":" << expected
        << ",\"measuredDisplacement\":" << double(box->position.x)
        << ",\"displacementError\":" << double(box->position.x) - expected
        << ",\"finalSpeed\":" << std::hypot(double(box->velocity.x), box->velocity.y) << "}";
}
int main() {
    try {
        std::cout << std::setprecision(17)
                  << "{\"schemaVersion\":1,\"classification\":\"observation\","
                     "\"productionDefaultsChanged\":false,\"rows\":[";
        bool first = true;
        for (int steps : {120, 240, 480})
            for (int fixture = 0; fixture < 4; ++fixture) {
                const float angle = fixture ? 3.14159265358979323846f / 12 : 0;
                for (int method = 0; method < 2; ++method) {
                    if (!first)
                        std::cout << ',';
                    first = false;
                    if (method)
                        Observe<ContactLoadExperiment::World>(std::cout, "frozen_load_projection",
                                                              steps, fixture == 2, fixture == 3,
                                                              angle);
                    else
                        Observe<PhysicsEngine::World>(std::cout, "production", steps, fixture == 2,
                                                      fixture == 3, angle);
                }
            }
        for (float dt : {.125f, .0625f, .03125f, .015625f})
            for (int method = 0; method < 2; ++method) {
                std::cout << ',';
                if (method)
                    ObserveStop<ContactLoadExperiment::World>(std::cout, "frozen_load_projection",
                                                              dt);
                else
                    ObserveStop<PhysicsEngine::World>(std::cout, "production", dt);
            }
        std::cout << "]}\n";
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}

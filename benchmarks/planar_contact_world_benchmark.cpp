#include "experimental/planar_contact_bridge.h"
#include "rigid_contact_metrics.h"
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <type_traits>

// Bounded diagnostic of the complete World pipeline. This does not configure
// production World or claim that a many-body support solve is available.
using namespace PhysicsEngine;
namespace {
struct CountingLoad : IForceGenerator {
    unsigned &calls;
    explicit CountingLoad(unsigned &n) : calls(n) {}
    void applyForce(RigidBody *b) override { ++calls; b->ApplyForce({0, -8}); }
};
struct Observation {
    bool experimental = false;
    std::array<double, 6> state{};
    std::uint64_t integrated = 0, mergedCandidates = 0, narrowCandidates = 0;
    std::uint64_t contacts = 0, constraints = 0, iterationCountSum = 0;
    std::uint64_t proposals = 0, selected = 0, closed = 0, cached = 0;
    unsigned forceCalls = 0;
    std::uint64_t begins = 0, persists = 0, ends = 0, featureChanges = 0;
    double elapsed = 0, penetration = 0, peakSpin = 0, supportDrift = 0;
    double workResidual = 0, momentumResidual = 0, torqueResidual = 0;
    double normalImpulse = 0, tangentImpulse = 0, externalWork = 0, frictionWork = 0;
    std::map<std::string, std::uint64_t> eligibility;
};
template<class W>
Observation Run(const std::string &fixture, float dt, int steps, int iterations) {
    SimulationConfig c;
    c.solverIterations = iterations;
    c.warmStartFactor = 0;
    c.enableSleeping = false;
    c.enableLinearVelocityLimit = false;
    c.enableAngularVelocityLimit = false;
    W w(c);
    RigidContactDiagnostic::Tracker tracker;
    w.addCollisionListener(&tracker);
    const Material m{1, 0, .25f, .25f};
    // Alternate creation order across signs to exercise canonical identities.
    const bool reverse = fixture.find("negative") != std::string::npos;
    RigidBodyPtr floor, body;
    auto makeBody = [&] {
        body = std::make_shared<RigidBody>(Polygon::MakeBox(1, 1), m, Vector2{0, .5f});
        body->SetMass(1);
    };
    if (reverse) makeBody();
    floor = std::make_shared<RigidBody>(Polygon::MakeBox(20, 1), m, Vector2{0, -.5f}, true);
    if (!reverse) makeBody();
    w.addBody(floor); w.addBody(body);
    const float sign = reverse ? -1.f : 1.f;
    if (fixture.find("slide") == 0) body->SetVelocity({sign * 2, 0});
    if (fixture.find("stop") == 0 || fixture.find("reversal") == 0)
        body->SetVelocity({sign * .2f, 0});
    if (fixture == "impact_fallback") { body->SetPosition({0, .5625f}); body->SetVelocity({0, -2}); }
    if (fixture == "extent_fallback") { body->SetPosition({9.5f, .5f}); body->SetVelocity({.2f, 0}); }
    Observation r;
    r.experimental = std::is_same_v<W, PlanarContactBridge::World>;
    Gravity *gravity = nullptr;
    if (fixture == "opaque_fallback")
        w.addForce(body, std::make_unique<CountingLoad>(r.forceCalls));
    else {
        auto load = std::make_unique<Gravity>(Vector2{0, -8});
        gravity = load.get();
        w.addUniversalForce(std::move(load));
    }
    for (int step = 0; step < steps; ++step) {
        if (fixture.find("reversal") == 0) body->ApplyForce({-sign * 3, 0});
        if (fixture == "ramp") body->ApplyForce({step < steps / 3 ? 1.f : step < 2 * steps / 3 ? 2.f : 3.f, 0});
        if (fixture == "tipping_fallback") body->ApplyTorque(8);
        if (fixture == "release" && step == steps / 2) gravity->setGravity({0, 8});
        const auto before = body->velocity;
        const auto start = std::chrono::steady_clock::now();
        w.step(dt);
        r.elapsed += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
        const auto &s = w.getLastStepStatistics();
        r.integrated += s.integratedBodyCount;
        r.mergedCandidates += s.broadPhaseCandidateCount;
        r.narrowCandidates += s.narrowPhaseCandidateCount;
        r.contacts += s.resolvedContactCount;
        r.constraints += s.solvedConstraintCount;
        r.iterationCountSum += s.solverIterationCount;
        r.penetration = std::max(r.penetration, RigidContactDiagnostic::Penetration(*floor, *body));
        r.peakSpin = std::max(r.peakSpin, std::abs(double(body->angularVelocity)));
        const auto cache = Detail::ExperimentalWorldStep::Cache(w, *floor, *body);
        r.cached += bool(cache);
        double n = 0, t = 0;
        if (cache) for (unsigned i = 0; i < cache->contactCount; ++i) {
            n += cache->contacts[i].impulse.normal;
            t -= cache->contacts[i].impulse.tangent; // Physical +x impulse on the dynamic body.
        }
        r.normalImpulse += n; r.tangentImpulse += t;
        if constexpr (std::is_same_v<W, PlanarContactBridge::World>) {
            const auto &e = w.lastExperiment();
            ++r.eligibility[e.eligibility];
            r.proposals += e.proposed;
            if (e.selected) {
                ++r.selected;
                r.closed += e.eligibility == "planar_support";
                r.externalWork += e.interval.externalWork;
                r.frictionWork += e.interval.frictionWork;
                const double kinetic = .5 * ((double(body->velocity.x) - before.x) *
                                             (double(body->velocity.x) + before.x) +
                                             (double(body->velocity.y) - before.y) *
                                             (double(body->velocity.y) + before.y));
                r.workResidual = std::max(r.workResidual, std::abs(kinetic - e.interval.externalWork - e.interval.frictionWork));
                r.momentumResidual = std::max(r.momentumResidual,
                    std::hypot(double(body->velocity.x) - before.x - double(e.appliedForce.x) * dt - t,
                               double(body->velocity.y) - before.y - double(e.appliedForce.y) * dt - n));
                r.torqueResidual = std::max(r.torqueResidual, std::abs(e.reactionTorqueResidual));
            }
        }
        if (fixture != "release" && fixture != "impact_fallback")
            r.supportDrift = std::max(r.supportDrift, std::abs(double(body->position.y) - .5));
    }
    r.state = {body->position.x, body->position.y, body->orientation,
               body->velocity.x, body->velocity.y, body->angularVelocity};
    r.begins = tracker.begins; r.persists = tracker.persists; r.ends = tracker.ends;
    r.featureChanges = tracker.featureChanges;
    w.removeCollisionListener(&tracker);
    return r;
}
void Write(std::ostream &out, const Observation &r) {
    out << "{\"finalState\":[";
    for (unsigned i = 0; i < r.state.size(); ++i) { if (i) out << ','; out << r.state[i]; }
    out << "],\"integratedBodies\":" << r.integrated
        << ",\"mergedBroadPhaseCandidates\":" << r.mergedCandidates
        << ",\"narrowPhaseCandidates\":" << r.narrowCandidates
        << ",\"resolvedContacts\":" << r.contacts << ",\"solvedConstraints\":" << r.constraints
        << ",\"solverIterationCountSum\":" << r.iterationCountSum
        << ",\"proposedSteps\":" << r.proposals << ",\"selectedSteps\":" << r.selected
        << ",\"closedSteps\":" << r.closed << ",\"cachedSteps\":" << r.cached
        << ",\"opaqueForceCalls\":" << r.forceCalls
        << ",\"contactBegins\":" << r.begins << ",\"contactPersists\":" << r.persists
        << ",\"contactEnds\":" << r.ends << ",\"featureChanges\":" << r.featureChanges
        << ",\"peakPenetration\":" << r.penetration << ",\"peakAngularSpeed\":" << r.peakSpin
        << ",\"peakSupportDrift\":" << r.supportDrift
        << ",\"normalImpulse\":" << r.normalImpulse << ",\"tangentImpulse\":" << r.tangentImpulse
        << ",\"acceptedIntervalAccounting\":";
    if (r.experimental)
        out << "{\"peakStoredStateWorkResidual\":" << r.workResidual
            << ",\"peakStoredStateMomentumResidual\":" << r.momentumResidual
            << ",\"peakReactionTorqueResidual\":" << r.torqueResidual
            << ",\"externalWork\":" << r.externalWork << ",\"frictionWork\":" << r.frictionWork << '}';
    else out << "null";
    out << ",\"worldStepMilliseconds\":" << r.elapsed << ",\"eligibility\":{";
    bool first = true;
    for (const auto &e : r.eligibility) { if (!first) out << ','; first = false; out << '"' << e.first << "\":" << e.second; }
    out << "}}";
}
} // namespace
int main(int argc, char **argv) {
    try {
        bool quick = false; std::string path;
        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            if (arg == "--quick") quick = true;
            else if (arg == "--output" && i + 1 < argc) path = argv[++i];
            else if (arg == "--help") {
                std::cout << "planar_contact_world_benchmark [--quick] [--output path]\n"; return 0;
            } else throw std::invalid_argument("Unknown or incomplete option");
        }
        const std::vector<std::string> fixtures{"rest", "slide_positive", "slide_negative", "stop_positive", "stop_negative",
            "reversal_positive", "reversal_negative", "ramp", "release", "impact_fallback", "tipping_fallback", "extent_fallback", "opaque_fallback"};
        const std::vector<float> dts = quick ? std::vector<float>{1.f / 64} : std::vector<float>{1.f / 64, 1.f / 128};
        const std::vector<int> iterations = quick ? std::vector<int>{4} : std::vector<int>{4, 10};
        const double duration = quick ? .5 : 2;
        std::uint64_t planned = 0;
        for (float dt : dts) planned += fixtures.size() * iterations.size() * 2 * static_cast<unsigned>(duration / dt);
        if (planned > 250000) throw std::invalid_argument("World-step work budget exceeded");
        std::ofstream file;
        if (!path.empty()) { file.open(path); if (!file) throw std::runtime_error("Cannot open output"); }
        auto &out = path.empty() ? std::cout : file;
        out << std::setprecision(17) << "{\"schemaVersion\":1,\"experimentalIntegration\":\"staged_planar_world_interval\","
            << "\"productionConfiguration\":false,\"timingIsDeterministic\":false,\"timingScope\":\"complete_World_step_including_events\","
            << "\"warmStart\":0,\"plannedWorldSteps\":" << planned << ",\"rows\":[";
        bool first = true;
        for (float dt : dts) for (int n : iterations) for (const auto &f : fixtures) {
            const int steps = static_cast<int>(duration / dt);
            const auto b = Run<PhysicsEngine::World>(f, dt, steps, n);
            const auto e = Run<PlanarContactBridge::World>(f, dt, steps, n);
            if (!first) out << ','; first = false;
            out << "{\"fixture\":\"" << f << "\",\"dt\":" << dt << ",\"iterations\":" << n
                << ",\"stepsPerPipeline\":" << steps << ",\"baseline\":";
            Write(out, b); out << ",\"experimental\":"; Write(out, e); out << '}';
        }
        out << "],\"executionFailures\":0}\n"; out.flush();
        if (!out) throw std::runtime_error("Output write failed");
        return 0;
    } catch (const std::exception &e) { std::cerr << "planar_contact_world_benchmark: " << e.what() << '\n'; return 1; }
}

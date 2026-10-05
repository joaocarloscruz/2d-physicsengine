#include "../benchmarks/experimental/planar_contact_bridge.h"
#include "../benchmarks/rigid_contact_metrics.h"
#include "catch_amalgamated.hpp"
using namespace PhysicsEngine;
namespace {
SimulationConfig Config() {
    SimulationConfig c;
    c.warmStartFactor = 0;
    c.enableLinearVelocityLimit = false;
    c.enableAngularVelocityLimit = false;
    c.enableSleeping = false;
    c.solverIterations = 4;
    return c;
}
struct Pair {
    RigidBodyPtr floor, body;
};
template <class W> Pair Add(W &w, float mu = .25f, bool reverse = false) {
    Pair p;
    const Material material{1, 0, mu, mu};
    auto make = [](Material m) {
        return std::make_shared<RigidBody>(Polygon::MakeBox(1, 1), m, Vector2{0, .5f});
    };
    if (reverse)
        p.body = make(material);
    p.floor =
        std::make_shared<RigidBody>(Polygon::MakeBox(20, 1), material, Vector2{0, -.5f}, true);
    if (!reverse)
        p.body = make(material);
    p.body->SetMass(1);
    w.addBody(p.floor);
    w.addBody(p.body);
    return p;
}
void Near(double x, double y, double margin = 2e-7) {
    REQUIRE(x == Catch::Approx(y).epsilon(0).margin(margin));
}
void Equal(const RigidBody &a, const RigidBody &b) {
    REQUIRE(a.position == b.position);
    REQUIRE(a.velocity == b.velocity);
    REQUIRE(a.orientation == b.orientation);
    REQUIRE(a.angularVelocity == b.angularVelocity);
    REQUIRE(a.force == b.force);
    REQUIRE(a.torque == b.torque);
}
struct Observer : ICollisionListener {
    RigidBodyPtr body;
    unsigned begins = 0, persists = 0, ends = 0, frames = 0;
    CollisionManifold last;
    void onCollisionBegin(const CollisionEvent &e) override {
        ++begins;
        REQUIRE(e.penetration == 0);
    }
    void onCollisionPersist(const CollisionEvent &e) override {
        ++persists;
        REQUIRE(e.penetration == 0);
    }
    void onCollisionEnd(const CollisionEvent &) override { ++ends; }
    void onCollision(const CollisionManifold &m) override {
        ++frames;
        last = m;
        REQUIRE(body->position.y == .5f);
        REQUIRE(body->velocity.y == 0);
        REQUIRE(m.contactCount == 2);
        REQUIRE(m.contacts[0].position.y == 0);
        REQUIRE(m.contacts[1].position.y == 0);
    }
};
} // namespace
TEST_CASE("Planar bridge closed touch lifecycle and reaction accounting are real",
          "[contact-bridge]") {
    PlanarContactBridge::World w(Config());
    const auto p = Add(w);
    REQUIRE_FALSE(CheckCollision(p.floor.get(), p.body.get()).hasCollision);
    auto gravity = std::make_unique<Gravity>(Vector2{0, -8});
    auto *change = gravity.get();
    w.addUniversalForce(std::move(gravity));
    Observer o;
    o.body = p.body;
    w.addCollisionListener(&o);
    for (unsigned i = 0; i < 3; ++i) {
        w.step(.125f);
        REQUIRE(w.lastExperiment().selected);
        REQUIRE(p.body->position == Vector2{0, .5f});
        REQUIRE(p.body->velocity == Vector2{});
        REQUIRE(p.body->force == Vector2{});
        REQUIRE(p.body->torque == 0);
        const auto j = Detail::ExperimentalWorldStep::Cache(w, *p.floor, *p.body);
        REQUIRE(j);
        Near(j->contacts[0].impulse.normal + j->contacts[1].impulse.normal, 1, 1e-14);
        REQUIRE(w.getPersistentContactCount() == 1);
        const auto s = w.getLastStepStatistics();
        REQUIRE(s.integratedBodyCount == 1);
        REQUIRE(s.resolvedContactCount == 2);
        REQUIRE(s.solvedConstraintCount == 1);
        REQUIRE(s.islandCount == 1);
        REQUIRE(s.solvedIslandCount == 1);
        REQUIRE(s.solverIterationCount == 0);
    }
    REQUIRE(o.begins == 1);
    REQUIRE(o.persists == 2);
    REQUIRE(o.frames == 3);
    change->setGravity({0, 8});
    w.step(.125f);
    REQUIRE(w.lastExperiment().eligibility == "planar_free");
    REQUIRE(o.ends == 1);
    REQUIRE(o.frames == 3);
    REQUIRE_FALSE(Detail::ExperimentalWorldStep::Cache(w, *p.floor, *p.body));
    REQUIRE(w.getPersistentContactCount() == 0);
    REQUIRE(w.getLastStepStatistics().solvedConstraintCount == 0);
    REQUIRE(p.body->position.y == .5625f);
    REQUIRE(p.body->velocity.y == 1);
    w.step(.125f);
    REQUIRE(p.body->position.y == .75f);
    REQUIRE(p.body->velocity.y == 2);
    REQUIRE(o.ends == 1);
    w.removeCollisionListener(&o);
}
TEST_CASE("Planar bridge stop respects canonical body ordering and sliding signs",
          "[contact-bridge]") {
    for (bool reverse : {false, true})
        for (float sign : {-1.f, 1.f}) {
            PlanarContactBridge::World w(Config());
            const auto p = Add(w, .25f, reverse);
            p.body->SetVelocity({sign * .2f, 0});
            w.addUniversalForce(std::make_unique<Gravity>(Vector2{0, -8}));
            Observer o;
            o.body = p.body;
            w.addCollisionListener(&o);
            w.step(.125f);
            REQUIRE(w.lastExperiment().selected);
            const double initial = double(.2f);
            Near(p.body->position.x, sign * initial * initial / 4, 1e-9);
            REQUIRE(p.body->velocity.x == 0);
            REQUIRE(p.body->angularVelocity == 0);
            REQUIRE(p.body->orientation == 0);
            const auto j = Detail::ExperimentalWorldStep::Cache(w, *p.floor, *p.body);
            REQUIRE(j);
            Near(j->contacts[0].impulse.tangent + j->contacts[1].impulse.tangent, sign * initial,
                 1e-14);
            Near(j->contacts[0].impulse.normal + j->contacts[1].impulse.normal, 1, 1e-14);
            REQUIRE(j->contacts[0].featureId == 0x7a000001u);
            REQUIRE(j->contacts[1].featureId == 0x7a000002u);
            REQUIRE(o.last.A->GetId() < o.last.B->GetId());
            REQUIRE(o.last.normal.y == (reverse ? -1.f : 1.f));
            const auto &r = w.lastExperiment().interval;
            Near(r.kineticChange, r.externalWork + r.frictionWork, 1e-14);
            REQUIRE(r.frictionWork <= 0);
            REQUIRE(o.begins == 1);
            w.removeCollisionListener(&o);
        }
}
TEST_CASE("Planar bridge registered and one-shot manual loads are consumed once",
          "[contact-bridge]") {
    PlanarContactBridge::World w(Config());
    const auto p = Add(w);
    w.addUniversalForce(std::make_unique<Gravity>(Vector2{0, -8}));
    p.body->ApplyForce({8, 0});
    p.body->ApplyTorque(.5f);
    w.step(.125f);
    REQUIRE(w.lastExperiment().appliedForce == Vector2{8, -8});
    REQUIRE(w.lastExperiment().appliedTorque == .5f);
    REQUIRE(p.body->position.x == .046875f);
    REQUIRE(p.body->velocity.x == .75f);
    REQUIRE(p.body->angularVelocity == 0);
    REQUIRE(p.body->force == Vector2{});
    REQUIRE(p.body->torque == 0);
    w.step(.125f);
    REQUIRE(w.lastExperiment().appliedForce == Vector2{0, -8});
    REQUIRE(w.lastExperiment().appliedTorque == 0);
    REQUIRE(p.body->position.x == .125f);
    REQUIRE(p.body->velocity.x == .5f);
    REQUIRE(p.body->velocity.y == 0);
    const auto j = Detail::ExperimentalWorldStep::Cache(w, *p.floor, *p.body);
    Near(j->contacts[0].impulse.normal + j->contacts[1].impulse.normal, 1, 1e-14);
    Near(j->contacts[0].impulse.tangent + j->contacts[1].impulse.tangent, .25, 1e-14);
}
TEST_CASE("Planar bridge static threshold ramp and signed restart", "[contact-bridge]") {
    PlanarContactBridge::World w(Config());
    const auto p = Add(w);
    w.addUniversalForce(std::make_unique<Gravity>(Vector2{0, -8}));
    for (float force : {0.f, 1.f, 2.f}) {
        p.body->ApplyForce({force, 0});
        w.step(.125f);
        REQUIRE(w.lastExperiment().selected);
        REQUIRE(p.body->position.x == 0);
        REQUIRE(p.body->velocity.x == 0);
        REQUIRE(w.lastExperiment().interval.intervals[0].mode ==
                PlanarContactInterval::Mode::Sticking);
    }
    p.body->ApplyForce({std::nextafter(2.f, 3.f), 0});
    w.step(.125f);
    REQUIRE(w.lastExperiment().selected);
    REQUIRE(p.body->position.x > 0);
    REQUIRE(p.body->velocity.x > 0);
    for (float sign : {-1.f, 1.f}) {
        PlanarContactBridge::World restart(Config());
        const auto q = Add(restart);
        q.body->SetVelocity({sign * .2f, 0});
        restart.addUniversalForce(std::make_unique<Gravity>(Vector2{0, -8}));
        q.body->ApplyForce({-sign * 3, 0});
        restart.step(.125f);
        REQUIRE(restart.lastExperiment().selected);
        const double stop = double(.2f) / 5, remain = .125 - stop;
        Near(q.body->velocity.x, -sign * remain, 1e-8);
        Near(q.body->position.x,
             sign * (double(.2f) * stop - 2.5 * stop * stop - .5 * remain * remain), 1e-9);
        const auto &r = restart.lastExperiment().interval;
        REQUIRE(r.intervals.size() == 2);
        Near(r.kineticChange, r.externalWork + r.frictionWork, 1e-14);
    }
}
namespace {
struct CountForce : IForceGenerator {
    unsigned &calls;
    explicit CountForce(unsigned &n) : calls(n) {}
    void applyForce(RigidBody *b) override {
        ++calls;
        b->ApplyForce({0, -8});
    }
};
} // namespace
TEST_CASE("Planar bridge whole-step fallback for impact tipping and opaque loads",
          "[contact-bridge]") {
    for (unsigned mode : {0u, 1u, 2u}) {
        PlanarContactBridge::World e(Config());
        PhysicsEngine::World b(Config());
        const auto ep = Add(e), bp = Add(b);
        unsigned ec = 0, bc = 0;
        if (mode == 0) {
            ep.body->SetVelocity({0, -2});
            bp.body->SetVelocity({0, -2});
        }
        if (mode == 1) {
            ep.body->ApplyTorque(8);
            bp.body->ApplyTorque(8);
        }
        if (mode == 2) {
            e.addForce(ep.body, std::make_unique<CountForce>(ec));
            b.addForce(bp.body, std::make_unique<CountForce>(bc));
        } else {
            e.addUniversalForce(std::make_unique<Gravity>(Vector2{0, -8}));
            b.addUniversalForce(std::make_unique<Gravity>(Vector2{0, -8}));
        }
        e.step(.125f);
        b.step(.125f);
        REQUIRE_FALSE(e.lastExperiment().selected);
        REQUIRE(e.lastExperiment().eligibility == (mode == 0   ? "needs_impact"
                                                   : mode == 1 ? "unsupported_wrench"
                                                               : "opaque_force"));
        Equal(*ep.body, *bp.body);
        Equal(*ep.floor, *bp.floor);
        REQUIRE(ec == bc);
        if (mode == 2)
            REQUIRE(ec == 1);
        REQUIRE(e.getPersistentContactCount() == b.getPersistentContactCount());
        REQUIRE(e.getLastStepStatistics().solverIterationCount ==
                b.getLastStepStatistics().solverIterationCount);
    }
}
TEST_CASE("Planar bridge stacks and warmed fixtures remain baseline", "[contact-bridge]") {
    for (const auto &f : RigidContactDiagnostic::Fixtures(true)) {
        const RigidContactDiagnostic::Controls c{1.f / 60, 4, .8f, .1};
        const auto b = RigidContactDiagnostic::Run(f, c),
                   e = RigidContactDiagnostic::Run<PlanarContactBridge::World>(f, c);
        REQUIRE(e.states == b.states);
        REQUIRE(e.begins == b.begins);
        REQUIRE(e.persists == b.persists);
        REQUIRE(e.ends == b.ends);
        REQUIRE(e.constraints == b.constraints);
        REQUIRE(e.solverIterations == b.solverIterations);
    }
    for (const auto &f : RigidContactDiagnostic::Fixtures(false))
        if (f.boxes > 1) {
            const RigidContactDiagnostic::Controls c{1.f / 60, 4, 0, .1};
            const auto b = RigidContactDiagnostic::Run(f, c),
                       e = RigidContactDiagnostic::Run<PlanarContactBridge::World>(f, c);
            REQUIRE(e.states == b.states);
            REQUIRE(e.begins == b.begins);
            REQUIRE(e.persists == b.persists);
            REQUIRE(e.ends == b.ends);
        }
}
TEST_CASE("Planar bridge covers the complete support interval and exact represented touch",
          "[contact-bridge]") {
    for (unsigned mode : {0u, 1u, 2u, 3u, 4u, 5u, 6u}) {
        PlanarContactBridge::World e(Config());
        PhysicsEngine::World b(Config());
        const auto ep = Add(e), bp = Add(b);
        for (auto p : {ep, bp}) {
            if (mode == 0) // Exact edge containment is supported.
                p.body->SetPosition({9.5f, .5f});
            if (mode == 1) // Endpoints are inside; interior free extremum is outside.
                p.body->SetPosition({9.49f, .5f});
            if (mode == 1) {
                p.body->SetVelocity({1, 0});
                p.body->ApplyForce({-16, 8});
            }
            if (mode == 2) {
                p.body->SetPosition({9.495f, .5f});
                p.body->SetVelocity({.2f, 0});
            }
            if (mode == 3)
                p.body->SetPosition({0, std::nextafter(.5f, 0.f)});
            if (mode == 4)
                p.body->SetPosition({0, std::nextafter(.5f, 1.f)});
            if (mode == 5) { // Float storage collapses distinct endpoint corners.
                p.floor->SetPosition({1e8f, -.5f});
                p.body->SetPosition({1e8f, .5f});
            }
            if (mode == 6) { // Adding a local half-height disappears at this magnitude.
                p.floor->SetPosition({0, 1e30f});
                p.body->SetPosition({0, 1e30f});
            }
        }
        if (mode != 1) {
            e.addUniversalForce(std::make_unique<Gravity>(Vector2{0, -8}));
            b.addUniversalForce(std::make_unique<Gravity>(Vector2{0, -8}));
        }
        e.step(.125f);
        b.step(.125f);
        REQUIRE(e.lastExperiment().selected == (mode == 0));
        if (mode == 0) {
            REQUIRE(ep.body->position == Vector2{9.5f, .5f});
            continue;
        }
        REQUIRE(e.lastExperiment().eligibility ==
                (mode < 3 ? "support_extent" : mode == 3 ? "overlap" :
                 mode == 4 ? "needs_impact" : "float_geometry"));
        Equal(*ep.body, *bp.body);
        REQUIRE(e.getLastStepStatistics().solverIterationCount ==
                b.getLastStepStatistics().solverIterationCount);
    }
}
TEST_CASE("Planar bridge mixes distinct materials and rejects reversed friction ordering",
          "[contact-bridge]") {
    PlanarContactBridge::World e(Config());
    const auto p = Add(e);
    p.floor->material.staticFriction = .5f;
    p.floor->material.dynamicFriction = .125f;
    p.body->material.staticFriction = .125f;
    p.body->material.dynamicFriction = .5f;
    p.body->ApplyForce({2, -8});
    e.step(.125f);
    REQUIRE(e.lastExperiment().selected);
    REQUIRE(p.body->velocity == Vector2{});
    p.floor->material.staticFriction = .125f;
    p.body->ApplyForce({0, -8});
    e.step(.125f);
    REQUIRE_FALSE(e.lastExperiment().selected);
    REQUIRE_FALSE(e.lastExperiment().proposed);
}
namespace {
struct InvalidIdentity : Detail::IntegrationStrategy {
    bool stage(const PhysicsEngine::World &w, float, Detail::IntegrationPlan &p) override {
        p.body = w.getBodies()[1].get();
        p.bodyId = p.body->GetId() + 1;
        p.startPosition = p.body->position;
        p.startVelocity = p.body->velocity;
        p.startForce = p.body->force;
        p.startTorque = p.body->torque;
        p.position = {500, 500};
        return true;
    }
};
} // namespace
TEST_CASE("Planar bridge publication rejects invalid identity and never claims invalid properties",
          "[contact-bridge]") {
    PhysicsEngine::World e(Config()), b(Config());
    const auto ep = Add(e), bp = Add(b);
    ep.body->ApplyForce({1, -8});
    bp.body->ApplyForce({1, -8});
    InvalidIdentity strategy;
    REQUIRE_FALSE(Detail::ExperimentalWorldStep::Step(e, .125f, strategy));
    b.step(.125f);
    Equal(*ep.body, *bp.body);
    REQUIRE(ep.body->force == Vector2{});
    PlanarContactBridge::World invalid(Config());
    const auto ip = Add(invalid);
    ip.body->ApplyForce({0, -8});
    ip.body->inverseMass = 0;
    REQUIRE_THROWS_AS(invalid.step(.125f), std::invalid_argument);
    REQUIRE(invalid.lastExperiment().proposed);
    REQUIRE_FALSE(invalid.lastExperiment().selected);
    REQUIRE(ip.body->position == Vector2{0, .5f});
    REQUIRE(ip.body->force == Vector2{0, -8});
}
TEST_CASE("Planar bridge excludes unrelated solver policies and components", "[contact-bridge]") {
    for (unsigned mode : {0u, 1u, 2u, 3u, 4u, 5u}) {
        auto config = Config();
        if (mode == 0) config.enableSleeping = true;
        if (mode == 1) config.enableLinearVelocityLimit = true;
        if (mode == 2) config.enableAngularVelocityLimit = true;
        PlanarContactBridge::World e(config);
        PhysicsEngine::World b(config);
        const auto ep = Add(e), bp = Add(b);
        if (mode == 3) { ep.body->SetCcdEnabled(true); bp.body->SetCcdEnabled(true); }
        if (mode == 4) {
            e.addJoint(std::make_shared<DistanceJoint>(ep.floor, ep.body, 1));
            b.addJoint(std::make_shared<DistanceJoint>(bp.floor, bp.body, 1));
        }
        if (mode == 5) {
            for (PhysicsEngine::World *w : {static_cast<PhysicsEngine::World *>(&e), &b}) {
                auto particles = std::make_shared<ParticleSystem>();
                particles->addParticle({1, 1}, {1, 0});
                w->addParticleSystem(particles);
            }
        }
        e.addUniversalForce(std::make_unique<Gravity>(Vector2{0, -8}));
        b.addUniversalForce(std::make_unique<Gravity>(Vector2{0, -8}));
        e.step(.125f); b.step(.125f);
        REQUIRE_FALSE(e.lastExperiment().selected);
        REQUIRE(e.lastExperiment().eligibility ==
                (mode == 0 ? "sleeping" : mode < 3 ? "velocity_caps" :
                 mode == 3 ? "ccd" : mode == 4 ? "joints" : "particles"));
        Equal(*ep.body, *bp.body);
        REQUIRE(e.getLastStepStatistics().solverIterationCount == b.getLastStepStatistics().solverIterationCount);
    }
}
TEST_CASE("Planar bridge range rejection follows the complete ordinary exception path",
          "[contact-bridge]") {
    PlanarContactBridge::World e(Config());
    PhysicsEngine::World b(Config());
    RigidBodyPtr eb, bb;
    const float maximum = std::numeric_limits<float>::max();
    const Polygon huge({{-.5f, -maximum}, {.5f, -maximum}, {.5f, maximum}, {-.5f, maximum}});
    const Material material{1, 0, .25f, .25f};
    for (PhysicsEngine::World *w : {static_cast<PhysicsEngine::World *>(&e), &b}) {
        w->addBody(std::make_shared<RigidBody>(huge, material, Vector2{0, maximum}, true));
        auto body = std::make_shared<RigidBody>(Polygon::MakeBox(1, 1), material, Vector2{0, .5f});
        body->SetMass(1);
        body->ApplyForce({1, -8});
        w->addBody(body);
        if (w == &e) eb = body; else bb = body;
    }
    REQUIRE_THROWS_AS(e.step(.125f), std::overflow_error);
    REQUIRE_THROWS_AS(b.step(.125f), std::overflow_error);
    REQUIRE_FALSE(e.lastExperiment().selected);
    REQUIRE(e.lastExperiment().eligibility == "derived_range");
    Equal(*eb, *bb);
    REQUIRE(eb->position == Vector2{.0078125f, .4375f});
    REQUIRE(eb->force == Vector2{});
}

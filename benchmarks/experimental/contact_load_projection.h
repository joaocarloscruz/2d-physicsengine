#pragma once

// Diagnostic-only adapter. Never installed or used by the production World.
#include "physics/physics.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <string>

namespace ContactLoadExperiment {
using namespace PhysicsEngine;
struct Response {
    double loadX = 0, loadY = 0, loadSpin = 0;
    double unloadedReactionX = 0, unloadedReactionY = 0, unloadedReactionSpin = 0;
    double loadedReactionX = 0, loadedReactionY = 0, loadedReactionSpin = 0;
    double correctionX = 0, correctionY = 0, correctionAngle = 0;
};
struct Report {
    std::string eligibility = "not_run";
    std::size_t pairs = 0, scratchSolves = 0;
    double unloadedClosingResidual = 0, loadedClosingResidual = 0;
    double unloadedTangentialSpeed = 0, loadedTangentialSpeed = 0;
    std::vector<Response> bodies;
};

class World : public PhysicsEngine::World {
    using Pair = std::pair<std::size_t, std::size_t>;
    std::set<std::pair<std::uint64_t, std::uint64_t>> previous;
    Report report;
    static float Float(double x) {
        if (!std::isfinite(x) || std::abs(x) > std::numeric_limits<float>::max())
            throw std::overflow_error("Experimental load projection is unrepresentable");
        return static_cast<float>(x);
    }
    struct PrescribedPairs : IBroadPhase {
        std::vector<Pair> pairs;
        explicit PrescribedPairs(std::vector<Pair> p) : pairs(std::move(p)) {}
        std::vector<CollisionPair>
        FindPotentialCollisions(const std::vector<RigidBodyPtr> &b) override {
            std::vector<CollisionPair> result;
            for (auto p : pairs)
                result.emplace_back(b[p.first], b[p.second]);
            return result;
        }
    };
    static RigidBodyPtr Clone(const RigidBody &b) {
        auto copy = std::make_shared<RigidBody>(*b.shape, b.material, b.position, b.IsStatic());
        copy->orientation = b.orientation;
        copy->velocity = b.velocity;
        copy->angularVelocity = b.angularVelocity;
        copy->mass = b.mass;
        copy->inverseMass = b.inverseMass;
        copy->inertia = b.inertia;
        copy->inverseInertia = b.inverseInertia;
        copy->SetCollisionCategoryBits(b.GetCollisionCategoryBits());
        copy->SetCollisionMaskBits(b.GetCollisionMaskBits());
        return copy;
    }
    static double Closing(const CollisionManifold &m) {
        double worst = 0;
        for (unsigned k = 0; k < m.contactCount; ++k) {
            const auto p = m.contacts[k].position;
            const auto &a = *m.A;
            const auto &b = *m.B;
            const double vx = double(b.velocity.x) - a.velocity.x -
                              b.angularVelocity * (double(p.y) - b.position.y) +
                              a.angularVelocity * (double(p.y) - a.position.y);
            const double vy = double(b.velocity.y) - a.velocity.y +
                              b.angularVelocity * (double(p.x) - b.position.x) -
                              a.angularVelocity * (double(p.x) - a.position.x);
            worst = std::max(worst, -(vx * m.normal.x + vy * m.normal.y));
        }
        return worst;
    }
    static double Tangential(const CollisionManifold &m) {
        double worst = 0;
        for (unsigned k = 0; k < m.contactCount; ++k) {
            const auto p = m.contacts[k].position;
            const auto &a = *m.A;
            const auto &b = *m.B;
            const double vx = double(b.velocity.x) - a.velocity.x -
                              b.angularVelocity * (double(p.y) - b.position.y) +
                              a.angularVelocity * (double(p.y) - a.position.y);
            const double vy = double(b.velocity.y) - a.velocity.y +
                              b.angularVelocity * (double(p.x) - b.position.x) -
                              a.angularVelocity * (double(p.x) - a.position.x);
            worst = std::max(worst, std::abs(-vx * m.normal.y + vy * m.normal.x));
        }
        return worst;
    }
    std::string Unsupported() const {
        const auto &c = getSimulationConfig();
        if (getBodies().size() > 32 || c.solverIterations > 64)
            return "budget";
        if (c.enableSleeping)
            return "sleeping";
        if (c.enableLinearVelocityLimit || c.enableAngularVelocityLimit)
            return "velocity_caps";
        if (!getJoints().empty())
            return "joints";
        for (const auto &b : getBodies()) {
            if (b->IsCcdEnabled())
                return "ccd";
            if (b->IsStatic() && (!(b->velocity == Vector2{}) || b->angularVelocity != 0))
                return "moving_support";
            if (!b->IsStatic() && !b->IsAwake())
                return "sleeping_body";
        }
        for (const auto &f : getForceRegistry()) {
            const auto *generator = f.generator.get();
            if (typeid(*generator) != typeid(Gravity))
                return "opaque_force";
        }
        for (const auto &f : getUniversalForceRegistry()) {
            const auto *generator = f.get();
            if (typeid(*generator) != typeid(Gravity))
                return "opaque_force";
        }
        return {};
    }
    struct State {
        double vx, vy, spin;
    };
    std::vector<State> Project(const std::vector<Pair> &pairs, const std::vector<State> &loads,
                               float dt, bool loaded, double &residual, double &tangential) {
        auto config = getSimulationConfig();
        config.positionCorrectionFactor = 0;
        config.warmStartFactor = 0;
        config.enableSleeping = false;
        PhysicsEngine::World scratch(config);
        scratch.setBroadPhase(std::make_unique<PrescribedPairs>(pairs));
        std::vector<RigidBodyPtr> copies;
        for (std::size_t i = 0; i < getBodies().size(); ++i) {
            auto b = Clone(*getBodies()[i]);
            if (loaded && !b->IsStatic()) {
                b->velocity = {Float(double(b->velocity.x) + loads[i].vx * dt),
                               Float(double(b->velocity.y) + loads[i].vy * dt)};
                b->angularVelocity = Float(double(b->angularVelocity) + loads[i].spin * dt);
            }
            scratch.addBody(b);
            copies.push_back(std::move(b));
        }
        scratch.step(0); // No force generators, drives, load consumption or positional projection.
        std::vector<State> result;
        for (auto &b : copies)
            result.push_back({b->velocity.x, b->velocity.y, b->angularVelocity});
        for (auto p : pairs) {
            const auto manifold = CheckCollision(copies[p.first].get(), copies[p.second].get());
            residual = std::max(residual, Closing(manifold));
            tangential = std::max(tangential, Tangential(manifold));
        }
        ++report.scratchSolves;
        return result;
    }
    void RememberContacts() {
        previous.clear();
        const auto &bodies = getBodies();
        if (bodies.size() > 32)
            return;
        for (std::size_t i = 0; i < bodies.size(); ++i)
            for (std::size_t j = i + 1; j < bodies.size(); ++j)
                if (bodies[i]->CanCollideWith(*bodies[j]) &&
                    !(bodies[i]->IsStatic() && bodies[j]->IsStatic()) &&
                    CheckCollision(bodies[i].get(), bodies[j].get()).hasCollision)
                    previous.insert({bodies[i]->GetId(), bodies[j]->GetId()});
    }

  public:
    using PhysicsEngine::World::World;
    const Report &lastExperiment() const { return report; }
    void step() { step(getSimulationConfig().fixedTimeStep); }
    void step(float dt) {
        report = {};
        if (!std::isfinite(dt) || dt < 0)
            throw std::invalid_argument("Invalid experiment dt");
        report.eligibility = dt == 0 ? "zero_dt" : Unsupported();
        if (!report.eligibility.empty()) {
            PhysicsEngine::World::step(dt);
            RememberContacts();
            return;
        }
        const auto &bodies = getBodies();
        std::vector<State> loads;
        for (const auto &b : bodies) {
            auto copy = Clone(*b);
            copy->force = b->force;
            copy->torque = b->torque;
            for (const auto &f : getForceRegistry())
                if (f.body == b)
                    f.generator->applyForce(copy.get());
            for (const auto &f : getUniversalForceRegistry())
                f->applyForce(copy.get());
            loads.push_back({double(copy->force.x) * copy->inverseMass,
                             double(copy->force.y) * copy->inverseMass,
                             double(copy->torque) * copy->inverseInertia});
        }
        std::vector<Pair> pairs;
        for (std::size_t i = 0; i < bodies.size(); ++i)
            for (std::size_t j = i + 1; j < bodies.size(); ++j) {
                if (!bodies[i]->CanCollideWith(*bodies[j]) ||
                    (bodies[i]->IsStatic() && bodies[j]->IsStatic()))
                    continue;
                if (bodies[i]->material.restitution != 0 || bodies[j]->material.restitution != 0)
                    continue;
                const auto m = CheckCollision(bodies[i].get(), bodies[j].get());
                if (!m.hasCollision)
                    continue; // Never create overlap or enlarge geometric tolerances.
                const double scale = dt * (std::hypot(loads[i].vx, loads[i].vy) +
                                           std::hypot(loads[j].vx, loads[j].vy));
                const double roundoff = 64 * std::numeric_limits<float>::epsilon() * scale;
                if (!previous.count({bodies[i]->GetId(), bodies[j]->GetId()}) &&
                    Closing(m) > roundoff)
                    continue; // New closing impact.
                pairs.push_back({i, j});
            }
        report.pairs = pairs.size();
        report.bodies.resize(bodies.size());
        if (pairs.empty()) {
            report.eligibility = "no_start_contacts";
        } else {
            report.eligibility = "frozen_load_projection";
            const auto unloaded = Project(pairs, loads, dt, false, report.unloadedClosingResidual,
                                          report.unloadedTangentialSpeed);
            const auto loaded = Project(pairs, loads, dt, true, report.loadedClosingResidual,
                                        report.loadedTangentialSpeed);
            std::vector<Vector2> positions;
            std::vector<float> angles;
            for (std::size_t i = 0; i < bodies.size(); ++i) {
                const auto &b = *bodies[i];
                auto &r = report.bodies[i];
                r.loadX = b.mass * loads[i].vx * dt;
                r.loadY = b.mass * loads[i].vy * dt;
                r.loadSpin = b.inertia * loads[i].spin * dt;
                r.unloadedReactionX = b.mass * (unloaded[i].vx - b.velocity.x);
                r.unloadedReactionY = b.mass * (unloaded[i].vy - b.velocity.y);
                r.unloadedReactionSpin = b.inertia * (unloaded[i].spin - b.angularVelocity);
                r.loadedReactionX = b.mass * (loaded[i].vx - b.velocity.x - loads[i].vx * dt);
                r.loadedReactionY = b.mass * (loaded[i].vy - b.velocity.y - loads[i].vy * dt);
                r.loadedReactionSpin =
                    b.inertia * (loaded[i].spin - b.angularVelocity - loads[i].spin * dt);
                r.correctionX = .5 * dt * (loaded[i].vx - unloaded[i].vx - loads[i].vx * dt);
                r.correctionY = .5 * dt * (loaded[i].vy - unloaded[i].vy - loads[i].vy * dt);
                r.correctionAngle =
                    .5 * dt * (loaded[i].spin - unloaded[i].spin - loads[i].spin * dt);
                positions.push_back(b.IsStatic() ? b.position
                                                 : Vector2{Float(b.position.x + r.correctionX),
                                                           Float(b.position.y + r.correctionY)});
                angles.push_back(b.IsStatic() ? b.orientation
                                              : Float(b.orientation + r.correctionAngle));
            }
            for (std::size_t i = 0; i < bodies.size(); ++i) {
                // Direct diagnostic pose writes avoid injecting public teleport wake requests.
                bodies[i]->position = positions[i];
                bodies[i]->orientation = angles[i];
            }
        }
        PhysicsEngine::World::step(dt);
        RememberContacts();
    }
};
} // namespace ContactLoadExperiment

#pragma once
// Benchmark-only opt-in strategy. Engine's seam never includes this operator.
#include "../../src/physics/core/experimental_world_step.h"
#include "physics/core/forces/gravity.h"
#include "planar_contact_interval.h"
#include <limits>
#include <string>
#include <typeinfo>
namespace PlanarContactBridge {
using namespace PhysicsEngine;
struct Report {
    std::string eligibility = "not_run";
    bool proposed = false;
    bool selected = false;
    PlanarContactInterval::Result interval;
    Vector2 appliedForce;
    float appliedTorque = 0;
    double positionStorageError = 0, velocityStorageError = 0, reactionTorqueResidual = 0;
};
class World : public PhysicsEngine::World, private Detail::IntegrationStrategy {
    Report report;
    struct Box {
        double halfX, halfY;
    };
    static Box Rectangle(const RigidBody &b) {
        if (b.orientation != 0 || b.shape->type != ShapeType::POLYGON)
            throw std::invalid_argument("geometry");
        const auto *p = dynamic_cast<const Polygon *>(b.shape.get());
        if (!p || p->getVertices().size() != 4)
            throw std::invalid_argument("geometry");
        double hx = 0, hy = 0;
        for (auto v : p->getVertices()) {
            hx = std::max(hx, std::abs(double(v.x)));
            hy = std::max(hy, std::abs(double(v.y)));
        }
        if (hx == 0 || hy == 0)
            throw std::invalid_argument("geometry");
        unsigned corners = 0;
        for (auto v : p->getVertices()) {
            if (std::abs(double(v.x)) != hx || std::abs(double(v.y)) != hy)
                throw std::invalid_argument("geometry");
            const unsigned bit = 1u << ((v.x > 0 ? 1 : 0) + (v.y > 0 ? 2 : 0));
            if (corners & bit)
                throw std::invalid_argument("geometry");
            corners |= bit;
        }
        return {hx, hy};
    }
    static float Float(double x) {
        if (!std::isfinite(x) || std::abs(x) > std::numeric_limits<float>::max())
            throw std::overflow_error("float_storage");
        const auto r = static_cast<float>(x);
        if (x != 0 && r == 0)
            throw std::overflow_error("float_storage");
        return r;
    }
    bool stage(const PhysicsEngine::World &w, float dt, Detail::IntegrationPlan &plan) override {
        report = {};
        const auto &c = w.getSimulationConfig();
        auto reject = [&](const char *reason) {
            report.eligibility = reason;
            return false;
        };
        if (dt == 0)
            return reject("zero_dt");
        if (c.enableSleeping)
            return reject("sleeping");
        if (c.enableLinearVelocityLimit || c.enableAngularVelocityLimit)
            return reject("velocity_caps");
        if (c.warmStartFactor != 0)
            return reject("warming");
        if (!w.getJoints().empty())
            return reject("joints");
        if (!w.getParticleSystems().empty())
            return reject("particles");
        if (w.getBodies().size() != 2)
            return reject("body_count");
        RigidBody *body = nullptr;
        RigidBody *floor = nullptr;
        for (const auto &b : w.getBodies()) {
            if (b->IsCcdEnabled())
                return reject("ccd");
            if (b->IsStatic()) {
                if (floor)
                    return reject("body_count");
                floor = b.get();
            } else {
                if (body)
                    return reject("dynamic_component");
                body = b.get();
            }
        }
        if (!body || !floor)
            return reject("dynamic_component");
        if (!(floor->velocity == Vector2{}) || floor->angularVelocity != 0)
            return reject("moving_support");
        if (floor->inverseMass != 0 || floor->inverseInertia != 0)
            return reject("support_properties");
        if (body->angularVelocity != 0)
            return reject("rotation");
        if (!body->CanCollideWith(*floor))
            return reject("filtered");
        if (body->material.restitution != 0 || floor->material.restitution != 0)
            return reject("restitution");
        for (const auto &f : w.getForceRegistry()) {
            const auto *generator = f.generator.get();
            if (typeid(*generator) != typeid(Gravity))
                return reject("opaque_force");
        }
        for (const auto &f : w.getUniversalForceRegistry()) {
            const auto *generator = f.get();
            if (typeid(*generator) != typeid(Gravity))
                return reject("opaque_force");
        }
        try {
            // One accumulated load sample; never run a generator in this strategy.
            const auto force = body->force;
            const auto torque = body->torque;
            const auto shape = Rectangle(*body), support = Rectangle(*floor);
            const double top = double(floor->position.y) + support.halfY;
            if (double(Float(top)) != top || top - double(floor->position.y) != support.halfY)
                return reject("float_geometry");
            const double gap = (double(body->position.y) - top) - shape.halfY;
            if (gap < 0)
                return reject("overlap");
            PlanarContactInterval::Input input;
            input.mass = body->mass;
            input.tangentPosition = body->position.x;
            input.tangentVelocity = body->velocity.x;
            input.normalVelocity = body->velocity.y;
            input.gap = gap;
            input.forceT = force.x;
            input.forceN = force.y;
            input.torque = torque;
            // Match the solver's represented mixed coefficient.
            input.staticFriction = Float(
                std::sqrt(double(body->material.staticFriction) * floor->material.staticFriction));
            input.dynamicFriction = Float(std::sqrt(double(body->material.dynamicFriction) *
                                                    floor->material.dynamicFriction));
            input.left = -shape.halfX;
            input.right = shape.halfX;
            input.leverDepth = shape.halfY;
            input.duration = dt;
            const auto result = PlanarContactInterval::Advance(input);
            report.interval = result;
            report.appliedForce = force;
            report.appliedTorque = torque;
            if (result.status == PlanarContactInterval::Status::NeedsImpact)
                return reject("needs_impact");
            if (result.status == PlanarContactInterval::Status::UnsupportedWrench)
                return reject("unsupported_wrench");
            double x = input.tangentPosition;
            const double floorLeft = double(floor->position.x) - support.halfX;
            const double floorRight = double(floor->position.x) + support.halfX;
            const auto within = [&](double center) {
                return center - shape.halfX >= floorLeft && center + shape.halfX <= floorRight;
            };
            if (!within(x))
                return reject("support_extent");
            for (const auto &interval : result.intervals) {
                // A free interval can reverse without being split by a contact
                // event. Check its interior extremum as well as its endpoints.
                if (interval.duration > 0 && interval.initialTangentVelocity !=
                                                 interval.finalTangentVelocity) {
                    const double acceleration = (interval.finalTangentVelocity -
                                                 interval.initialTangentVelocity) /
                                                interval.duration;
                    const double stop = -interval.initialTangentVelocity / acceleration;
                    if (stop > 0 && stop < interval.duration &&
                        !within(x + .5 * interval.initialTangentVelocity * stop))
                        return reject("support_extent");
                }
                x += interval.tangentDistance;
                if (!within(x))
                    return reject("support_extent");
            }
            plan.body = body;
            plan.bodyId = body->GetId();
            plan.startPosition = body->position;
            plan.startVelocity = body->velocity;
            plan.startForce = force;
            plan.startOrientation = body->orientation;
            plan.startAngularVelocity = body->angularVelocity;
            plan.startTorque = torque;
            plan.position = {Float(result.tangentPosition), Float(top + shape.halfY + result.gap)};
            plan.velocity = {Float(result.tangentVelocity), Float(result.normalVelocity)};
            plan.orientation = body->orientation;
            plan.angularVelocity = 0;
            if (!within(plan.position.x))
                return reject("support_extent");
            const double storedGap = (double(plan.position.y) - top) - shape.halfY;
            plan.closedContact = result.gap == 0 && result.normalVelocity == 0;
            if (plan.closedContact ? storedGap != 0 : (result.gap > 0 && storedGap <= 0))
                return reject("float_geometry");
            report.positionStorageError =
                std::hypot(double(plan.position.x) - result.tangentPosition,
                           double(plan.position.y) - (top + shape.halfY + result.gap));
            report.velocityStorageError =
                std::hypot(double(plan.velocity.x) - result.tangentVelocity,
                           double(plan.velocity.y) - result.normalVelocity);
            if (plan.closedContact) {
                auto &m = plan.manifold;
                m.A = floor;
                m.B = body;
                m.hasCollision = true;
                m.normal = {0, 1};
                m.contactCount = 2;
                m.contacts[0] = {
                    {Float(double(plan.position.x) - shape.halfX), Float(top)}, 0, 0x7a000001u};
                m.contacts[1] = {
                    {Float(double(plan.position.x) + shape.halfX), Float(top)}, 0, 0x7a000002u};
                if (m.contacts[0].position.x == m.contacts[1].position.x)
                    return reject("float_geometry");
                // PrepareConstraint stores local anchors as floats. Preflight
                // every anchor using the endpoint before publishing that state.
                for (unsigned i = 0; i < 2; ++i) {
                    Float(double(m.contacts[i].position.x) - plan.position.x);
                    Float(double(m.contacts[i].position.y) - plan.position.y);
                    Float(double(m.contacts[i].position.x) - floor->position.x);
                    Float(double(m.contacts[i].position.y) - floor->position.y);
                }
                m.contactPoint = {
                    Float((double(m.contacts[0].position.x) + m.contacts[1].position.x) * .5),
                    Float(top)};
                plan.reaction.contactCount = 2;
                plan.reaction.contacts[0].featureId = m.contacts[0].featureId;
                plan.reaction.contacts[1].featureId = m.contacts[1].featureId;
                for (const auto &interval : result.intervals) {
                    auto &left = plan.reaction.contacts[0].impulse;
                    auto &right = plan.reaction.contacts[1].impulse;
                    left.normal += interval.leftNormal * interval.duration;
                    right.normal += interval.rightNormal * interval.duration;
                    // Solver tangent is (-ny,nx): for the body's +x force the
                    // cached scalar is negative, invariant under A/B swapping.
                    left.tangent -= interval.leftTangent * interval.duration;
                    right.tangent -= interval.rightTangent * interval.duration;
                }
                for (const auto &point : plan.reaction.contacts)
                    if (!std::isfinite(point.impulse.normal) ||
                        !std::isfinite(point.impulse.tangent))
                        return reject("reaction_range");
                double torque = 0, scale = std::abs(result.spinImpulse);
                for (unsigned i = 0; i < 2; ++i) {
                    const double rx = double(m.contacts[i].position.x) - plan.position.x;
                    const double ry = double(m.contacts[i].position.y) - plan.position.y;
                    const auto j = plan.reaction.contacts[i].impulse;
                    const double nm = rx * j.normal, tm = ry * j.tangent;
                    torque += nm + tm;
                    scale += std::abs(nm) + std::abs(tm);
                }
                report.reactionTorqueResidual = torque - result.spinImpulse;
                if (!std::isfinite(torque) || !std::isfinite(scale) ||
                    std::abs(report.reactionTorqueResidual) >
                        64 * std::numeric_limits<float>::epsilon() * scale)
                    return reject("float_wrench");
            }
            report.proposed = true;
            report.eligibility = plan.closedContact ? "planar_support" : "planar_free";
            return true;
        } catch (const std::invalid_argument &e) {
            report.eligibility = e.what();
            return false;
        } catch (const std::overflow_error &) {
            return reject("derived_range");
        }
    }

  public:
    using PhysicsEngine::World::World;
    const Report &lastExperiment() const { return report; }
    void step() { step(getSimulationConfig().fixedTimeStep); }
    void step(float dt) {
        report.selected = Detail::ExperimentalWorldStep::Step(*this, dt, *this);
        if (report.proposed && !report.selected)
            report.eligibility = "publication_validation";
    }
};
} // namespace PlanarContactBridge

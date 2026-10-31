#include "physics/core/world.h"
#include "experimental_world_step.h"
#include "trig_cache.h"
#include "physics/core/collisions/collision_resolver.h"
#include "physics/core/collisions/broad_phase/sweep_and_prune.h"
#include "physics/core/collisions/continuous_collision.h"
#include "collisions/continuous_collision_detail.h"
#include "physics/math/matrix2x2.h"
#include <utility>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <unordered_set>
#include <numeric>
#include <limits>
#include <tuple>
#include <array>
#include <typeinfo>

namespace PhysicsEngine {

World::ContactKey World::ContactKey::From(const RigidBody* bodyA, const RigidBody* bodyB) {
    const std::uint64_t idA = bodyA->GetId();
    const std::uint64_t idB = bodyB->GetId();
    return idA < idB ? ContactKey{idA, idB} : ContactKey{idB, idA};
}

bool World::ContactKey::contains(std::uint64_t bodyId) const {
    return first == bodyId || second == bodyId;
}

bool World::ContactKey::operator==(const ContactKey& other) const {
    return first == other.first && second == other.second;
}

std::size_t World::ContactKeyHash::operator()(const ContactKey& key) const {
    const std::size_t firstHash = std::hash<std::uint64_t>{}(key.first);
    const std::size_t secondHash = std::hash<std::uint64_t>{}(key.second);
    return firstHash ^ (secondHash << 1);
}

namespace {
    void CanonicalizeManifold(CollisionManifold& manifold) {
        if (manifold.A->GetId() > manifold.B->GetId()) {
            std::swap(manifold.A, manifold.B);
            manifold.normal = manifold.normal * -1.0f;
        }
    }

    // Ends are collected from a hash map, whose order depends on the body
    // ids; sorting them by pair key makes the order depend only on the pairs.
    template <class Events>
    void SortByPair(Events& events, std::size_t first) {
        std::sort(events.begin() + static_cast<std::ptrdiff_t>(first), events.end(), [](const auto& x, const auto& y) {
            return std::tie(x.event.bodyAId, x.event.bodyBId) < std::tie(y.event.bodyAId, y.event.bodyBId);
        });
    }

    bool HasSensor(const CollisionPair& pair) {
        return pair.first->IsSensor() || pair.second->IsSensor();
    }

    // Category/mask bits, then joints that disable collision between their
    // bodies. KeySet is World::ContactKeySet.
    template <class KeySet>
    bool PassesFilter(const CollisionPair& pair, const KeySet& jointedPairs) {
        using Key = typename KeySet::key_type;
        return pair.first && pair.second && pair.first->CanCollideWith(*pair.second)
            && (jointedPairs.empty() || !jointedPairs.count(Key::From(pair.first.get(), pair.second.get())));
    }

    // Drops the candidates that do not collide, in place and in order. A pair
    // with exactly one sensor that passes filtering moves to sensorPairs
    // instead: it is tested for overlap after the step and never solved.
    template <class KeySet>
    void SplitCandidates(std::vector<CollisionPair>& pairs, std::vector<CollisionPair>& sensorPairs,
                         const KeySet& jointedPairs) {
        sensorPairs.clear();
        pairs.erase(std::remove_if(pairs.begin(), pairs.end(), [&](const CollisionPair& pair) {
            if (!PassesFilter(pair, jointedPairs)) return true;
            if (!HasSensor(pair)) return false;
            if (pair.first->IsSensor() != pair.second->IsSensor()) sensorPairs.push_back(pair);
            return true;
        }), pairs.end());
    }
}

// Per-step working vectors kept between steps, so their (large) buffers are
// reused instead of being reallocated and faulted in every step.
struct World::StepScratch {
    struct Entry { ContactKey key; CollisionManifold manifold; };
    struct EndContact { ContactKey key; CollisionManifold manifold; const CollisionPair* pair; };
    std::vector<Entry> entries;
    std::vector<ContactConstraint> constraints, endConstraints;
    std::vector<ContactImpulseCache> endCaches;
    std::vector<EndContact> ended;
    // This step's sensor candidates (see SplitCandidates).
    std::vector<CollisionPair> sensorPairs;
    // Body positions when advanceCcd returned (see sweepCcdCorrections).
    std::vector<Vector2> ccdPositions;
};

World::World() : World(SimulationConfig{}) {}

World::World(const SimulationConfig& config)
    : simulationConfig(config),
      broadPhase(std::make_unique<SweepAndPrune>()) {
    simulationConfig.Validate();
}

World::~World() {}

void World::requireMutationAllowed() const {
    if (stepping && !dispatchingEvents)
        throw std::logic_error("World structure cannot change during integration or solving.");
}

void World::addBody(RigidBodyPtr body) {
    requireMutationAllowed();
    if (!body) throw std::invalid_argument("Body cannot be null.");
    if (body && std::find(bodies.begin(), bodies.end(), body) == bodies.end()) {
        bodies.push_back(body);
    }
}

void World::removeBody(RigidBodyPtr body) {
    requireMutationAllowed();
    for (const auto& joint : joints) {
        if (joint->getBodyA() == body || joint->getBodyB() == body) {
            joint->getBodyA()->Wake(); joint->getBodyB()->Wake();
        }
    }
    joints.erase(std::remove_if(joints.begin(), joints.end(), [&body](const JointPtr& joint) {
        return joint->getBodyA() == body || joint->getBodyB() == body;
    }), joints.end());
    bodies.erase(std::remove(bodies.begin(), bodies.end(), body), bodies.end());

    forceRegistry.erase(std::remove_if(forceRegistry.begin(), forceRegistry.end(), 
        [body](const ForceRegistration& reg) {
            return reg.body == body;
        }), forceRegistry.end());

    if (body) {
        const std::uint64_t bodyId = body->GetId();
        for (auto it = contactCache.begin(); it != contactCache.end();) {
            if (it->first.contains(bodyId)) {
                if (it->second.touchingStep == touchingStamp) --touchingCount;
                it = contactCache.erase(it);
            } else {
                ++it;
            }
        }
        potentialCollisions.erase(std::remove_if(potentialCollisions.begin(),
            potentialCollisions.end(), [&body](const CollisionPair& pair) {
                return pair.first == body || pair.second == body;
            }), potentialCollisions.end());
        endContacts(bodyId);
    }
    dispatchEvents();
}

void World::clearBodies() {
    requireMutationAllowed();
    joints.clear();
    for (const auto& body : bodies) {
        endContacts(body->GetId());
    }
    bodies.clear();
    forceRegistry.clear();
    universalForceRegistry.clear();
    potentialCollisions.clear();
    contactCache.clear();
    touchingCount = 0;
    dispatchEvents();
}

void World::addForce(RigidBodyPtr body, std::unique_ptr<IForceGenerator> generator) {
    requireMutationAllowed();
    if (!body || !generator) throw std::invalid_argument("Force registration requires a body and generator.");
    body->Wake();
    this->forceRegistry.push_back({body, std::move(generator)});
}

void World::addUniversalForce(std::unique_ptr<IForceGenerator> generator) {
    requireMutationAllowed();
    if (!generator) throw std::invalid_argument("Universal force requires a generator.");
    for (const auto& body : bodies) body->Wake();
    this->universalForceRegistry.push_back(std::move(generator));
}

void World::addParticleSystem(ParticleSystemPtr system) {
    requireMutationAllowed();
    if (!system) throw std::invalid_argument("Particle system cannot be null.");
    if (std::find(particleSystems.begin(), particleSystems.end(), system) == particleSystems.end()) {
        particleSystems.push_back(std::move(system));
    }
}

void World::removeParticleSystem(const ParticleSystemPtr& system) {
    requireMutationAllowed();
    particleSystems.erase(
        std::remove(particleSystems.begin(), particleSystems.end(), system),
        particleSystems.end()
    );
}

void World::clearParticleSystems() {
    requireMutationAllowed();
    particleSystems.clear();
}

void World::addCollisionListener(ICollisionListener* listener) {
    if (!listener) throw std::invalid_argument("Collision listener cannot be null.");
    if (listener && std::find(collisionListeners.begin(), collisionListeners.end(),
            listener) == collisionListeners.end()) {
        collisionListeners.push_back(listener);
    }
}

void World::removeCollisionListener(ICollisionListener* listener) {
    collisionListeners.erase(
        std::remove(collisionListeners.begin(), collisionListeners.end(), listener),
        collisionListeners.end()
    );
}

void World::setBroadPhase(std::unique_ptr<IBroadPhase> bp) {
    requireMutationAllowed();
    if (!bp) throw std::invalid_argument("Broad phase cannot be null.");
    if (bp) {
        broadPhase = std::move(bp);
    }
}

void World::setSimulationConfig(const SimulationConfig& config) {
    requireMutationAllowed();
    config.Validate();
    simulationConfig = config;
    for (const auto& body : bodies) body->Wake();
}

void World::step() {
    step(simulationConfig.fixedTimeStep);
}

void World::step(float deltaTime) { stepInternal(deltaTime, nullptr); }

namespace {
// Treat each solved contact impulse as a constant force over the step. The
// integrator has already moved bodies with their pre-contact constant load, so
// the matching displacement is half of the impulse's velocity change times dt.
// Linear displacement uses only the tangential impulse; rotation uses the full
// contact wrench so normal and friction torques stay balanced. Equal and
// opposite pair terms leave the center of mass on its momentum trajectory.
void ApplyContactDisplacement(const std::vector<ContactConstraint*>& contacts,
    const ContactConstraint* skipConstraint, float deltaTime,
    const std::unordered_set<const RigidBody*>& skipBodies) {
    struct Delta { double x = 0, y = 0, angle = 0; };
    std::vector<std::pair<RigidBody*, Delta>> deltas;
    const auto delta = [&](RigidBody* body) -> Delta& {
        for (auto& entry : deltas) if (entry.first == body) return entry.second;
        return deltas.emplace_back(body, Delta{}).second;
    };
    const double half = 0.5 * double(deltaTime);
    for (const auto* c : contacts) {
        if (c == skipConstraint) continue;
        for (std::uint8_t i = 0; i < c->pointCount; ++i) {
            const auto& p = c->points[i];
            const double px = double(c->normal.x) * p.impulse.normal + double(c->tangent.x) * p.impulse.tangent;
            const double py = double(c->normal.y) * p.impulse.normal + double(c->tangent.y) * p.impulse.tangent;
            // Normal settling stays with penetration slop and position correction.
            const double tx = double(c->tangent.x) * p.impulse.tangent, ty = double(c->tangent.y) * p.impulse.tangent;
            for (const auto& [body, anchor, sign] : {std::tuple<RigidBody*, Vector2, double>{c->bodyA, p.localAnchorA, -1.0},
                                                     {c->bodyB, p.localAnchorB, 1.0}}) {
                if (body->IsStatic() || skipBodies.count(body)) continue;
                const Detail::CosSin rotation = Detail::CachedCosSin(double(body->orientation));
                const double cosine = rotation.cosine, sine = rotation.sine;
                const double rx = cosine * anchor.x - sine * anchor.y, ry = sine * anchor.x + cosine * anchor.y;
                auto& d = delta(body);
                d.x += sign * half * body->inverseMass * tx;
                d.y += sign * half * body->inverseMass * ty;
                d.angle += sign * half * body->inverseInertia * (rx * py - ry * px);
            }
        }
    }
    for (const auto& [body, d] : deltas) {
        const double x = body->position.x + d.x, y = body->position.y + d.y, angle = body->orientation + d.angle;
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(angle)
            || std::abs(x) > std::numeric_limits<float>::max() || std::abs(y) > std::numeric_limits<float>::max())
            throw std::overflow_error("Contact displacement exceeds finite float range.");
        body->position = {static_cast<float>(x), static_cast<float>(y)};
        body->orientation = static_cast<float>(angle);
    }
}
}

bool World::stepInternal(float deltaTime, Detail::IntegrationStrategy *strategy) {
    if (stepping || dispatchingEvents) {
        throw std::logic_error("World::step cannot be called from a collision callback.");
    }
    if (!std::isfinite(deltaTime) || deltaTime < 0.0f) {
        throw std::invalid_argument("World delta time must be finite and non-negative.");
    }
    struct StepGuard {
        bool& flag;
        explicit StepGuard(bool& flag) : flag(flag) { flag = true; }
        ~StepGuard() { flag = false; }
    } guard(stepping);
    if (deltaTime > 0) validateCcdSupport();
    SimulationStatistics statistics;
    potentialCollisions.clear();
    const std::uint64_t stamp = ++stepStamp;
    for (std::size_t i = 0; i < bodies.size(); ++i) bodies[i]->worldIndex = i;

    for (const ParticleSystemPtr& system : particleSystems) {
        statistics.integratedParticleCount += static_cast<std::uint32_t>(system->size());
        system->step(deltaTime);
    }

    std::vector<ContactConstraint> previousConstraints;
    for (const auto& entry : contactEvents) {
        const auto& event = entry.second;
        // Sensor overlaps wake nobody, unless SetSensor(false) just made the
        // pair solid.
        if (event.event.sensor && (event.bodyA->IsSensor() || event.bodyB->IsSensor())) continue;
        if (event.bodyA->contactWakeRequested || event.bodyB->contactWakeRequested) {
            event.bodyA->Wake(); event.bodyB->Wake();
        }
    }
    for (const auto& joint : joints) {
        joint->prepareStep(deltaTime);
        if (joint->preventsSleeping() || joint->getBodyA()->contactWakeRequested || joint->getBodyB()->contactWakeRequested) {
            joint->getBodyA()->Wake(); joint->getBodyB()->Wake();
        }
    }
    for (const auto& body : bodies) body->contactWakeRequested = false;
    auto previousIslands = buildIslands(previousConstraints, true);
    for (auto& island : previousIslands) wakeIsland(island);
    auto applyAutomaticForce = [](RigidBody& body, IForceGenerator& generator) {
        if (!body.IsAwake()) return;
        body.applyingAutomaticForces = true;
        try { generator.applyForce(&body); }
        catch (...) { body.applyingAutomaticForces = false; throw; }
        body.applyingAutomaticForces = false;
    };
    for (auto& registration : forceRegistry)
        applyAutomaticForce(*registration.body, *registration.generator);
    for (auto& generator : universalForceRegistry)
        for (auto& body : bodies) applyAutomaticForce(*body, *generator);

    Detail::IntegrationPlan integrationPlan;
    bool integratedPlan = false;
    // Capture engine-owned identities before invoking the const staging seam.
    // A strategy's proposed endpoint can only target an unchanged owned body.
    struct Snapshot {
        RigidBody *body;
        std::uint64_t id;
        Vector2 position, velocity, force;
        float orientation, angularVelocity, torque, mass, inverseMass, inertia, inverseInertia;
        Material material;
        std::uint32_t category, mask;
        bool awake, ccd;
        explicit Snapshot(RigidBody &b)
            : body(&b), id(b.GetId()), position(b.position), velocity(b.velocity), force(b.force),
              orientation(b.orientation), angularVelocity(b.angularVelocity), torque(b.torque),
              mass(b.mass), inverseMass(b.inverseMass), inertia(b.inertia), inverseInertia(b.inverseInertia),
              material(b.material), category(b.GetCollisionCategoryBits()), mask(b.GetCollisionMaskBits()),
              awake(b.IsAwake()), ccd(b.IsCcdEnabled()) {}
        bool unchanged() const {
            const auto &b = *body;
            return b.GetId() == id && b.position == position && b.velocity == velocity &&
                   b.force == force && b.orientation == orientation &&
                   b.angularVelocity == angularVelocity && b.torque == torque &&
                   b.mass == mass && b.inverseMass == inverseMass && b.inertia == inertia &&
                   b.inverseInertia == inverseInertia && b.material.density == material.density &&
                   b.material.restitution == material.restitution &&
                   b.material.staticFriction == material.staticFriction &&
                   b.material.dynamicFriction == material.dynamicFriction &&
                   b.GetCollisionCategoryBits() == category && b.GetCollisionMaskBits() == mask &&
                   b.IsAwake() == awake && b.IsCcdEnabled() == ccd;
        }
    };
    std::vector<Snapshot> stagingSnapshots;
    if (strategy && bodies.size() == 2)
        for (const auto &body : bodies) stagingSnapshots.emplace_back(*body);
    if (strategy && strategy->stage(*this, deltaTime, integrationPlan)) {
        const auto &p = integrationPlan;
        const auto member = [&](RigidBody *b) {
            return std::any_of(bodies.begin(), bodies.end(),
                               [&](const RigidBodyPtr &owned) { return owned.get() == b; });
        };
        const auto finite = [](Vector2 v) { return std::isfinite(v.x) && std::isfinite(v.y); };
        // Validate identity and the untouched staging snapshot before publication.
        integratedPlan =
            bodies.size() == 2 && stagingSnapshots.size() == 2 &&
            stagingSnapshots[0].unchanged() && stagingSnapshots[1].unchanged() &&
            joints.empty() && particleSystems.empty() &&
            !simulationConfig.enableSleeping && !simulationConfig.enableLinearVelocityLimit &&
            !simulationConfig.enableAngularVelocityLimit && simulationConfig.warmStartFactor == 0 &&
            std::count_if(bodies.begin(), bodies.end(),
                          [](const RigidBodyPtr &b) { return b->IsStatic(); }) == 1 &&
            std::none_of(bodies.begin(), bodies.end(),
                         [](const RigidBodyPtr &b) { return b->IsCcdEnabled(); }) &&
            p.body && member(p.body) && p.body->GetId() == p.bodyId && !p.body->IsStatic() &&
            p.body->IsAwake() && finite(p.position) && finite(p.velocity) &&
            std::isfinite(p.orientation) && std::isfinite(p.angularVelocity) &&
            p.body->position == p.startPosition && p.body->velocity == p.startVelocity &&
            p.body->force == p.startForce && p.body->orientation == p.startOrientation &&
            p.body->angularVelocity == p.startAngularVelocity && p.body->torque == p.startTorque &&
            std::isfinite(p.body->inverseMass) && p.body->inverseMass > 0 &&
            std::isfinite(p.body->inverseInertia) && p.body->inverseInertia > 0;
        if (integratedPlan && p.closedContact) {
            const auto &m = p.manifold;
            // A sensor pair is never a contact, so a plan that closes one
            // falls back to the regular step.
            integratedPlan = m.hasCollision && m.A && m.B && m.A != m.B && member(m.A) &&
                             !m.A->IsSensor() && !m.B->IsSensor() &&
                             member(m.B) && (m.A == p.body || m.B == p.body) &&
                             m.contactCount == 2 && p.reaction.contactCount == 2 &&
                             finite(m.normal) && finite(m.contactPoint) && m.normal.magnitudeSquared() == 1 &&
                             m.penetration == 0;
            for (unsigned i = 0; i < 2 && integratedPlan; ++i) {
                const auto &c = m.contacts[i];
                const auto &j = p.reaction.contacts[i];
                integratedPlan = finite(c.position) && c.penetration == 0 &&
                                 c.featureId == j.featureId && std::isfinite(j.impulse.normal) &&
                                 j.impulse.normal >= 0 && std::isfinite(j.impulse.tangent);
            }
            integratedPlan = integratedPlan && m.contacts[0].featureId != m.contacts[1].featureId;
        }
    }

    jointedPairs.clear();
    for (const auto& joint : joints)
        if (!joint->getCollideConnected())
            jointedPairs.insert(ContactKey::From(joint->getBodyA().get(), joint->getBodyB().get()));
    std::vector<CollisionManifold> impacts;
    // A published strategy plan is an end-of-step state with its contact
    // already closed at the integrated pose, which only fits the legacy order.
    if (simulationConfig.enableSpeculativeContacts && !integratedPlan) {
        stepSpeculative(deltaTime, statistics, impacts);
    } else {
        std::vector<Vector2> starts;
        const bool useCcd = deltaTime > 0 && std::any_of(bodies.begin(), bodies.end(),
            [](const RigidBodyPtr& body) { return body->IsCcdEnabled(); });
        if (useCcd) {
            for (const auto& body : bodies) starts.push_back(body->position);
        }
        // Integrate velocities and positions
        for (RigidBodyPtr& body : bodies) {
            if (body->IsAwake()) {
                ++statistics.integratedBodyCount;
                if (integratedPlan && body.get() == integrationPlan.body) {
                    body->position = integrationPlan.position;
                    body->orientation = integrationPlan.orientation;
                    body->velocity = integrationPlan.velocity;
                    body->angularVelocity = integrationPlan.angularVelocity;
                    body->force = {};
                    body->torque = 0;
                } else
                    body->Integrate(deltaTime, simulationConfig);

                body->orientation = std::remainder(body->orientation, 6.283185307179586f);
            }
        }

        if (useCcd) impacts = advanceCcd(starts, deltaTime, statistics);

        // Detect contacts once, then solve the prepared constraints in distinct
        // velocity and position phases.
        potentialCollisions = broadPhase->FindPotentialCollisions(bodies);
        if (integratedPlan && integrationPlan.closedContact) {
            const auto &m = integrationPlan.manifold;
            const bool exists = std::any_of(
                potentialCollisions.begin(), potentialCollisions.end(), [&](const CollisionPair &pair) {
                    return (pair.first.get() == m.A && pair.second.get() == m.B) ||
                           (pair.first.get() == m.B && pair.second.get() == m.A);
                });
            if (!exists) {
                RigidBodyPtr a, b;
                for (const auto &body : bodies) {
                    if (body.get() == m.A)
                        a = body;
                    if (body.get() == m.B)
                        b = body;
                }
                potentialCollisions.emplace_back(a, b);
            }
        }
        statistics.broadPhaseCandidateCount = static_cast<std::uint32_t>(
            potentialCollisions.size()
        );
        if (!scratch) scratch = std::make_unique<StepScratch>();
        SplitCandidates(potentialCollisions, scratch->sensorPairs, jointedPairs);
        statistics.narrowPhaseCandidateCount = static_cast<std::uint32_t>(
            potentialCollisions.size() + scratch->sensorPairs.size()
        );

        std::vector<ContactConstraint> constraints;
        constraints.reserve(potentialCollisions.size());
        ContactConstraint *integratedConstraint = nullptr;
        for (const auto &pair : potentialCollisions) {
            const bool closed = integratedPlan && integrationPlan.closedContact &&
                                ((pair.first.get() == integrationPlan.manifold.A &&
                                  pair.second.get() == integrationPlan.manifold.B) ||
                                 (pair.first.get() == integrationPlan.manifold.B &&
                                  pair.second.get() == integrationPlan.manifold.A));
            CollisionManifold manifold =
                closed ? integrationPlan.manifold : CheckCollision(pair.first.get(), pair.second.get());
            if (!manifold.hasCollision) {
                continue;
            }

            statistics.resolvedContactCount += std::max<std::uint32_t>(
                manifold.contactCount,
                1
            );
            CanonicalizeManifold(manifold);
            const ContactKey key = ContactKey::From(manifold.A, manifold.B);
            auto [contact, inserted] = contactCache.try_emplace(key);
            contact->second.keptStep = stamp;
            auto& cache = contact->second.impulses;

            if (closed)
                cache = integrationPlan.reaction;
            if (!inserted && !closed) {
                for (std::uint8_t i = 0;
                     i < cache.contactCount;
                     ++i) {
                    cache.contacts[i].impulse.normal *=
                        simulationConfig.warmStartFactor;
                    cache.contacts[i].impulse.tangent *=
                        simulationConfig.warmStartFactor;
                }
            }

            constraints.push_back(CollisionResolver::PrepareConstraint(
                manifold,
                cache,
                simulationConfig,
                true
            ));
            // Impulses cached by a substepped solve are per substep.
            CollisionResolver::MatchImpulseSubsteps(constraints.back(), 1);
            if (closed) integratedConstraint = &constraints.back();
            publishContact(key, manifold, pair.first, pair.second, false);
        }

        std::unordered_set<const RigidBody*> ccdImpacted;
        for (const auto& impact : impacts) { ccdImpacted.insert(impact.A); ccdImpacted.insert(impact.B); }
        auto islands = buildIslands(constraints);
        statistics.islandCount = static_cast<std::uint32_t>(islands.size());
        statistics.solverIterationCount = integratedPlan ? 0 : simulationConfig.solverIterations;
        for (auto& island : islands) {
            wakeIsland(island);
            if (!island.bodies.front()->IsAwake()) {
                statistics.sleepingBodyCount += static_cast<std::uint32_t>(island.bodies.size());
                continue;
            }
            ++statistics.solvedIslandCount;
            statistics.solvedConstraintCount += static_cast<std::uint32_t>(island.contacts.size()+island.joints.size());
            for (const auto& joint : island.joints) joint->warmStart(simulationConfig.warmStartFactor);
            for (auto* constraint : island.contacts)
                if (constraint != integratedConstraint) CollisionResolver::WarmStart(*constraint);
            const bool alreadySolved = integratedPlan && island.joints.empty() &&
                std::all_of(island.contacts.begin(), island.contacts.end(),
                            [&](const ContactConstraint *c) { return c == integratedConstraint; });
            for (int iter=0; !alreadySolved && iter<simulationConfig.solverIterations; ++iter) {
                for (auto* constraint : island.contacts)
                    if (constraint != integratedConstraint) CollisionResolver::SolveVelocity(*constraint);
                for (const auto& joint : island.joints) joint->solveVelocity();
            }
            // Experimental integration strategies own their displacement.
            if (!alreadySolved && deltaTime > 0 && !strategy && simulationConfig.enableFrictionDisplacementCorrection)
                ApplyContactDisplacement(island.contacts, integratedConstraint, deltaTime, ccdImpacted);
            for (int iter=0; !alreadySolved && iter<simulationConfig.solverIterations; ++iter) {
                bool solved = true;
                for (auto* constraint : island.contacts)
                    if (constraint != integratedConstraint)
                        solved = CollisionResolver::SolvePosition(*constraint) && solved;
                for (const auto& joint : island.joints)
                    solved = joint->solvePosition(simulationConfig.penetrationSlop,
                        simulationConfig.maxPositionCorrection) && solved;
                if (solved) break;
            }
            if (simulationConfig.enableSleeping && deltaTime > 0) {
                bool canSleep = std::none_of(island.joints.begin(), island.joints.end(),
                    [](const JointPtr& joint) { return joint->preventsSleeping(); });
                for (auto* body : island.bodies) {
                    const float specificEnergy = 0.5f*(body->velocity.magnitudeSquared()
                        + body->inertia*body->inverseMass*body->angularVelocity*body->angularVelocity);
                    if (specificEnergy <= simulationConfig.sleepEnergyThreshold) body->sleepTime += deltaTime;
                    else body->sleepTime = 0;
                    canSleep = canSleep && body->sleepTime >= simulationConfig.sleepTimeThreshold;
                }
                if (canSleep) {
                    for (auto* body : island.bodies) {
                        body->awake = false; body->velocity = {}; body->angularVelocity = 0;
                        body->force = {}; body->torque = 0;
                    }
                    statistics.sleepingBodyCount += static_cast<std::uint32_t>(island.bodies.size());
                }
            }
        }
        if (useCcd) sweepCcdCorrections(deltaTime);
    }

    // A transient swept impact also participates in the lifecycle, even when
    // its bodies have separated again by the end of this step.
    for (auto manifold : impacts) {
        CanonicalizeManifold(manifold);
        const auto key = ContactKey::From(manifold.A, manifold.B);
        RigidBodyPtr bodyA, bodyB;
        for (const auto& body : bodies) {
            if (body.get() == manifold.A) bodyA = body;
            if (body.get() == manifold.B) bodyB = body;
        }
        publishContact(key, manifold, bodyA, bodyB, true);
    }
    // Sensor overlaps at the end-of-step poses. Only a real overlap counts:
    // no speculative margin, no touch tolerance. Sensor pairs never enter the
    // contact cache, so they are not touching contacts in the statistics.
    if (scratch) {
        auto& sensorPairs = scratch->sensorPairs;
        for (const auto& pair : sensorPairs) {
            CollisionManifold manifold = CheckCollision(pair.first.get(), pair.second.get());
            if (!manifold.hasCollision) continue;
            CanonicalizeManifold(manifold);
            publishContact(ContactKey::From(manifold.A, manifold.B), manifold, pair.first, pair.second, false);
        }
        // getPotentialCollisions() lists every pair that passed filtering.
        potentialCollisions.insert(potentialCollisions.end(), sensorPairs.begin(), sensorPairs.end());
        sensorPairs.clear();
    }
    // Keep the cache entries this step detected (keptStep), then count the
    // touching ones: those whose contact was published this step.
    for (auto it = contactCache.begin(); it != contactCache.end();) {
        if (it->second.keptStep != stamp) {
            it = contactCache.erase(it);
        } else {
            ++it;
        }
    }
    std::size_t touchingNow = 0;
    for (const auto& entry : contactEvents) {
        if (entry.second.activeStep != stamp) continue;
        const auto cached = contactCache.find(entry.first);
        if (cached == contactCache.end()) continue;
        cached->second.touchingStep = stamp;
        ++touchingNow;
    }
    touchingStamp = stamp;
    touchingCount = touchingNow;
    statistics.activeContactCount = static_cast<std::uint32_t>(touchingCount);
    lastStepStatistics = statistics;
    const std::size_t firstEnd = pendingEvents.size();
    for (auto it = contactEvents.begin(); it != contactEvents.end();) {
        if (it->second.activeStep != stamp) {
            it->second.phase = EventPhase::End;
            pendingEvents.push_back(it->second);
            it = contactEvents.erase(it);
        } else {
            ++it;
        }
    }
    SortByPair(pendingEvents, firstEnd);
    dispatchEvents();
    return integratedPlan;
}

namespace {
struct DPoint { double x = 0, y = 0; };
DPoint WorldAnchor(const RigidBody& body, Vector2 local) {
    const Detail::CosSin rotation = Detail::CachedCosSin(double(body.orientation));
    const double c = rotation.cosine, s = rotation.sine;
    return {body.position.x + c * local.x - s * local.y, body.position.y + s * local.x + c * local.y};
}
double Cross(DPoint r, DPoint v) { return r.x * v.y - r.y * v.x; }
// Weight of the trapezoid (half) displacement for a solver velocity change:
// 1 while the change is no larger than the load over the step, 0 from twice
// that on, blended in between.
double ForceLike(double change, double load, double step) {
    const double scale = step * load;
    if (change <= scale) return 1.0;
    return change >= 2 * scale ? 0.0 : 2.0 - change / scale;
}
float StateFloat(double value) {
    if (!std::isfinite(value) || std::abs(value) > std::numeric_limits<float>::max())
        throw std::overflow_error("RigidBody result exceeds finite float range.");
    return static_cast<float>(value);
}
}

void World::stepSpeculative(float deltaTime, SimulationStatistics& statistics,
    std::vector<CollisionManifold>& impacts) {
    const SimulationConfig& config = simulationConfig;
    const double dt = deltaTime;
    // Pairs closer than this count as touching: they raise events and link
    // islands. Resting bodies settle at zero separation, so exact overlap
    // would make their contacts flicker.
    const float touchTolerance = std::max(0.25f * config.penetrationSlop, 1e-6f);

    const bool useCcd = deltaTime > 0 && std::any_of(bodies.begin(), bodies.end(),
        [](const RigidBodyPtr& body) { return body->IsCcdEnabled(); });
    std::vector<Vector2> starts;
    if (useCcd)
        for (const auto& body : bodies) starts.push_back(body->position);

    // Velocities first. Positions stay at the start of the step until the
    // solve is done, so detection below still sees the start-of-step poses.
    std::vector<RigidBody::StepStart> start(bodies.size());
    std::vector<char> integrated(bodies.size(), 0);
    for (std::size_t i = 0; i < bodies.size(); ++i) {
        if (!bodies[i]->IsAwake()) continue;
        ++statistics.integratedBodyCount;
        start[i] = bodies[i]->IntegrateVelocity(deltaTime, config);
        integrated[i] = 1;
    }

    // Each body may close margin of distance this step: a fixed allowance
    // (4 * slop per pair, as in Box2D) plus its own reach over dt.
    std::vector<float> margins(bodies.size(), 0.0f);
    // Bounds computed here are handed to the built-in broad phase, which
    // would otherwise compute the same boxes again.
    std::vector<AABB> knownBounds(bodies.size());
    std::vector<char> knownBound(bodies.size(), 0);
    if (deltaTime > 0) {
        for (std::size_t i = 0; i < bodies.size(); ++i) {
            double margin = 2.0 * config.penetrationSlop;
            if (integrated[i]) {
                const auto& body = *bodies[i];
                const auto& s = start[i];
                const AABB box = body.GetAABB();
                knownBounds[i] = box;
                knownBound[i] = 1;
                double reach = 0;
                for (const double x : {double(box.min.x), double(box.max.x)})
                    for (const double y : {double(box.min.y), double(box.max.y)})
                        reach = std::max(reach, std::hypot(x - body.position.x, y - body.position.y));
                const double speed = std::hypot(double(s.velocity.x), double(s.velocity.y))
                    + std::abs(double(s.angularVelocity)) * reach;
                const double acceleration = std::hypot(s.ax, s.ay) + std::abs(s.alpha) * reach;
                margin += dt * speed + dt * dt * acceleration;
            }
            margins[i] = std::isfinite(margin)
                ? static_cast<float>(std::min(margin, double(std::numeric_limits<float>::max()) / 4)) : 0.0f;
        }
    }

    // An exact SweepAndPrune (not a subclass, which may override the virtual)
    // can take the precomputed bounds.
    IBroadPhase& phase = *broadPhase;
    auto* sweep = typeid(phase) == typeid(SweepAndPrune) ? static_cast<SweepAndPrune*>(&phase) : nullptr;
    potentialCollisions = sweep ? sweep->FindSpeculativeCollisions(bodies, margins, knownBounds, knownBound)
                                : broadPhase->FindSpeculativeCollisions(bodies, margins);
    statistics.broadPhaseCandidateCount = static_cast<std::uint32_t>(potentialCollisions.size());
    if (!scratch) scratch = std::make_unique<StepScratch>();
    SplitCandidates(potentialCollisions, scratch->sensorPairs, jointedPairs);
    statistics.narrowPhaseCandidateCount = static_cast<std::uint32_t>(
        potentialCollisions.size() + scratch->sensorPairs.size());

    auto& entries = scratch->entries;
    auto& constraints = scratch->constraints;
    entries.clear();
    constraints.clear();
    std::vector<char> touchingStart, closing;
    std::vector<std::array<double, 2>> bounces; // target separating speed per point
    constraints.reserve(potentialCollisions.size());
    entries.reserve(potentialCollisions.size());
    touchingStart.reserve(potentialCollisions.size());
    closing.reserve(potentialCollisions.size());
    bounces.reserve(potentialCollisions.size());
    const auto ccdSweeps = [&](const RigidBody& a, const RigidBody& b) {
        return useCcd && (a.IsCcdEnabled() || b.IsCcdEnabled()) && !(a.IsStatic() && b.IsStatic())
            && (a.shape->type == ShapeType::CIRCLE || b.shape->type == ShapeType::CIRCLE);
    };
    const auto pointVelocity = [](const RigidBody* body, DPoint r) -> DPoint {
        return {body->velocity.x - double(body->angularVelocity) * r.y,
                body->velocity.y + double(body->angularVelocity) * r.x};
    };
    for (const auto& pair : potentialCollisions) {
        // CCD sweeps these pairs exactly; they only get the fixed allowance.
        const float margin = deltaTime <= 0 ? 0.0f : ccdSweeps(*pair.first, *pair.second)
            ? float(4.0 * config.penetrationSlop)
            : margins[bodyIndex(pair.first.get())] + margins[bodyIndex(pair.second.get())];
        CollisionManifold manifold = margin > 0 ? CheckCollision(pair.first.get(), pair.second.get(), margin)
                                                : CheckCollision(pair.first.get(), pair.second.get());
        if (!manifold.hasCollision) continue;
        CanonicalizeManifold(manifold);
        const ContactKey key = ContactKey::From(manifold.A, manifold.B);
        auto [contact, inserted] = contactCache.try_emplace(key);
        contact->second.keptStep = stepStamp;
        auto& cache = contact->second.impulses;
        if (!inserted) {
            for (std::uint8_t i = 0; i < cache.contactCount; ++i) {
                cache.contacts[i].impulse.normal *= config.warmStartFactor;
                cache.contacts[i].impulse.tangent *= config.warmStartFactor;
            }
        }
        constraints.push_back(CollisionResolver::PrepareConstraint(manifold, cache, config, true));
        auto& c = constraints.back();
        std::array<double, 2> bounce{0.0, 0.0};
        bool closes = false;
        for (std::uint8_t i = 0; deltaTime > 0 && i < c.pointCount; ++i) {
            auto& p = c.points[i];
            if (p.penetration >= 0) continue;
            // v_n >= -gap/dt: the gap may close over this step, no further.
            const double speculative = double(p.penetration) / dt;
            // Restitution waits until the body has reached the surface: it is
            // applied after the position update, and only if the gap closes
            // during this step (Box2D v3 does the same).
            const Vector2 point = manifold.contactCount > 0 ? manifold.contacts[i].position : manifold.contactPoint;
            const DPoint rA{double(point.x) - c.bodyA->position.x, double(point.y) - c.bodyA->position.y};
            const DPoint rB{double(point.x) - c.bodyB->position.x, double(point.y) - c.bodyB->position.y};
            const DPoint vA = pointVelocity(c.bodyA, rA), vB = pointVelocity(c.bodyB, rB);
            const double normalVelocity = (vB.x - vA.x) * c.normal.x + (vB.y - vA.y) * c.normal.y;
            if (normalVelocity < speculative) {
                closes = true;
                // Bounce off the speed at the moment of contact, not the
                // end-of-step prediction: that one includes the load for the
                // part of the step after contact, and e = 1 would gain energy.
                const auto startMotion = [&](const RigidBody* body, DPoint r) {
                    const auto& s = start[bodyIndex(body)];
                    const double w = s.angularVelocity;
                    return std::array<double, 2>{
                        (double(s.velocity.x) - w * r.y) * c.normal.x + (double(s.velocity.y) + w * r.x) * c.normal.y,
                        (s.ax - s.alpha * r.y) * c.normal.x + (s.ay + s.alpha * r.x) * c.normal.y};
                };
                const auto motionA = startMotion(c.bodyA, rA), motionB = startMotion(c.bodyB, rB);
                const double startVelocity = motionB[0] - motionA[0], acceleration = motionB[1] - motionA[1];
                const double gap = -double(p.penetration);
                const double hit = std::max(-std::sqrt(std::max(
                    startVelocity * startVelocity - 2 * acceleration * gap, 0.0)), normalVelocity);
                const double restitution = std::max(c.bodyA->material.restitution, c.bodyB->material.restitution);
                if (hit < -config.restitutionVelocityThreshold) bounce[i] = -restitution * hit;
            }
            p.velocityBias = speculative;
        }
        bounces.push_back(bounce);
        entries.push_back({key, manifold});
        touchingStart.push_back(manifold.penetration >= -touchTolerance);
        closing.push_back(closes);
    }

    // An awake body about to reach a sleeping one this step wakes it now.
    // Waiting for the touching contact would let the gap close unopposed and
    // the mover sink up to v * dt into the sleeper.
    if (config.enableSleeping) {
        for (std::size_t j = 0; j < constraints.size(); ++j) {
            RigidBody* a = constraints[j].bodyA;
            RigidBody* b = constraints[j].bodyB;
            if (touchingStart[j] || !closing[j] || a->IsStatic() || b->IsStatic() || a->IsAwake() == b->IsAwake())
                continue;
            (a->IsAwake() ? b : a)->Wake();
        }
    }

    // Islands (sleeping, waking, statistics) only follow touching contacts.
    auto islands = buildIslands(constraints, false, &touchingStart);
    statistics.islandCount = static_cast<std::uint32_t>(islands.size());
    statistics.solverIterationCount = config.solverIterations;
    // Island of each body, by body index (static bodies belong to none).
    constexpr std::size_t none = std::numeric_limits<std::size_t>::max();
    std::vector<std::size_t> islandOfBody(bodies.size(), none);
    for (std::size_t k = 0; k < islands.size(); ++k) {
        wakeIsland(islands[k]);
        for (auto* body : islands[k].bodies) islandOfBody[bodyIndex(body)] = k;
    }
    const auto islandOf = [&](const RigidBody* body) {
        const std::size_t island = islandOfBody[bodyIndex(body)];
        if (island == none) throw std::out_of_range("Body has no island.");
        return island;
    };
    // Bodies woken above join this step with their (zero) sleeping velocity,
    // so the solver's velocity change still moves them.
    for (std::size_t i = 0; i < bodies.size(); ++i) {
        if (integrated[i] || !bodies[i]->IsAwake()) continue;
        ++statistics.integratedBodyCount;
        start[i] = bodies[i]->IntegrateVelocity(deltaTime, config);
        integrated[i] = 1;
    }
    std::vector<char> awakeIsland(islands.size(), 0), solved(constraints.size(), 0);
    for (std::size_t k = 0; k < islands.size(); ++k) {
        auto& island = islands[k];
        if (!island.bodies.front()->IsAwake()) {
            statistics.sleepingBodyCount += static_cast<std::uint32_t>(island.bodies.size());
            continue;
        }
        awakeIsland[k] = 1;
        ++statistics.solvedIslandCount;
        statistics.solvedConstraintCount += static_cast<std::uint32_t>(island.contacts.size() + island.joints.size());
        for (auto* c : island.contacts) solved[static_cast<std::size_t>(c - constraints.data())] = 1;
    }
    // Solve groups: awake islands joined by speculative contacts. A speculative
    // pair with a sleeping body is skipped so that it never wakes the body.
    std::vector<std::size_t> parent(islands.size());
    std::iota(parent.begin(), parent.end(), 0);
    const auto root = [&](std::size_t i) {
        while (parent[i] != i) { parent[i] = parent[parent[i]]; i = parent[i]; }
        return i;
    };
    const auto dynamicIsland = [&](const RigidBody* a, const RigidBody* b) {
        return islandOf(a->IsStatic() ? b : a);
    };
    for (std::size_t j = 0; j < constraints.size(); ++j) {
        if (touchingStart[j]) continue;
        const auto& c = constraints[j];
        if ((!c.bodyA->IsStatic() && !c.bodyA->IsAwake()) || (!c.bodyB->IsStatic() && !c.bodyB->IsAwake()))
            continue;
        solved[j] = 1;
        if (!c.bodyA->IsStatic() && !c.bodyB->IsStatic())
            parent[root(islandOf(c.bodyA))] = root(islandOf(c.bodyB));
    }
    struct Group { std::vector<ContactConstraint*> contacts; std::vector<JointPtr> joints; };
    std::vector<Group> groups;
    // Group of each root island; an island that is no awake group's root has none.
    std::vector<std::size_t> groupOfRoot(islands.size(), none);
    const auto groupOf = [&](std::size_t rootIsland) {
        if (groupOfRoot[rootIsland] == none) throw std::out_of_range("Island has no solve group.");
        return groupOfRoot[rootIsland];
    };
    for (std::size_t k = 0; k < islands.size(); ++k) {
        if (!awakeIsland[k]) continue;
        auto& slot = groupOfRoot[root(k)];
        if (slot == none) {
            slot = groups.size();
            groups.emplace_back();
        }
        auto& group = groups[slot];
        group.contacts.insert(group.contacts.end(), islands[k].contacts.begin(), islands[k].contacts.end());
        group.joints.insert(group.joints.end(), islands[k].joints.begin(), islands[k].joints.end());
    }
    for (std::size_t j = 0; j < constraints.size(); ++j)
        if (solved[j] && !touchingStart[j])
            groups[groupOf(root(dynamicIsland(constraints[j].bodyA, constraints[j].bodyB)))]
                .contacts.push_back(&constraints[j]);

    // Solve path of each group. With solverSubsteps > 1, a group of contacts
    // only takes Box2D v3's soft step (substeps with soft contacts and a relax
    // pass) unless one of its contacts would sag past its tolerance under the
    // soft model (CollisionResolver::UpdateRigidSolve: small boxes, heavy
    // loads on a small slop). Those groups, groups with joints (which keep
    // their one-solve-per-step semantics) and every group when
    // solverSubsteps is 1 take the rigid single velocity solve.
    const int substeps = deltaTime > 0 ? config.solverSubsteps : 1;
    const double substep = dt / substeps;
    // A soft contact is stable up to about a quarter of the substep rate.
    const auto softness = substeps > 1
        ? CollisionResolver::MakeSoftness(std::min(double(config.contactHertz), 0.25 / substep),
                                          config.contactDampingRatio, substep)
        : CollisionResolver::Softness{};
    std::vector<char> groupSubstepped(groups.size(), 0), substepped(bodies.size(), 0);
    // Each dynamic body's size bound for UpdateRigidSolve, computed on first use.
    std::vector<double> sizeLimits;
    std::vector<char> sizeLimitKnown;
    const bool chooseSolve = substeps > 1 && softness.biasRate > 0;
    if (chooseSolve) {
        sizeLimits.resize(bodies.size());
        sizeLimitKnown.resize(bodies.size(), 0);
    }
    const auto sizeLimit = [&](const RigidBody* body) {
        if (body->IsStatic()) return 0.0;
        const std::size_t i = bodyIndex(body);
        if (!sizeLimitKnown[i]) {
            sizeLimits[i] = CollisionResolver::SagSizeLimit(*body);
            sizeLimitKnown[i] = 1;
        }
        return sizeLimits[i];
    };
    for (std::size_t g = 0; g < groups.size(); ++g) {
        auto& group = groups[g];
        bool rigid = substeps <= 1 || !group.joints.empty() || group.contacts.empty();
        for (auto* c : group.contacts)
            rigid = (chooseSolve && c->cache
                ? CollisionResolver::UpdateRigidSolve(*c, softness, substeps, sizeLimit(c->bodyA), sizeLimit(c->bodyB))
                : CollisionResolver::UpdateRigidSolve(*c, softness, substeps)) || rigid;
        groupSubstepped[g] = !rigid;
        for (auto* c : group.contacts) {
            CollisionResolver::MatchImpulseSubsteps(*c, rigid ? 1 : substeps);
            if (!rigid)
                for (const RigidBody* body : {c->bodyA, c->bodyB})
                    if (!body->IsStatic()) substepped[bodyIndex(body)] = 1;
        }
    }
    const auto substeppedContact = [&](const ContactConstraint& c) {
        return substepped[bodyIndex(c.bodyA->IsStatic() ? c.bodyB : c.bodyA)] != 0;
    };

    if (std::find(groupSubstepped.begin(), groupSubstepped.end(), 1) != groupSubstepped.end()) {
        // Restitution of points that already overlap at the start of the
        // step is applied after the substeps, like the speculative bounces.
        for (std::size_t j = 0; j < constraints.size(); ++j) {
            const auto& c = constraints[j];
            if (!solved[j] || !substeppedContact(c)) continue;
            for (std::uint8_t i = 0; i < c.pointCount; ++i)
                if (c.points[i].penetration >= 0 && c.points[i].velocityBias > 0)
                    bounces[j][i] = c.points[i].velocityBias;
        }
        // Each substep integrates velocities, warm-starts the contacts and
        // solves them with a soft push-out of overlap, integrates positions,
        // then relaxes (rigid rows, no push-out) so that the push-out leaves
        // no velocity behind. The contacts keep the start-of-step manifold;
        // each row measures its separation at the current poses.
        const double inverseSubstep = 1 / substep;
        const int iterations = std::max(1, config.solverIterations / substeps);
        std::vector<RigidBody::StepStart> sub(bodies.size());
        for (std::size_t i = 0; i < bodies.size(); ++i) {
            if (!substepped[i]) continue;
            bodies[i]->velocity = start[i].velocity;
            bodies[i]->angularVelocity = start[i].angularVelocity;
        }
        for (int k = 0; k < substeps; ++k) {
            for (std::size_t i = 0; i < bodies.size(); ++i) {
                if (!substepped[i]) continue;
                auto& body = *bodies[i];
                auto& u = sub[i];
                u = start[i];
                u.velocity = body.velocity;
                u.angularVelocity = body.angularVelocity;
                // The same load and limits as IntegrateVelocity, over one substep.
                double vx = body.velocity.x + u.ax * substep, vy = body.velocity.y + u.ay * substep;
                double omega = body.angularVelocity + u.alpha * substep;
                if (config.enableLinearVelocityLimit) {
                    const double speed = std::hypot(vx, vy);
                    if (speed > config.maxLinearSpeed) {
                        const double scale = config.maxLinearSpeed / speed;
                        vx *= scale;
                        vy *= scale;
                    }
                }
                if (config.enableAngularVelocityLimit)
                    omega = std::clamp(omega, -double(config.maxAngularSpeed), double(config.maxAngularSpeed));
                body.velocity = {StateFloat(vx), StateFloat(vy)};
                body.angularVelocity = StateFloat(omega);
                u.predictedVelocity = body.velocity;
                u.predictedAngularVelocity = body.angularVelocity;
            }
            for (std::size_t g = 0; g < groups.size(); ++g) {
                if (!groupSubstepped[g]) continue;
                const auto& contacts = groups[g].contacts;
                for (auto* c : contacts) CollisionResolver::WarmStartSubstep(*c);
                for (int iter = 0; iter < iterations; ++iter)
                    for (auto* c : contacts)
                        CollisionResolver::SolveSubstep(*c, inverseSubstep, softness, config.contactPushSpeed,
                                                        true, true);
            }
            for (std::size_t i = 0; i < bodies.size(); ++i) {
                if (!substepped[i]) continue;
                auto& body = *bodies[i];
                const auto& u = sub[i];
                const double dvx = double(body.velocity.x) - u.predictedVelocity.x;
                const double dvy = double(body.velocity.y) - u.predictedVelocity.y;
                const double dw = double(body.angularVelocity) - u.predictedAngularVelocity;
                const double linear = ForceLike(std::hypot(dvx + u.ax * substep, dvy + u.ay * substep),
                                                std::hypot(u.ax, u.ay), substep);
                const double angular = ForceLike(std::abs(dw + u.alpha * substep), std::abs(u.alpha), substep);
                body.IntegratePosition(static_cast<float>(substep), u, dvx * (1 - 0.5 * linear),
                                       dvy * (1 - 0.5 * linear), dw * (1 - 0.5 * angular));
            }
            // The last relax leaves open gaps alone: the next step detects
            // them again, and clamping the approach now would slow a body
            // before its contact (and its bounce) is reached.
            const bool gapRows = k + 1 < substeps;
            for (std::size_t g = 0; g < groups.size(); ++g) {
                if (!groupSubstepped[g]) continue;
                for (auto* c : groups[g].contacts)
                    CollisionResolver::SolveSubstep(*c, inverseSubstep, softness, config.contactPushSpeed,
                                                    false, gapRows);
            }
        }
    }

    // Joints and contacts share one velocity solve, so a chain resting on a
    // box or a ragdoll on the ground gets impulses that satisfy both.
    for (std::size_t g = 0; g < groups.size(); ++g) {
        if (groupSubstepped[g]) continue;
        auto& group = groups[g];
        for (const auto& joint : group.joints) joint->warmStart(config.warmStartFactor);
        for (auto* c : group.contacts) CollisionResolver::WarmStart(*c);
        for (int iter = 0; iter < config.solverIterations; ++iter) {
            for (const auto& joint : group.joints) joint->solveVelocity();
            for (auto* c : group.contacts) CollisionResolver::SolveVelocity(*c);
        }
    }

    // Position update. A solver velocity change no larger than the step's
    // applied load (support, friction, closing a small gap) acts like a
    // force: the body moves by half of it, the trapezoid rule, which keeps
    // resting and sliding bodies on their constant-load path with no friction
    // creep. A larger change is an impact and moves the body by all of it,
    // which is what v_n >= -gap/dt assumed. In between the two blend.
    // Substepped bodies have already moved.
    for (std::size_t i = 0; i < bodies.size(); ++i) {
        if (!integrated[i]) continue;
        auto& body = *bodies[i];
        if (!substepped[i]) {
            const auto& s = start[i];
            const double dvx = double(body.velocity.x) - s.predictedVelocity.x;
            const double dvy = double(body.velocity.y) - s.predictedVelocity.y;
            const double dw = double(body.angularVelocity) - s.predictedAngularVelocity;
            const double linear = ForceLike(std::hypot(dvx + s.ax * dt, dvy + s.ay * dt), std::hypot(s.ax, s.ay), dt);
            const double angular = ForceLike(std::abs(dw + s.alpha * dt), std::abs(s.alpha), dt);
            body.IntegratePosition(deltaTime, s, dvx * (1 - 0.5 * linear), dvy * (1 - 0.5 * linear),
                                   dw * (1 - 0.5 * angular));
        }
        body.orientation = std::remainder(body.orientation, 6.283185307179586f);
    }

    for (std::size_t j = 0; j < constraints.size(); ++j) {
        if (!solved[j]) continue;
        auto& c = constraints[j];
        const bool substepContact = substeppedContact(c);
        for (std::uint8_t i = 0; i < c.pointCount; ++i) {
            auto& p = c.points[i];
            // A substepped point counts if any substep carried it.
            if (bounces[j][i] <= 0 || (substepContact ? p.maxNormalImpulse : p.impulse.normal) <= 0) continue;
            const DPoint a = WorldAnchor(*c.bodyA, p.localAnchorA), b = WorldAnchor(*c.bodyB, p.localAnchorB);
            const DPoint point{0.5 * (a.x + b.x), 0.5 * (a.y + b.y)};
            const DPoint rA{point.x - c.bodyA->position.x, point.y - c.bodyA->position.y};
            const DPoint rB{point.x - c.bodyB->position.x, point.y - c.bodyB->position.y};
            const DPoint vA = pointVelocity(c.bodyA, rA), vB = pointVelocity(c.bodyB, rB);
            const double normalVelocity = (vB.x - vA.x) * c.normal.x + (vB.y - vA.y) * c.normal.y;
            const double total = std::max(p.impulse.normal + p.normalMass * (bounces[j][i] - normalVelocity), 0.0);
            const double change = total - p.impulse.normal;
            p.impulse.normal = total;
            if (c.cache) c.cache->contacts[i].impulse.normal = total;
            const DPoint impulse{c.normal.x * change, c.normal.y * change};
            for (const auto& [body, r, sign] : {std::tuple<RigidBody*, DPoint, double>{c.bodyA, rA, -1.0},
                                                {c.bodyB, rB, 1.0}}) {
                if (body->IsStatic()) continue;
                body->velocity = {static_cast<float>(body->velocity.x + sign * body->inverseMass * impulse.x),
                                  static_cast<float>(body->velocity.y + sign * body->inverseMass * impulse.y)};
                body->angularVelocity = static_cast<float>(
                    body->angularVelocity + sign * body->inverseInertia * Cross(r, impulse));
            }
        }
    }

    if (useCcd) impacts = advanceCcd(starts, deltaTime, statistics);

    // Position correction and the contact lifecycle use the integrated poses,
    // as the legacy order did. The start-of-step manifold has done its job
    // (the velocity solve); for spinning bodies its normal and points are
    // stale.
    // Entry of each start-of-step key; only pairs where nothing moved need
    // it, so it is built on first use.
    std::unordered_map<ContactKey, std::size_t, ContactKeyHash> entryOf;
    bool entryOfBuilt = false;
    auto& ended = scratch->ended;
    auto& endCaches = scratch->endCaches;
    auto& endConstraints = scratch->endConstraints;
    ended.clear();
    endCaches.clear();
    endConstraints.clear();
    ended.reserve(potentialCollisions.size());
    endCaches.reserve(potentialCollisions.size());
    endConstraints.reserve(potentialCollisions.size());
    std::vector<std::vector<ContactConstraint*>> positionContacts(groups.size());
    for (const auto& pair : potentialCollisions) {
        RigidBody* a = pair.first.get();
        RigidBody* b = pair.second.get();
        const bool moving = a->IsAwake() || b->IsAwake();
        if (!moving) {
            // Nothing moved: a sleeping contact keeps its start-of-step manifold.
            if (!entryOfBuilt) {
                for (std::size_t j = 0; j < entries.size(); ++j) entryOf.emplace(entries[j].key, j);
                entryOfBuilt = true;
            }
            const auto key = ContactKey::From(a, b);
            const auto found = entryOf.find(key);
            if (found != entryOf.end() && touchingStart[found->second])
                ended.push_back({key, entries[found->second].manifold, &pair});
            continue;
        }
        CollisionManifold manifold = CheckCollision(a, b, touchTolerance);
        if (!manifold.hasCollision || manifold.penetration < -touchTolerance) continue;
        CanonicalizeManifold(manifold);
        ended.push_back({ContactKey::From(manifold.A, manifold.B), manifold, &pair});
        const RigidBody* dynamic = a->IsAwake() ? a : b;
        const RigidBody* other = dynamic == a ? b : a;
        if (!other->IsStatic() && !other->IsAwake()) continue; // Woken next step via its Begin event.
        // A CCD impact can wake a sleeping body after the solve groups were
        // formed; its island has no group, so it joins its partner's group,
        // or (both woken that way) is left to next step's solve.
        std::size_t group = groupOfRoot[root(islandOf(dynamic))];
        if (group == none && !other->IsStatic()) group = groupOfRoot[root(islandOf(other))];
        if (group == none) continue;
        endCaches.emplace_back();
        endConstraints.push_back(CollisionResolver::PrepareConstraint(manifold, endCaches.back(), config, true));
        positionContacts[group].push_back(&endConstraints.back());
    }

    for (std::size_t g = 0; g < groups.size(); ++g) {
        for (int iter = 0; iter < config.solverIterations; ++iter) {
            bool done = true;
            for (auto* c : positionContacts[g]) done = CollisionResolver::SolvePosition(*c) && done;
            for (const auto& joint : groups[g].joints)
                done = joint->solvePosition(config.penetrationSlop, config.maxPositionCorrection) && done;
            if (done) break;
        }
    }
    if (useCcd) sweepCcdCorrections(deltaTime);

    for (std::size_t k = 0; k < islands.size(); ++k) {
        if (!awakeIsland[k] || !config.enableSleeping || deltaTime <= 0) continue;
        auto& island = islands[k];
        bool canSleep = std::none_of(island.joints.begin(), island.joints.end(),
            [](const JointPtr& joint) { return joint->preventsSleeping(); });
        for (auto* body : island.bodies) {
            const float specificEnergy = 0.5f*(body->velocity.magnitudeSquared()
                + body->inertia*body->inverseMass*body->angularVelocity*body->angularVelocity);
            if (specificEnergy <= config.sleepEnergyThreshold) body->sleepTime += deltaTime;
            else body->sleepTime = 0;
            canSleep = canSleep && body->sleepTime >= config.sleepTimeThreshold;
        }
        if (canSleep) {
            for (auto* body : island.bodies) {
                body->awake = false; body->velocity = {}; body->angularVelocity = 0;
                body->force = {}; body->torque = 0;
            }
            statistics.sleepingBodyCount += static_cast<std::uint32_t>(island.bodies.size());
        }
    }

    for (auto& contact : ended) {
        // Listeners see touching contacts only; a residual gap reads as zero.
        auto& manifold = contact.manifold;
        manifold.penetration = 0;
        for (std::uint8_t i = 0; i < manifold.contactCount; ++i) {
            manifold.contacts[i].penetration = std::max(manifold.contacts[i].penetration, 0.0f);
            manifold.penetration = std::max(manifold.penetration, manifold.contacts[i].penetration);
        }
        statistics.resolvedContactCount += std::max<std::uint32_t>(manifold.contactCount, 1);
        publishContact(contact.key, manifold, contact.pair->first, contact.pair->second, false);
    }
}

bool World::pairCollides(const CollisionPair& pair) const {
    // Sensor pairs never become contacts or CCD sweeps; SplitCandidates
    // routes them to the end-of-step overlap test instead.
    return PassesFilter(pair, jointedPairs) && !HasSensor(pair);
}

std::size_t World::findBodyIndex(const RigidBody* body) const {
    if (!body) return bodies.size();
    const std::size_t hint = body->worldIndex;
    if (hint < bodies.size() && bodies[hint].get() == body) return hint;
    for (std::size_t i = 0; i < bodies.size(); ++i)
        if (bodies[i].get() == body) return i;
    return bodies.size();
}

std::size_t World::bodyIndex(const RigidBody* body) const {
    const std::size_t index = findBodyIndex(body);
    if (index == bodies.size()) throw std::out_of_range("Body is not in this world.");
    return index;
}

bool World::publishContact(const ContactKey& key, const CollisionManifold& manifold,
    const RigidBodyPtr& bodyA, const RigidBodyPtr& bodyB, bool skipIfActive) {
    const bool sensor = bodyA && bodyB && (bodyA->IsSensor() || bodyB->IsSensor());
    auto [it, inserted] = contactEvents.try_emplace(key);
    PendingEvent& event = it->second;
    if (!inserted && skipIfActive && event.activeStep == stepStamp) return false;
    bool begins = inserted;
    if (!inserted && event.event.sensor != sensor) {
        // SetSensor changed the pair's kind: end the old lifecycle first.
        event.phase = EventPhase::End;
        pendingEvents.push_back(event);
        begins = true;
    }
    event.phase = begins ? EventPhase::Begin : EventPhase::Persist;
    event.event = {key.first, key.second, manifold.normal, manifold.penetration,
        manifold.contacts, manifold.contactCount};
    event.event.sensor = sensor;
    event.manifold = manifold;
    event.bodyA = bodyA;
    event.bodyB = bodyB;
    event.activeStep = stepStamp;
    pendingEvents.push_back(event);
    return true;
}

// Capsule CCD covers only translating circles against static capsules; any
// other requested capsule pair is rejected before a step advances state.
bool World::SupportsCapsuleCcd(const RigidBody& a, const RigidBody& b) {
    const auto circleOnStaticCapsule = [](const RigidBody& mover, const RigidBody& target) {
        return mover.shape->type == ShapeType::CIRCLE && target.shape->type == ShapeType::CAPSULE
            && target.IsStatic();
    };
    return circleOnStaticCapsule(a, b) || circleOnStaticCapsule(b, a);
}

void World::validateCcdSupport() const {
    // Only pairs with a CCD body are checked, so scan from each CCD body
    // (O(ccd bodies * bodies)) rather than over every pair. The test below is
    // symmetric in a and b; a pair of two CCD bodies is simply seen twice.
    for (std::size_t i=0; i<bodies.size(); ++i) {
        if (!bodies[i]->IsCcdEnabled()) continue;
        for (std::size_t j=0; j<bodies.size(); ++j) {
            if (j == i) continue;
            const auto& a = *bodies[i]; const auto& b = *bodies[j];
            // Sensor pairs are never swept, so any shape pairing is fine.
            if ((a.IsStatic() && b.IsStatic()) || !a.CanCollideWith(b) || a.IsSensor() || b.IsSensor()) continue;
            if ((a.shape->type == ShapeType::CAPSULE || b.shape->type == ShapeType::CAPSULE)
                && !SupportsCapsuleCcd(a, b))
                throw std::invalid_argument(
                    "Capsule continuous collision supports only circles against static capsules.");
        }
    }
}

namespace {
// Translational sweep of one pair from the bodies' current poses; the normal
// points from a to b. Pairs without a circle (pure polygons) are not swept.
SweepHit SweepPair(const RigidBody& a, Vector2 da, const RigidBody& b, Vector2 db, bool capsuleSupported) {
    SweepHit hit;
    const auto typeA = a.shape->type, typeB = b.shape->type;
    if (typeA == ShapeType::CAPSULE || typeB == ShapeType::CAPSULE) {
        const bool firstCircle = typeA == ShapeType::CIRCLE;
        const RigidBody& circle = firstCircle ? a : b;
        const RigidBody& capsuleBody = firstCircle ? b : a;
        const auto* capsule = dynamic_cast<const Capsule*>(capsuleBody.shape.get());
        if (!capsuleSupported || !capsule)
            throw std::logic_error("Unsupported capsule continuous collision reached the sweep.");
        const Vector2 axis = Matrix2x2::rotation(capsuleBody.orientation) * Vector2{1, 0};
        hit = SweepCircleCapsule(circle.position, firstCircle ? da : db, circle.shape->GetRadius(),
            capsuleBody.position - axis*capsule->GetHalfLength(),
            capsuleBody.position + axis*capsule->GetHalfLength(), capsule->GetRadius(),
            firstCircle ? db : da);
        if (!firstCircle) hit.normal = hit.normal*-1.0f;
    } else if (typeA == ShapeType::CIRCLE && typeB == ShapeType::CIRCLE) {
        hit = SweepCircleCircle(a.position, da, a.shape->GetRadius(), b.position, db, b.shape->GetRadius());
    } else if (typeA == ShapeType::CIRCLE || typeB == ShapeType::CIRCLE) {
        const bool firstCircle = typeA == ShapeType::CIRCLE;
        const RigidBody& circle = firstCircle ? a : b;
        const RigidBody& polygon = firstCircle ? b : a;
        std::vector<Vector2> vertices;
        const auto rotation = Matrix2x2::rotation(polygon.orientation);
        for (const auto& v : static_cast<const Polygon*>(polygon.shape.get())->getVertices())
            vertices.push_back(polygon.position + rotation*v);
        // The local outline was validated when the Polygon was built; the
        // rounded world copy may not pass that check again (it is swept as is).
        hit = Detail::SweepCirclePolygonRounded(circle.position, firstCircle ? da : db, circle.shape->GetRadius(),
            vertices, firstCircle ? db : da);
        if (!firstCircle) hit.normal = hit.normal*-1.0f;
    }
    return hit;
}
}

std::vector<CollisionManifold> World::advanceCcd(const std::vector<Vector2>& starts,
    float deltaTime, SimulationStatistics& statistics) {
    if (!scratch) scratch = std::make_unique<StepScratch>();
    // Every return leaves the poses sweepCcdCorrections starts from. Sized
    // here so that the recording itself cannot throw.
    scratch->ccdPositions.resize(bodies.size());
    struct RecordPositions {
        World& world;
        ~RecordPositions() {
            for (std::size_t i = 0; i < world.bodies.size(); ++i)
                world.scratch->ccdPositions[i] = world.bodies[i]->position;
        }
    } record{*this};
    std::vector<Vector2> motion;
    for (std::size_t i=0; i<bodies.size(); ++i) {
        motion.push_back((bodies[i]->position-starts[i])/deltaTime);
        bodies[i]->position = starts[i];
    }
    std::vector<CollisionManifold> impacts;
    float remaining = deltaTime;
    for (int iteration=0; iteration<simulationConfig.maximumCcdImpacts; ++iteration) {
        SweepHit earliest;
        std::size_t hitA = 0, hitB = 0;
        for (std::size_t i=0; i<bodies.size(); ++i) {
            for (std::size_t j=i+1; j<bodies.size(); ++j) {
                const auto& a = bodies[i]; const auto& b = bodies[j];
                if ((!a->IsCcdEnabled() && !b->IsCcdEnabled()) ||
                    (a->IsStatic() && b->IsStatic()) || !pairCollides({a, b})) continue;
                const SweepHit hit = SweepPair(*a, motion[i]*remaining, *b, motion[j]*remaining,
                    SupportsCapsuleCcd(*a, *b));
                // Ignore resting/separating pairs, including initial overlap.
                if (hit.hit && (motion[j]-motion[i]).dot(hit.normal) < -1e-6f &&
                    (!earliest.hit || hit.fraction < earliest.fraction)) {
                    earliest = hit; hitA = i; hitB = j;
                }
            }
        }
        const float advance = remaining*(earliest.hit ? earliest.fraction : 1.0f);
        for (std::size_t i=0; i<bodies.size(); ++i)
            bodies[i]->position = bodies[i]->position + motion[i]*advance;
        remaining -= advance;
        if (!earliest.hit) return impacts;
        CollisionManifold manifold;
        manifold.A = bodies[hitA].get(); manifold.B = bodies[hitB].get();
        manifold.hasCollision = true;
        manifold.normal = earliest.normal;
        manifold.contactPoint = earliest.point;
        manifold.contactCount = 1;
        manifold.contacts[0] = {earliest.point, 0, 0x60000001u};
        bodies[hitA]->Wake(); bodies[hitB]->Wake();
        ContactImpulseCache cache;
        auto constraint = CollisionResolver::PrepareConstraint(manifold, cache, simulationConfig, true);
        for (int i=0; i<simulationConfig.solverIterations; ++i)
            CollisionResolver::SolveVelocity(constraint);
        impacts.push_back(manifold);
        ++statistics.ccdImpactCount;
        motion[hitA] = bodies[hitA]->IsStatic() ? Vector2{} : bodies[hitA]->velocity;
        motion[hitB] = bodies[hitB]->IsStatic() ? Vector2{} : bodies[hitB]->velocity;
        if (remaining <= 0) return impacts;
    }
    // Stay at the last safe time when the work budget is exhausted.
    statistics.ccdIterationLimitReached = true;
    return impacts;
}

void World::sweepCcdCorrections(float deltaTime) {
    // Joint and contact position correction (and the legacy friction
    // displacement) move bodies after advanceCcd has swept the step. A rigid
    // joint pulling toward a body beyond a wall would otherwise carry a CCD
    // circle through it without any velocity. Each moved body sweeps its
    // correction against the static bodies of its CCD pairs and stops at the
    // first one it would enter, as advanceCcd does; corrections that only
    // separate or slide along a surface pass unchanged.
    if (!scratch || deltaTime <= 0) return;
    const auto& from = scratch->ccdPositions;
    if (from.size() != bodies.size()) return;
    // A pair is swept only if one side has CCD; without a CCD static body,
    // only CCD bodies need the scan over the statics.
    bool staticCcd = false;
    for (const auto& body : bodies) staticCcd = staticCcd || (body->IsStatic() && body->IsCcdEnabled());
    for (std::size_t i = 0; i < bodies.size(); ++i) {
        const auto& body = bodies[i];
        if (body->IsStatic() || (!staticCcd && !body->IsCcdEnabled())) continue;
        const Vector2 correction = body->position - from[i];
        if (correction.x == 0 && correction.y == 0) continue;
        const Vector2 motion = correction/deltaTime;
        bool hit = false;
        float earliest = 1;
        const Vector2 corrected = body->position;
        body->position = from[i];
        for (std::size_t j = 0; j < bodies.size(); ++j) {
            const auto& other = bodies[j];
            if (!other->IsStatic() || (!body->IsCcdEnabled() && !other->IsCcdEnabled())
                || !pairCollides({body, other})) continue;
            const SweepHit sweep = SweepPair(*body, correction, *other, {}, SupportsCapsuleCcd(*body, *other));
            // Same approach test as advanceCcd, with the static body at rest.
            if (sweep.hit && (motion*-1.0f).dot(sweep.normal) < -1e-6f && sweep.fraction < earliest) {
                hit = true;
                earliest = sweep.fraction;
            }
        }
        body->position = hit ? from[i] + correction*earliest : corrected;
    }
}

void World::endContacts(std::uint64_t bodyId) {
    const std::size_t firstEnd = pendingEvents.size();
    for (auto it = contactEvents.begin(); it != contactEvents.end();) {
        if (it->first.contains(bodyId)) {
            // A sensor overlap holds nothing up, so ending it wakes nobody.
            if (!it->second.event.sensor) { it->second.bodyA->Wake(); it->second.bodyB->Wake(); }
            it->second.phase = EventPhase::End;
            pendingEvents.push_back(it->second);
            it = contactEvents.erase(it);
        } else {
            ++it;
        }
    }
    SortByPair(pendingEvents, firstEnd);
}

void World::dispatchEvents() {
    if (dispatchingEvents) return;
    dispatchingEvents = true;
    try {
        for (std::size_t i = 0; i < pendingEvents.size(); ++i) {
            // Copy before calling user code: callbacks may append end events.
            const PendingEvent event = pendingEvents[i];
            const auto listeners = collisionListeners;
            for (ICollisionListener* listener : listeners) {
                if (std::find(collisionListeners.begin(), collisionListeners.end(),
                        listener) == collisionListeners.end()) continue;
                switch (event.phase) {
                case EventPhase::Begin: listener->onCollisionBegin(event.event); break;
                case EventPhase::Persist: listener->onCollisionPersist(event.event); break;
                case EventPhase::End: listener->onCollisionEnd(event.event); break;
                }
                if (event.phase != EventPhase::End &&
                    std::find(collisionListeners.begin(), collisionListeners.end(),
                        listener) != collisionListeners.end()) {
                    listener->onCollision(event.manifold);
                }
            }
        }
    } catch (...) {
        pendingEvents.clear();
        dispatchingEvents = false;
        throw;
    }
    pendingEvents.clear();
    dispatchingEvents = false;
}

void World::addJoint(JointPtr joint) {
    requireMutationAllowed();
    if (!joint) throw std::invalid_argument("Joint cannot be null.");
    for (const auto& body : {joint->getBodyA(), joint->getBodyB()}) {
        if (std::find(bodies.begin(), bodies.end(), body) == bodies.end())
            throw std::invalid_argument("Joint bodies must belong to this world.");
    }
    if (std::find(joints.begin(), joints.end(), joint) == joints.end()) {
        joint->getBodyA()->Wake(); joint->getBodyB()->Wake();
        joints.push_back(std::move(joint));
    }
}

void World::removeJoint(const JointPtr& joint) {
    requireMutationAllowed();
    if (std::find(joints.begin(), joints.end(), joint) != joints.end()) {
        joint->getBodyA()->Wake(); joint->getBodyB()->Wake();
        joints.erase(std::remove(joints.begin(), joints.end(), joint), joints.end());
    }
}

std::vector<World::Island> World::buildIslands(std::vector<ContactConstraint>& constraints, bool previousContacts,
    const std::vector<char>* connects) {
    const auto included = [&](const ContactConstraint& c) {
        return !connects || (*connects)[static_cast<std::size_t>(&c - constraints.data())];
    };
    // Dynamic-body index of each body (by its index in `bodies`).
    constexpr std::size_t none = std::numeric_limits<std::size_t>::max();
    std::vector<std::size_t> dynamicIndex(bodies.size(), none);
    std::vector<RigidBody*> dynamicBodies;
    for (std::size_t i = 0; i < bodies.size(); ++i) if (!bodies[i]->IsStatic()) {
        dynamicIndex[i] = dynamicBodies.size(); dynamicBodies.push_back(bodies[i].get());
    }
    const auto indexOf = [&](const RigidBody* body) {
        const std::size_t i = findBodyIndex(body);
        return i < bodies.size() ? dynamicIndex[i] : none;
    };
    std::vector<std::size_t> parents(dynamicBodies.size());
    std::iota(parents.begin(), parents.end(), 0);
    auto root = [&](std::size_t i) {
        while (parents[i] != i) { parents[i] = parents[parents[i]]; i = parents[i]; }
        return i;
    };
    auto connect = [&](RigidBody* a, RigidBody* b) {
        const std::size_t ia = indexOf(a), ib = indexOf(b);
        if (ia != none && ib != none) parents[root(ib)] = root(ia);
    };
    for (auto& c : constraints) if (included(c)) connect(c.bodyA, c.bodyB);
    for (const auto& joint : joints) connect(joint->getBodyA().get(), joint->getBodyB().get());
    // Sensor overlaps never link islands.
    if (previousContacts) for (const auto& entry : contactEvents)
        if (!entry.second.event.sensor) connect(entry.second.bodyA.get(), entry.second.bodyB.get());
    std::vector<std::size_t> islandIndex(dynamicBodies.size(), none); // by root
    std::vector<Island> result;
    for (std::size_t i=0; i<dynamicBodies.size(); ++i) {
        auto& slot = islandIndex[root(i)];
        if (slot == none) {
            slot = result.size();
            result.emplace_back();
        }
        result[slot].bodies.push_back(dynamicBodies[i]);
    }
    auto islandFor = [&](RigidBody* a, RigidBody* b) -> Island& {
        const auto index = indexOf(a->IsStatic() ? b : a);
        if (index == none) throw std::out_of_range("Constraint has no dynamic body in this world.");
        return result[islandIndex[root(index)]];
    };
    for (auto& c : constraints) if (included(c) && (!c.bodyA->IsStatic() || !c.bodyB->IsStatic()))
        islandFor(c.bodyA, c.bodyB).contacts.push_back(&c);
    for (const auto& joint : joints)
        islandFor(joint->getBodyA().get(), joint->getBodyB().get()).joints.push_back(joint);
    return result;
}

void World::wakeIsland(Island& island) {
    const bool wake = !simulationConfig.enableSleeping || std::any_of(island.bodies.begin(),
        island.bodies.end(), [](const RigidBody* body) { return body->IsAwake(); });
    if (wake) for (auto* body : island.bodies) if (!body->IsAwake()) body->Wake();
}

const std::vector<RigidBodyPtr>& World::getBodies() const {
    return bodies;
}

const std::vector<CollisionPair>& World::getPotentialCollisions() const {
    return potentialCollisions;
}

const std::vector<ForceRegistration>& World::getForceRegistry() const {
    return forceRegistry;
}

const std::vector<std::unique_ptr<IForceGenerator>>& World::getUniversalForceRegistry() const {
    return universalForceRegistry;
}

const std::vector<ParticleSystemPtr>& World::getParticleSystems() const {
    return particleSystems;
}

std::size_t World::getPersistentContactCount() const {
    return touchingCount;
}

const SimulationConfig& World::getSimulationConfig() const {
    return simulationConfig;
}

const SimulationStatistics& World::getLastStepStatistics() const {
    return lastStepStatistics;
}

}

#include "physics/core/world.h"
#include "physics/core/collisions/collision_resolver.h"
#include "physics/core/collisions/broad_phase/sweep_and_prune.h"
#include "physics/core/collisions/continuous_collision.h"
#include "physics/math/matrix2x2.h"
#include <utility>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <unordered_set>
#include <numeric>

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
}

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

void World::step(float deltaTime) {
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
    SimulationStatistics statistics;
    potentialCollisions.clear();
    std::unordered_set<ContactKey, ContactKeyHash> activeContacts;

    for (const ParticleSystemPtr& system : particleSystems) {
        statistics.integratedParticleCount += static_cast<std::uint32_t>(system->size());
        system->step(deltaTime);
    }

    std::vector<ContactConstraint> previousConstraints;
    for (const auto& entry : contactEvents) {
        const auto& event = entry.second;
        if (event.bodyA->contactWakeRequested || event.bodyB->contactWakeRequested) {
            event.bodyA->Wake(); event.bodyB->Wake();
        }
    }
    for (const auto& joint : joints) {
        if (joint->getBodyA()->contactWakeRequested || joint->getBodyB()->contactWakeRequested) {
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
            body->Integrate(deltaTime, simulationConfig);

            body->orientation = std::remainder(body->orientation, 6.283185307179586f);
        }
    }

    const auto impacts = useCcd ? advanceCcd(starts, deltaTime, statistics)
        : std::vector<CollisionManifold>{};

    // Detect contacts once, then solve the prepared constraints in distinct
    // velocity and position phases.
    potentialCollisions = broadPhase->FindPotentialCollisions(bodies);
    statistics.broadPhaseCandidateCount = static_cast<std::uint32_t>(
        potentialCollisions.size()
    );
    potentialCollisions.erase(
        std::remove_if(
            potentialCollisions.begin(),
            potentialCollisions.end(),
            [](const CollisionPair& pair) {
                return !pair.first
                    || !pair.second
                    || !pair.first->CanCollideWith(*pair.second);
            }
        ),
        potentialCollisions.end()
    );
    statistics.narrowPhaseCandidateCount = static_cast<std::uint32_t>(
        potentialCollisions.size()
    );

    std::vector<ContactConstraint> constraints;
    constraints.reserve(potentialCollisions.size());
    for (const auto& pair : potentialCollisions) {
        CollisionManifold manifold = CheckCollision(
            pair.first.get(),
            pair.second.get()
        );
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
        activeContacts.insert(key);

        if (!inserted) {
            for (std::uint8_t i = 0;
                 i < contact->second.contactCount;
                 ++i) {
                contact->second.contacts[i].impulse.normal *=
                    simulationConfig.warmStartFactor;
                contact->second.contacts[i].impulse.tangent *=
                    simulationConfig.warmStartFactor;
            }
        }

        constraints.push_back(CollisionResolver::PrepareConstraint(
            manifold,
            contact->second,
            simulationConfig
        ));
        PendingEvent event;
        event.phase = contactEvents.count(key) ? EventPhase::Persist : EventPhase::Begin;
        event.event = {key.first, key.second, manifold.normal, manifold.penetration,
            manifold.contacts, manifold.contactCount};
        event.manifold = manifold;
        event.bodyA = pair.first;
        event.bodyB = pair.second;
        contactEvents.insert_or_assign(key, event);
        pendingEvents.push_back(event);
    }

    auto islands = buildIslands(constraints);
    statistics.islandCount = static_cast<std::uint32_t>(islands.size());
    statistics.solverIterationCount = simulationConfig.solverIterations;
    for (auto& island : islands) {
        wakeIsland(island);
        if (!island.bodies.front()->IsAwake()) {
            statistics.sleepingBodyCount += static_cast<std::uint32_t>(island.bodies.size());
            continue;
        }
        ++statistics.solvedIslandCount;
        statistics.solvedConstraintCount += static_cast<std::uint32_t>(island.contacts.size()+island.joints.size());
        for (auto* constraint : island.contacts) CollisionResolver::WarmStart(*constraint);
        for (int iter=0; iter<simulationConfig.solverIterations; ++iter) {
            for (auto* constraint : island.contacts) CollisionResolver::SolveVelocity(*constraint);
            for (const auto& joint : island.joints) joint->solveVelocity();
        }
        for (int iter=0; iter<simulationConfig.solverIterations; ++iter) {
            bool solved = true;
            for (auto* constraint : island.contacts)
                solved = CollisionResolver::SolvePosition(*constraint) && solved;
            for (const auto& joint : island.joints)
                solved = joint->solvePosition(simulationConfig.penetrationSlop,
                    simulationConfig.maxPositionCorrection) && solved;
            if (solved) break;
        }
        if (simulationConfig.enableSleeping && deltaTime > 0) {
            bool canSleep = true;
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

    // A transient swept impact also participates in the lifecycle, even when
    // its bodies have separated again by the end of this step.
    for (auto manifold : impacts) {
        CanonicalizeManifold(manifold);
        const auto key = ContactKey::From(manifold.A, manifold.B);
        if (!activeContacts.insert(key).second) continue;
        PendingEvent event;
        event.phase = contactEvents.count(key) ? EventPhase::Persist : EventPhase::Begin;
        event.event = {key.first, key.second, manifold.normal, manifold.penetration,
            manifold.contacts, manifold.contactCount};
        event.manifold = manifold;
        for (const auto& body : bodies) {
            if (body.get() == manifold.A) event.bodyA = body;
            if (body.get() == manifold.B) event.bodyB = body;
        }
        contactEvents.insert_or_assign(key, event);
        pendingEvents.push_back(event);
    }
    for (auto it = contactCache.begin(); it != contactCache.end();) {
        if (activeContacts.find(it->first) == activeContacts.end()) {
            it = contactCache.erase(it);
        } else {
            ++it;
        }
    }
    statistics.activeContactCount = static_cast<std::uint32_t>(contactCache.size());
    lastStepStatistics = statistics;
    for (auto it = contactEvents.begin(); it != contactEvents.end();) {
        if (activeContacts.count(it->first) == 0) {
            it->second.phase = EventPhase::End;
            pendingEvents.push_back(it->second);
            it = contactEvents.erase(it);
        } else {
            ++it;
        }
    }
    dispatchEvents();
}

std::vector<CollisionManifold> World::advanceCcd(const std::vector<Vector2>& starts,
    float deltaTime, SimulationStatistics& statistics) {
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
                    (a->IsStatic() && b->IsStatic()) || !a->CanCollideWith(*b)) continue;
                SweepHit hit;
                if (a->shape->type == ShapeType::CIRCLE && b->shape->type == ShapeType::CIRCLE) {
                    hit = SweepCircleCircle(a->position, motion[i]*remaining, a->shape->GetRadius(),
                        b->position, motion[j]*remaining, b->shape->GetRadius());
                } else if (a->shape->type == ShapeType::CIRCLE || b->shape->type == ShapeType::CIRCLE) {
                    const bool firstCircle = a->shape->type == ShapeType::CIRCLE;
                    const auto& circle = firstCircle ? a : b;
                    const auto& polygon = firstCircle ? b : a;
                    std::vector<Vector2> vertices;
                    const auto rotation = Matrix2x2::rotation(polygon->orientation);
                    for (const auto& v : static_cast<const Polygon*>(polygon->shape.get())->getVertices())
                        vertices.push_back(polygon->position + rotation*v);
                    hit = SweepCirclePolygon(circle->position,
                        motion[firstCircle ? i : j]*remaining, circle->shape->GetRadius(),
                        vertices, motion[firstCircle ? j : i]*remaining);
                    if (!firstCircle) hit.normal = hit.normal*-1.0f;
                }
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
        auto constraint = CollisionResolver::PrepareConstraint(manifold, cache, simulationConfig);
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

void World::endContacts(std::uint64_t bodyId) {
    for (auto it = contactEvents.begin(); it != contactEvents.end();) {
        if (it->first.contains(bodyId)) {
            it->second.bodyA->Wake(); it->second.bodyB->Wake();
            it->second.phase = EventPhase::End;
            pendingEvents.push_back(it->second);
            it = contactEvents.erase(it);
        } else {
            ++it;
        }
    }
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

std::vector<World::Island> World::buildIslands(std::vector<ContactConstraint>& constraints, bool previousContacts) {
    std::unordered_map<RigidBody*, std::size_t> indices;
    std::vector<RigidBody*> dynamicBodies;
    for (const auto& body : bodies) if (!body->IsStatic()) {
        indices[body.get()] = dynamicBodies.size(); dynamicBodies.push_back(body.get());
    }
    std::vector<std::size_t> parents(dynamicBodies.size());
    std::iota(parents.begin(), parents.end(), 0);
    auto root = [&](std::size_t i) {
        while (parents[i] != i) { parents[i] = parents[parents[i]]; i = parents[i]; }
        return i;
    };
    auto connect = [&](RigidBody* a, RigidBody* b) {
        if (indices.count(a) && indices.count(b)) parents[root(indices.at(b))] = root(indices.at(a));
    };
    for (auto& c : constraints) connect(c.bodyA, c.bodyB);
    for (const auto& joint : joints) connect(joint->getBodyA().get(), joint->getBodyB().get());
    if (previousContacts) for (const auto& entry : contactEvents)
        connect(entry.second.bodyA.get(), entry.second.bodyB.get());
    std::unordered_map<std::size_t, std::size_t> islandIndex;
    std::vector<Island> result;
    for (std::size_t i=0; i<dynamicBodies.size(); ++i) {
        const auto r = root(i);
        auto [it, inserted] = islandIndex.emplace(r, result.size());
        if (inserted) result.emplace_back();
        result[it->second].bodies.push_back(dynamicBodies[i]);
    }
    auto islandFor = [&](RigidBody* a, RigidBody* b) -> Island& {
        const auto index = indices.at(a->IsStatic() ? b : a);
        return result[islandIndex.at(root(index))];
    };
    for (auto& c : constraints) if (!c.bodyA->IsStatic() || !c.bodyB->IsStatic())
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
    return contactCache.size();
}

const SimulationConfig& World::getSimulationConfig() const {
    return simulationConfig;
}

const SimulationStatistics& World::getLastStepStatistics() const {
    return lastStepStatistics;
}

}

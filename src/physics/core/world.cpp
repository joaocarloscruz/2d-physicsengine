#include "physics/core/world.h"
#include "physics/core/collisions/collision_resolver.h"
#include "physics/core/collisions/broad_phase/sweep_and_prune.h"
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

void World::addBody(RigidBodyPtr body) {
    if (!body) throw std::invalid_argument("Body cannot be null.");
    if (body && std::find(bodies.begin(), bodies.end(), body) == bodies.end()) {
        bodies.push_back(body);
    }
}

void World::removeBody(RigidBodyPtr body) {
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
    if (!body || !generator) throw std::invalid_argument("Force registration requires a body and generator.");
    body->Wake();
    this->forceRegistry.push_back({body, std::move(generator)});
}

void World::addUniversalForce(std::unique_ptr<IForceGenerator> generator) {
    if (!generator) throw std::invalid_argument("Universal force requires a generator.");
    for (const auto& body : bodies) body->Wake();
    this->universalForceRegistry.push_back(std::move(generator));
}

void World::addParticleSystem(ParticleSystemPtr system) {
    if (!system) throw std::invalid_argument("Particle system cannot be null.");
    if (system) {
        particleSystems.push_back(std::move(system));
    }
}

void World::removeParticleSystem(const ParticleSystemPtr& system) {
    particleSystems.erase(
        std::remove(particleSystems.begin(), particleSystems.end(), system),
        particleSystems.end()
    );
}

void World::clearParticleSystems() {
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
    if (!bp) throw std::invalid_argument("Broad phase cannot be null.");
    if (bp) {
        broadPhase = std::move(bp);
    }
}

void World::setSimulationConfig(const SimulationConfig& config) {
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

    // Integrate velocities and positions
    for (RigidBodyPtr& body : bodies) {
        if (body->IsAwake()) {
            ++statistics.integratedBodyCount;
            body->Integrate(deltaTime, simulationConfig);

            body->orientation = std::remainder(body->orientation, 6.283185307179586f);
        }
    }

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

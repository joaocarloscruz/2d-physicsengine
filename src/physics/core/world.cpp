#include "physics/core/world.h"
#include "physics/core/collisions/collision_resolver.h"
#include "physics/core/collisions/broad_phase/sweep_and_prune.h"
#include <utility>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <unordered_set>

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
    this->forceRegistry.push_back({body, std::move(generator)});
}

void World::addUniversalForce(std::unique_ptr<IForceGenerator> generator) {
    if (!generator) throw std::invalid_argument("Universal force requires a generator.");
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

    for (auto& registration : forceRegistry) {
        registration.generator->applyForce(registration.body.get());
    }

    for (auto& generator : universalForceRegistry) {
        for (auto& body : bodies) {
            generator->applyForce(body.get());
        }
    }

    // Integrate velocities and positions
    for (RigidBodyPtr& body : bodies) {
        if (!body->IsStatic()) {
            ++statistics.integratedBodyCount;
            body->Integrate(deltaTime, simulationConfig);

            while (body->GetOrientation() > M_PI) {
                body->SetOrientation(body->GetOrientation() - 2.0f * M_PI);
            }
            while (body->GetOrientation() < -M_PI) {
                body->SetOrientation(body->GetOrientation() + 2.0f * M_PI);
            }
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
        if (!inserted) CollisionResolver::WarmStart(constraints.back());
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

    for (int iter = 0; iter < simulationConfig.solverIterations; ++iter) {
        ++statistics.solverIterationCount;
        for (ContactConstraint& constraint : constraints) {
            CollisionResolver::SolveVelocity(constraint);
        }
    }
    for (int iter = 0; iter < simulationConfig.solverIterations; ++iter) {
        bool positionsSolved = true;
        for (ContactConstraint& constraint : constraints) {
            positionsSolved = CollisionResolver::SolvePosition(constraint)
                && positionsSolved;
        }
        if (positionsSolved) {
            break;
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

#include "physics/core/world.h"
#include "physics/core/collisions/collision_resolver.h"
#include "physics/core/collisions/broad_phase/sweep_and_prune.h"
#include <utility>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <unordered_set>

namespace PhysicsEngine {

ContactKey ContactKey::From(const RigidBody* bodyA, const RigidBody* bodyB) {
    const std::uint64_t idA = bodyA->GetId();
    const std::uint64_t idB = bodyB->GetId();
    return idA < idB ? ContactKey{idA, idB} : ContactKey{idB, idA};
}

bool ContactKey::contains(std::uint64_t bodyId) const {
    return first == bodyId || second == bodyId;
}

bool ContactKey::operator==(const ContactKey& other) const {
    return first == other.first && second == other.second;
}

std::size_t ContactKeyHash::operator()(const ContactKey& key) const {
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
    if (body) {
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
    }
}

void World::clearBodies() {
    bodies.clear();
    forceRegistry.clear();
    universalForceRegistry.clear();
    potentialCollisions.clear();
    contactCache.clear();
}

void World::addForce(RigidBodyPtr body, std::unique_ptr<IForceGenerator> generator) {
    this->forceRegistry.push_back({body, std::move(generator)});
}

void World::addUniversalForce(std::unique_ptr<IForceGenerator> generator) {
    this->universalForceRegistry.push_back(std::move(generator));
}

void World::addParticleSystem(ParticleSystemPtr system) {
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
    if (listener) {
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
    if (!std::isfinite(deltaTime) || deltaTime < 0.0f) {
        throw std::invalid_argument("World delta time must be finite and non-negative.");
    }
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
        if (!inserted) {
            CollisionResolver::WarmStart(constraints.back());
        }
        for (ICollisionListener* listener : collisionListeners) {
            listener->onCollision(manifold);
        }
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

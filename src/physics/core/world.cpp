#include "physics/core/world.h"
#include "physics/core/collisions/collision_resolver.h"
#include "physics/core/collisions/broad_phase/sweep_and_prune.h"
#include <utility>
#include <algorithm>
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

World::World() : broadPhase(std::make_unique<SweepAndPrune>()) {}

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

void World::step(float deltaTime) {
    potentialCollisions.clear();
    std::unordered_set<ContactKey, ContactKeyHash> activeContacts;

    for (const ParticleSystemPtr& system : particleSystems) {
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
            body->Integrate(deltaTime);

            while (body->GetOrientation() > M_PI) {
                body->SetOrientation(body->GetOrientation() - 2.0f * M_PI);
            }
            while (body->GetOrientation() < -M_PI) {
                body->SetOrientation(body->GetOrientation() + 2.0f * M_PI);
            }
        }
    }

    // Narrow phase + resolve with multiple iterations for stability
    // Listeners fire only on the first iteration to avoid duplicate callbacks
    const int iterations = 10;
    for (int iter = 0; iter < iterations; ++iter) {
        potentialCollisions = broadPhase->FindPotentialCollisions(bodies);
        for (const auto& pair : potentialCollisions) {
            CollisionManifold manifold = CheckCollision(pair.first.get(), pair.second.get());
            if (manifold.hasCollision) {
                CanonicalizeManifold(manifold);
                const ContactKey key = ContactKey::From(manifold.A, manifold.B);
                auto [contact, inserted] = contactCache.try_emplace(key);
                activeContacts.insert(key);

                if (iter == 0 && !inserted) {
                    constexpr float warmStartFactor = 0.8f;
                    contact->second.normal *= warmStartFactor;
                    contact->second.tangent *= warmStartFactor;
                    CollisionResolver::WarmStart(manifold, contact->second);
                }

                CollisionResolver::Resolve(
                    manifold,
                    contact->second,
                    iter == 0 && !inserted
                );
                if (iter == 0) {
                    for (ICollisionListener* listener : collisionListeners) {
                        listener->onCollision(manifold);
                    }
                }
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

}

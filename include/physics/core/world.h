#ifndef WORLD_H
#define WORLD_H

#include <vector>
#include <memory>
#include <cstdint>
#include <unordered_map>
#include "rigidbody.h"
#include "force_generator.h"
#include "particles/particle_system.h"
#include "collisions/broad_phase/aabb.h"
#include "collisions/collision_manifold.h"
#include "collisions/collision_dispatcher.h"
#include "collisions/collision_resolver.h"
#include "collisions/collision_listener.h"
#include "collisions/broad_phase/ibroad_phase.h"
#include "types.h"
#include "simulation_config.h"

namespace PhysicsEngine {

struct ForceRegistration {
    RigidBodyPtr body;
    std::unique_ptr<IForceGenerator> generator;
};

struct ContactKey {
    std::uint64_t first;
    std::uint64_t second;

    static ContactKey From(const RigidBody* bodyA, const RigidBody* bodyB);
    bool contains(std::uint64_t bodyId) const;
    bool operator==(const ContactKey& other) const;
};

struct ContactKeyHash {
    std::size_t operator()(const ContactKey& key) const;
};

class World {
public:
    World();
    explicit World(const SimulationConfig& config);
    ~World();

    void addBody(RigidBodyPtr body);
    void removeBody(RigidBodyPtr body);
    void clearBodies();

    void addForce(RigidBodyPtr body, std::unique_ptr<IForceGenerator> generator);
    void addUniversalForce(std::unique_ptr<IForceGenerator> generator);

    void addParticleSystem(ParticleSystemPtr system);
    void removeParticleSystem(const ParticleSystemPtr& system);
    void clearParticleSystems();

    void addCollisionListener(ICollisionListener* listener);
    void removeCollisionListener(ICollisionListener* listener);

    void setBroadPhase(std::unique_ptr<IBroadPhase> bp);
    void setSimulationConfig(const SimulationConfig& config);

    void step(float deltaTime);
    
    const std::vector<RigidBodyPtr>& getBodies() const;
    const std::vector<ForceRegistration>& getForceRegistry() const;
    const std::vector<std::unique_ptr<IForceGenerator>>& getUniversalForceRegistry() const;
    const std::vector<CollisionPair>& getPotentialCollisions() const;
    const std::vector<ParticleSystemPtr>& getParticleSystems() const;
    std::size_t getPersistentContactCount() const;
    const SimulationConfig& getSimulationConfig() const;

private:
    std::vector<RigidBodyPtr> bodies;
    std::vector<ForceRegistration> forceRegistry;
    std::vector<std::unique_ptr<IForceGenerator>> universalForceRegistry;
    std::vector<ParticleSystemPtr> particleSystems;
    std::vector<CollisionPair> potentialCollisions;
    std::vector<ICollisionListener*> collisionListeners;
    SimulationConfig simulationConfig;
    std::unique_ptr<IBroadPhase> broadPhase;
    std::unordered_map<ContactKey, ContactImpulse, ContactKeyHash> contactCache;
};

}

#endif // WORLD_H

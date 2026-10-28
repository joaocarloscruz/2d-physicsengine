#ifndef WORLD_H
#define WORLD_H

#include <vector>
#include <memory>
#include <cstdint>
#include <unordered_map>
#include <unordered_set>
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
#include "simulation_statistics.h"
#include "joints.h"

namespace PhysicsEngine {
namespace Detail { struct IntegrationStrategy; struct ExperimentalWorldStep; }

struct ForceRegistration {
    RigidBodyPtr body;
    std::unique_ptr<IForceGenerator> generator;
};

class World {
public:
    World();
    explicit World(const SimulationConfig& config);
    ~World();

    void addBody(RigidBodyPtr body);
    void removeBody(RigidBodyPtr body);
    void clearBodies();
    void addJoint(JointPtr joint);
    void removeJoint(const JointPtr& joint);
    const std::vector<JointPtr>& getJoints() const { return joints; }

    void addForce(RigidBodyPtr body, std::unique_ptr<IForceGenerator> generator);
    void addUniversalForce(std::unique_ptr<IForceGenerator> generator);

    void addParticleSystem(ParticleSystemPtr system);
    void removeParticleSystem(const ParticleSystemPtr& system);
    void clearParticleSystems();

    void addCollisionListener(ICollisionListener* listener);
    void removeCollisionListener(ICollisionListener* listener);

    void setBroadPhase(std::unique_ptr<IBroadPhase> bp);
    void setSimulationConfig(const SimulationConfig& config);

    void step();
    void step(float deltaTime);
    
    const std::vector<RigidBodyPtr>& getBodies() const;
    const std::vector<ForceRegistration>& getForceRegistry() const;
    const std::vector<std::unique_ptr<IForceGenerator>>& getUniversalForceRegistry() const;
    const std::vector<CollisionPair>& getPotentialCollisions() const;
    const std::vector<ParticleSystemPtr>& getParticleSystems() const;
    std::size_t getPersistentContactCount() const;
    const SimulationConfig& getSimulationConfig() const;
    const SimulationStatistics& getLastStepStatistics() const;

private:
    friend struct Detail::ExperimentalWorldStep;
    bool stepInternal(float deltaTime, Detail::IntegrationStrategy* strategy);
    void requireMutationAllowed() const;
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

    struct Island {
        std::vector<RigidBody*> bodies;
        std::vector<ContactConstraint*> contacts;
        std::vector<JointPtr> joints;
    };
    // connects, when given, selects the constraints that link and join islands.
    std::vector<Island> buildIslands(std::vector<ContactConstraint>& constraints, bool previousContacts = false,
        const std::vector<char>* connects = nullptr);
    using ContactKeySet = std::unordered_set<ContactKey, ContactKeyHash>;
    // Speculative-contact step after loads are applied. Publishes the
    // touching (event) contacts, stamps the cache entries to keep and fills
    // the swept CCD impacts.
    void stepSpeculative(float deltaTime, SimulationStatistics& statistics,
        std::vector<CollisionManifold>& impacts);
    // Position of body in `bodies` (O(1) through the index each step
    // records), or bodies.size() when the body is not in this world.
    std::size_t findBodyIndex(const RigidBody* body) const;
    // As findBodyIndex, but a body outside this world throws std::out_of_range.
    std::size_t bodyIndex(const RigidBody* body) const;
    // Records a touching contact for this step and queues its Begin/Persist
    // event. Returns false, doing nothing, if the key was already recorded.
    bool publishContact(const ContactKey& key, const CollisionManifold& manifold,
        const RigidBodyPtr& bodyA, const RigidBodyPtr& bodyB, bool skipIfActive);
    void wakeIsland(Island& island);
    // Broad-phase pair filter: collision bits, then joints that disable
    // collision between their bodies (jointedPairs, rebuilt every step).
    bool pairCollides(const CollisionPair& pair) const;
    std::vector<JointPtr> joints;
    enum class EventPhase { Begin, Persist, End };
    struct PendingEvent {
        EventPhase phase;
        CollisionEvent event;
        CollisionManifold manifold;
        RigidBodyPtr bodyA;
        RigidBodyPtr bodyB;
        // Step stamp of the last step in which this contact was touching.
        std::uint64_t activeStep = 0;
    };
    void dispatchEvents();
    static bool SupportsCapsuleCcd(const RigidBody& a, const RigidBody& b);
    void validateCcdSupport() const;
    std::vector<CollisionManifold> advanceCcd(const std::vector<Vector2>& starts,
        float deltaTime, SimulationStatistics& statistics);
    void endContacts(std::uint64_t bodyId);
    std::unordered_map<ContactKey, PendingEvent, ContactKeyHash> contactEvents;
    std::vector<PendingEvent> pendingEvents;
    bool dispatchingEvents = false;
    bool stepping = false;
    std::vector<RigidBodyPtr> bodies;
    std::vector<ForceRegistration> forceRegistry;
    std::vector<std::unique_ptr<IForceGenerator>> universalForceRegistry;
    std::vector<ParticleSystemPtr> particleSystems;
    std::vector<CollisionPair> potentialCollisions;
    std::vector<ICollisionListener*> collisionListeners;
    SimulationConfig simulationConfig;
    SimulationStatistics lastStepStatistics;
    std::unique_ptr<IBroadPhase> broadPhase;
    struct CachedContact {
        ContactImpulseCache impulses;
        std::uint64_t keptStep = 0;     // stamp of the last step that detected the pair
        std::uint64_t touchingStep = 0; // stamp of the last step it was touching
    };
    std::unordered_map<ContactKey, CachedContact, ContactKeyHash> contactCache;
    // Incremented at the start of every step; entries compare their stamps
    // against it instead of collecting per-step key sets.
    std::uint64_t stepStamp = 0;
    // Cached pairs that are actually touching (touchingStep == touchingStamp);
    // speculative-only cache entries are excluded from the persistent contact
    // count. Updated when a step finishes its contact bookkeeping.
    std::uint64_t touchingStamp = ~std::uint64_t{0};
    std::size_t touchingCount = 0;
    ContactKeySet jointedPairs;
};

}

#endif // WORLD_H

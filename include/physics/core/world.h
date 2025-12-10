#ifndef WORLD_H
#define WORLD_H

#include <vector>
#include <memory>
#include "rigidbody.h"
#include "force_generator.h"
#include "collisions/broad_phase/aabb.h"
#include "collisions/collision_manifold.h"
#include "collisions/collisions_dispatcher.h"
#include "collisions/collision_resolver.h"
#include "types.h"

namespace PhysicsEngine {

struct ForceRegistration {
    RigidBodyPtr body;
    std::unique_ptr<IForceGenerator> generator;
};

class World {
public:
    World();
    ~World();

    void addBody(RigidBodyPtr body);
    void removeBody(RigidBodyPtr body);

    void addForce(RigidBodyPtr body, std::unique_ptr<IForceGenerator> generator);
    void addUniversalForce(std::unique_ptr<IForceGenerator> generator);

    void step(float deltaTime);
    
    const std::vector<RigidBodyPtr>& getBodies() const;
    const std::vector<ForceRegistration>& getForceRegistry() const;
    const std::vector<std::unique_ptr<IForceGenerator>>& getUniversalForceRegistry() const;
    const std::vector<CollisionPair>& getPotentialCollisions() const;

private:
    std::vector<RigidBodyPtr> bodies;
    std::vector<ForceRegistration> forceRegistry;
    std::vector<std::unique_ptr<IForceGenerator>> universalForceRegistry;
    std::vector<CollisionPair> potentialCollisions;
};

}

#endif // WORLD_H

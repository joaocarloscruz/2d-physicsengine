#ifndef WORLD_H
#define WORLD_H

#include <vector>
#include <memory> // For std::unique_ptr
#include "rigidbody.h"
#include "force_generator.h"
#include "collisions/broad_phase/aabb.h"
#include "collisions/collision_manifold.h"
#include "collisions/collisions_dispatcher.h"
#include "collisions/collision_resolver.h"

namespace PhysicsEngine {
    // A structure to link a generator to a body
    struct ForceRegistration {
        RigidBody* body;
        std::unique_ptr<IForceGenerator> generator;
    };

    class World {
    public:
        World();
        ~World();

        void addBody(RigidBody* body);
        void removeBody(RigidBody* body);

        void addForce(RigidBody* body, std::unique_ptr<IForceGenerator> generator);
        void addUniversalForce(std::unique_ptr<IForceGenerator> generator);

        void step(float deltaTime);
        
        const std::vector<RigidBody*>& getBodies() const;
        const std::vector<ForceRegistration>& getForceRegistry() const;
        const std::vector<std::unique_ptr<IForceGenerator>>& getUniversalForceRegistry() const;
        const std::vector<std::pair<RigidBody*, RigidBody*>>& getPotentialCollisions() const;

    private:
        std::vector<RigidBody*> bodies;
        std::vector<ForceRegistration> forceRegistry;
        std::vector<std::unique_ptr<IForceGenerator>> universalForceRegistry;
        std::vector<std::pair<RigidBody*, RigidBody*>> potentialCollisions;
    };
}

#endif // WORLD_H

#include "physics/core/world.h"
#include "physics/core/collisions/collision_resolver.h"
#include "physics/core/collisions/broad_phase/sweep_and_prune.h"
#include <utility>   // For std::move
#include <algorithm> // For std::remove

namespace PhysicsEngine {

    World::World() {
    }

    World::~World() {
    }

    void World::addBody(RigidBody* body) {
        if (body) {
            bodies.push_back(body);
        }
    }

    void World::removeBody(RigidBody* body) {
        bodies.erase(std::remove(bodies.begin(), bodies.end(), body), bodies.end());

        // Remove any force registrations associated with this body
        forceRegistry.erase(std::remove_if(forceRegistry.begin(), forceRegistry.end(), 
            [body](const ForceRegistration& reg) {
                return reg.body == body;
            }), forceRegistry.end());
    }

    void World::addForce(RigidBody* body, std::unique_ptr<IForceGenerator> generator) {
        this->forceRegistry.push_back({body, std::move(generator)});
    }

    void World::addUniversalForce(std::unique_ptr<IForceGenerator> generator) {
        this->universalForceRegistry.push_back(std::move(generator));
    }

    void World::step(float deltaTime) {
        potentialCollisions.clear();

        // Apply all registered forces from the registry
        for (auto& registration : forceRegistry) {
            registration.generator->applyForce(registration.body);
        }

        // Apply all universal forces to all bodies
        for (auto& generator : universalForceRegistry) {
            for (auto& body : bodies) {
                generator->applyForce(body);
            }
        }

        potentialCollisions = SweepAndPrune::FindPotentialCollisions(bodies);
        
        
        for (const auto& pair : potentialCollisions) {
            CollisionManifold manifold = CheckCollision(pair.first, pair.second);
            if (manifold.hasCollision) {
                CollisionResolver::Resolve(manifold);
            }
        }

        // Integrate all bodies
        for (RigidBody* body : bodies) {
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
    }

    const std::vector<RigidBody*>& World::getBodies() const {
        return bodies;
    }

    const std::vector<std::pair<RigidBody*, RigidBody*>>& World::getPotentialCollisions() const {
        return potentialCollisions;
    }

    const std::vector<ForceRegistration>& World::getForceRegistry() const {
        return forceRegistry;
    }

    const std::vector<std::unique_ptr<IForceGenerator>>& World::getUniversalForceRegistry() const {
        return universalForceRegistry;
    }

}

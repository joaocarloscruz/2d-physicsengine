#include "physics/core/world.h"
#include "physics/core/collisions/collision_resolver.h"
#include "physics/core/collisions/broad_phase/sweep_and_prune.h"
#include <utility>
#include <algorithm>

namespace PhysicsEngine {

World::World() {
    broadPhase = std::make_unique<SweepAndPrune>();
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
}

void World::clearBodies() {
    bodies.clear();
    forceRegistry.clear();
    universalForceRegistry.clear();
    potentialCollisions.clear();
}

void World::addForce(RigidBodyPtr body, std::unique_ptr<IForceGenerator> generator) {
    this->forceRegistry.push_back({body, std::move(generator)});
}

void World::addUniversalForce(std::unique_ptr<IForceGenerator> generator) {
    this->universalForceRegistry.push_back(std::move(generator));
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
                CollisionResolver::Resolve(manifold);
                if (iter == 0) {
                    for (ICollisionListener* listener : collisionListeners) {
                        listener->onCollision(manifold);
                    }
                }
            }
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

}

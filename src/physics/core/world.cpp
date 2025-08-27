#include "../include/physics/core/world.h"
#include <utility>   // For std::move
#include <algorithm> // For std::remove

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

void World::step(float deltaTime) {
    // Apply all registered forces from the registry
    for (auto& registration : forceRegistry) {
        registration.generator->applyForce(registration.body);
    }

    // Integrate all bodies
    for (RigidBody* body : bodies) {
        body->Integrate(deltaTime);
    }
}

const std::vector<RigidBody*>& World::getBodies() const {
    return bodies;
}

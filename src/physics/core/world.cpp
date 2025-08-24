#include "../include/physics/core/world.h"
#include <algorithm> // Required for std::remove

World::World() : gravity(0.0f, 9.8f) {

}

// Destructor to clean up memory
World::~World() {
    for (auto& body : bodies) {
        delete body;
    }
    bodies.clear();
}

void World::step(float deltaTime) {
    // apply forces 
    for (RigidBody* body : bodies) {
        body->ApplyForce(gravity * body->GetMass()); // gravity
    }

    // 2. Integrate all bodies
    for (RigidBody* body : bodies) {
        body->Integrate(deltaTime);
    }
}

void World::addBody(RigidBody* body) {
    if (body) {
        bodies.push_back(body);
    }
}

void World::removeBody(RigidBody* body) {
    bodies.erase(std::remove(bodies.begin(), bodies.end(), body), bodies.end());
}

const std::vector<RigidBody*>& World::getBodies() const {
    return bodies;
}

void World::setGravity(const Vector2& g) {
    gravity = g;
}

Vector2 World::getGravity() const {
    return gravity;
}

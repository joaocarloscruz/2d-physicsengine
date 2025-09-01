#include "physics_engine.h"

namespace PhysicsEngine {

    PhysicsEngine::PhysicsEngine() {
        // The world is empty by default. The sandbox is responsible for adding objects.
    }

    void PhysicsEngine::step(float deltaTime) {
        world.step(deltaTime);
    }

    void PhysicsEngine::addBody(RigidBody* body) {
        world.addBody(body);
    }

    void PhysicsEngine::addForce(RigidBody* body, std::unique_ptr<IForceGenerator> generator) {
        world.addForce(body, std::move(generator));
    }

    void PhysicsEngine::addUniversalForce(std::unique_ptr<IForceGenerator> generator) {
        world.addUniversalForce(std::move(generator));
    }

    const std::vector<RigidBody*>& PhysicsEngine::getBodies() const {
        return world.getBodies();
    }

}
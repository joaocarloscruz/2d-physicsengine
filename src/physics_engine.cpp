#include "physics_engine.h"

namespace PhysicsEngine {

    PhysicsEngine::PhysicsEngine() {
        // The world is empty by default. The sandbox is responsible for adding objects.
        addMaterial("default", {1.0f, 0.5f});
        addMaterial("rock", {1.2f, 0.1f});
        addMaterial("wood", {0.7f, 0.3f});
        addMaterial("metal", {2.0f, 0.05f});
        addMaterial("bouncy", {0.5f, 1.0f});
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

    void PhysicsEngine::addMaterial(const std::string& name, const Material& material) {
        materials[name] = material;
    }

    const Material& PhysicsEngine::getMaterial(const std::string& name) const {
        return materials.at(name);
    }

}

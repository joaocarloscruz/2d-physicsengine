#include "../include/engine.h"

namespace PhysicsEngine {

    Engine::Engine() {
        // The world is empty by default. The sandbox is responsible for adding objects.
        addMaterial("default", {1.0f, 0.5f});
        addMaterial("rock", {1.2f, 0.1f});
        addMaterial("wood", {0.7f, 0.3f});
        addMaterial("metal", {2.0f, 0.05f});
        addMaterial("bouncy", {0.5f, 1.0f});
    }

    void Engine::step(float deltaTime) {
        world.step(deltaTime);
    }

    void Engine::addBody(RigidBodyPtr body) {
        world.addBody(body);
    }

    void Engine::addForce(RigidBodyPtr body, std::unique_ptr<IForceGenerator> generator) {
        world.addForce(body, std::move(generator));
    }

    void Engine::addUniversalForce(std::unique_ptr<IForceGenerator> generator) {
        world.addUniversalForce(std::move(generator));
    }

    const std::vector<RigidBodyPtr>& Engine::getBodies() const {
        return world.getBodies();
    }

    void Engine::addCollisionListener(ICollisionListener* listener) {
        world.addCollisionListener(listener);
    }

    void Engine::removeCollisionListener(ICollisionListener* listener) {
        world.removeCollisionListener(listener);
    }

    void Engine::addMaterial(const std::string& name, const Material& material) {
        materials[name] = material;
    }

    const Material& Engine::getMaterial(const std::string& name) const {
        return materials.at(name);
    }

}

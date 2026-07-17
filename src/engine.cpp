#include "../include/engine.h"

namespace PhysicsEngine {

    Engine::Engine() : fixedStepRunner(world) {
        // The world is empty by default. The sandbox is responsible for adding objects.
        // Material = {density, restitution, staticFriction, dynamicFriction}
        addMaterial("default", {1.0f, 0.5f, 0.6f, 0.4f});
        addMaterial("rock", {1.2f, 0.1f, 0.8f, 0.6f});
        addMaterial("wood", {0.7f, 0.3f, 0.5f, 0.3f});
        addMaterial("metal", {2.0f, 0.05f, 0.3f, 0.2f});
        addMaterial("bouncy", {0.5f, 1.0f, 0.4f, 0.3f});
        addMaterial("ice", {0.9f, 0.1f, 0.05f, 0.02f});
    }

    void Engine::step(float deltaTime) {
        world.step(deltaTime);
    }

    void Engine::stepFixed() {
        world.step();
    }

    FixedStepResult Engine::advance(double elapsedTime) {
        return fixedStepRunner.advance(elapsedTime);
    }

    void Engine::resetTiming() {
        fixedStepRunner.reset();
    }

    double Engine::getAccumulatedTime() const {
        return fixedStepRunner.getAccumulatedTime();
    }

    std::uint64_t Engine::getTotalStepCount() const {
        return fixedStepRunner.getTotalStepCount();
    }

    void Engine::setSimulationConfig(const SimulationConfig& config) {
        world.setSimulationConfig(config);
    }

    SimulationConfig Engine::getSimulationConfig() const {
        return world.getSimulationConfig();
    }

    SimulationStatistics Engine::getLastStepStatistics() const {
        return world.getLastStepStatistics();
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

    void Engine::addParticleSystem(ParticleSystemPtr system) {
        world.addParticleSystem(std::move(system));
    }

    void Engine::removeParticleSystem(const ParticleSystemPtr& system) {
        world.removeParticleSystem(system);
    }

    const std::vector<RigidBodyPtr>& Engine::getBodies() const {
        return world.getBodies();
    }

    const std::vector<ParticleSystemPtr>& Engine::getParticleSystems() const {
        return world.getParticleSystems();
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

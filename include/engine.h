#ifndef ENGINE_H
#define ENGINE_H

#include <vector>
#include <memory>
#include <string>
#include <map>
#include "physics/core/world.h"
#include "physics/core/shape.h"
#include "physics/core/force_generator.h"
#include "physics/core/material.h"
#include "physics/core/types.h"
#include "physics/core/collisions/collision_listener.h"
#include "physics/core/fixed_step_runner.h"

namespace PhysicsEngine {
    class Engine {
    public:
        Engine();
        void step(float deltaTime);
        void stepFixed();
        FixedStepResult advance(double elapsedTime);
        void resetTiming();
        double getAccumulatedTime() const;
        std::uint64_t getTotalStepCount() const;
        void setSimulationConfig(const SimulationConfig& config);
        SimulationConfig getSimulationConfig() const;
        SimulationStatistics getLastStepStatistics() const;

        void addBody(RigidBodyPtr body);
        void addForce(RigidBodyPtr body, std::unique_ptr<IForceGenerator> generator);
        void addUniversalForce(std::unique_ptr<IForceGenerator> generator);
        void addParticleSystem(ParticleSystemPtr system);
        void removeParticleSystem(const ParticleSystemPtr& system);
        const std::vector<RigidBodyPtr>& getBodies() const;
        const std::vector<ParticleSystemPtr>& getParticleSystems() const;

        void addCollisionListener(ICollisionListener* listener);
        void removeCollisionListener(ICollisionListener* listener);

        void addMaterial(const std::string& name, const Material& material);
        const Material& getMaterial(const std::string& name) const;

    private:
        World world;
        FixedStepRunner fixedStepRunner;
        std::map<std::string, Material> materials;
        std::vector<std::unique_ptr<Shape>> owned_shapes;
        std::vector<RigidBodyPtr> owned_bodies;
    };
}

#endif // ENGINE_H

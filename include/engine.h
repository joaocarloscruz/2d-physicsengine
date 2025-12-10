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

namespace PhysicsEngine {
    class Engine {
    public:
        Engine();
        void step(float deltaTime);

        void addBody(RigidBodyPtr body);
        void addForce(RigidBodyPtr body, std::unique_ptr<IForceGenerator> generator);
        void addUniversalForce(std::unique_ptr<IForceGenerator> generator);
        const std::vector<RigidBodyPtr>& getBodies() const;

        void addMaterial(const std::string& name, const Material& material);
        const Material& getMaterial(const std::string& name) const;

    private:
        World world;
        std::map<std::string, Material> materials;
        std::vector<std::unique_ptr<Shape>> owned_shapes;
        std::vector<RigidBodyPtr> owned_bodies;
    };
}

#endif // ENGINE_H

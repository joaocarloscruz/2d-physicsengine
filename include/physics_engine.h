#ifndef PHYSICS_ENGINE_H
#define PHYSICS_ENGINE_H

#include <vector>
#include <memory>
#include <string>
#include <map>
#include "physics/core/world.h"
#include "physics/core/rigidbody.h"
#include "physics/core/shape.h"
#include "physics/core/force_generator.h"
#include "physics/core/material.h"

namespace PhysicsEngine {
    class PhysicsEngine {
    public:
        PhysicsEngine();
        void step(float deltaTime);

        void addBody(RigidBody* body);
        void addForce(RigidBody* body, std::unique_ptr<IForceGenerator> generator);
        void addUniversalForce(std::unique_ptr<IForceGenerator> generator);
        const std::vector<RigidBody*>& getBodies() const;

        void addMaterial(const std::string& name, const Material& material);
        const Material& getMaterial(const std::string& name) const;

    private:
        World world;
        std::map<std::string, Material> materials;
        std::vector<std::unique_ptr<Shape>> owned_shapes;
        std::vector<std::unique_ptr<RigidBody>> owned_bodies;
    };
}

#endif // PHYSICS_ENGINE_H

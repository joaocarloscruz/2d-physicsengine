#ifndef PHYSICS_ENGINE_H
#define PHYSICS_ENGINE_H

#include <vector>
#include <memory>
#include "physics/core/world.h"
#include "physics/core/rigidbody.h"
#include "physics/core/shape.h"
#include "physics/core/force_generator.h"

class PhysicsEngine {
public:
    PhysicsEngine();
    void step(float deltaTime);

    void addBody(RigidBody* body);
    void addForce(RigidBody* body, std::unique_ptr<IForceGenerator> generator);
    const std::vector<RigidBody*>& getBodies() const;

private:
    World world;
    std::vector<std::unique_ptr<Shape>> owned_shapes;
    std::vector<std::unique_ptr<RigidBody>> owned_bodies;
};

#endif // PHYSICS_ENGINE_H

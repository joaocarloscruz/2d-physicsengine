#ifndef WORLD_H
#define WORLD_H

#include <vector>
#include <memory> // For std::unique_ptr
#include "rigidbody.h"
#include "force_generator.h"

// A structure to link a generator to a body
struct ForceRegistration {
    RigidBody* body;
    std::unique_ptr<IForceGenerator> generator;
};

class World {
public:
    World();
    ~World();

    void addBody(RigidBody* body);
    void removeBody(RigidBody* body);

    void addForce(RigidBody* body, std::unique_ptr<IForceGenerator> generator);

    void step(float deltaTime);
    
    const std::vector<RigidBody*>& getBodies() const;

private:
    std::vector<RigidBody*> bodies;
    std::vector<ForceRegistration> forceRegistry; 
};

#endif // WORLD_H

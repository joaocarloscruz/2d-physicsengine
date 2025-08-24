#ifndef WORLD_H
#define WORLD_H

#include <vector>
#include "rigidbody.h"
#include "../math/vector2.h"

class World {
public:
    World();
    ~World(); 

    void addBody(RigidBody* body);
    void removeBody(RigidBody* body); 
    void step(float deltaTime);

    // Gravity accessors
    void setGravity(const Vector2& g);
    Vector2 getGravity() const;

    const std::vector<RigidBody*>& getBodies() const;

private:
    std::vector<RigidBody*> bodies;
    Vector2 gravity;
};

#endif // WORLD_H

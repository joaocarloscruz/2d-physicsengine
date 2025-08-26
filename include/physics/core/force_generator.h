#ifndef FORCE_GENERATOR_H
#define FORCE_GENERATOR_H

#include "rigidbody.h"

// This is the interface that all force generators will implement.
class IForceGenerator {
public:
    virtual ~IForceGenerator() = default;

    virtual void applyForce(RigidBody* body) = 0;
};

#endif // FORCE_GENERATOR_H

#ifndef FORCE_GENERATOR_H
#define FORCE_GENERATOR_H

#include "rigidbody.h"

// This is the interface that all force generators will implement.
class IForceGenerator {
public:
    // The destructor must be virtual for any class with virtual functions.
    virtual ~IForceGenerator() = default;

    // The core function that applies a force to a given body.
    // The "= 0" makes this a pure virtual function, defining this class as an interface.
    virtual void applyForce(RigidBody* body) = 0;
};

#endif // FORCE_GENERATOR_H

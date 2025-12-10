#ifndef PHYSICS_TYPES_H
#define PHYSICS_TYPES_H

#include <memory>
#include <vector>
#include <utility>

namespace PhysicsEngine {

class RigidBody;

using RigidBodyPtr = std::shared_ptr<RigidBody>;
using RigidBodyWeakPtr = std::weak_ptr<RigidBody>;
using CollisionPair = std::pair<RigidBodyPtr, RigidBodyPtr>;

}

#endif // PHYSICS_TYPES_H

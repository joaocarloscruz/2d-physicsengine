#pragma once
#include "physics/core/collisions/collision_resolver.h"
namespace PhysicsEngine::ContactSolverDetail {
// Internal implementation/testing seam; not an installed or supported API.
// Returns false only when there is no usable two-point normal block.
bool SolveNormalBlock(ContactConstraint& constraint);
void SynchronizeFeatureCache(const CollisionManifold& manifold,ContactImpulseCache& cache);
}

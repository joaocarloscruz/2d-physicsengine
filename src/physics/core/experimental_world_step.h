#pragma once
// Internal diagnostic seam; not installed and not a production configuration API.
#include "physics/core/world.h"
#include <optional>
namespace PhysicsEngine::Detail {
struct IntegrationPlan {
    RigidBody *body = nullptr;
    std::uint64_t bodyId = 0;
    Vector2 startPosition, startVelocity, startForce;
    float startOrientation = 0, startAngularVelocity = 0, startTorque = 0;
    Vector2 position, velocity;
    float orientation = 0, angularVelocity = 0;
    bool closedContact = false;
    CollisionManifold manifold;
    ContactImpulseCache reaction;
};
struct IntegrationStrategy {
    virtual ~IntegrationStrategy() = default;
    // Called exactly once, after automatic forces. Must not mutate World/bodies.
    virtual bool stage(const World &, float, IntegrationPlan &) = 0;
};
struct ExperimentalWorldStep {
    static bool Step(World &world, float dt, IntegrationStrategy &strategy) {
        return world.stepInternal(dt, &strategy);
    }
    static std::optional<ContactImpulseCache> Cache(const World &world, const RigidBody &a,
                                                    const RigidBody &b) {
        const auto it = world.contactCache.find(World::ContactKey::From(&a, &b));
        if (it == world.contactCache.end())
            return std::nullopt;
        return it->second;
    }
};
} // namespace PhysicsEngine::Detail

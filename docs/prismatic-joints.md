# Prismatic joints

`physics/core/prismatic_joint.h` and the supported `physics/physics.h` entry point
export `PrismaticJoint`. Construct it with bodies A and B, a local axis on A,
and optional local anchors on A and B, then pass it to `World::addJoint`.

```cpp
auto slider = std::make_shared<PhysicsEngine::PrismaticJoint>(
    support, carriage, PhysicsEngine::Vector2{1, 0},
    PhysicsEngine::Vector2{0, 1}, PhysicsEngine::Vector2{0, 1});
world.addJoint(slider);
```

The joint locks transverse anchor separation and the relative orientation from
construction, leaving axial translation free. The axis rotates with A. The
signed translation is the projection of B's anchor minus A's anchor onto that
axis, in world units; construction does not reset translation to zero.
`getTranslationSpeed()` includes rotation of A's axis. Both dynamic endpoints,
or one fixed endpoint, are supported; two fixed endpoints are rejected.
Finite nonzero axes are normalized with double intermediates, including tiny
and large finite values. Local anchors must be finite.

`getAngle()` reports principal relative orientation error in [-pi, pi]. The
construction reference angle is also principal; full revolutions are not
tracked. Double effective mass calculations include rotational lever arms and
solve transverse and angular rows together. Joint changes to both endpoints
are staged and reject unrepresentable float results before publishing them.

These are discrete iterative constraints. World integrates positions before
solving velocities; use sufficiently small timesteps and enough iterations,
especially with interacting contacts or multiple joints. Position correction
is bounded by the world's maximum correction and 0.2 radians per solve.
There is no guarantee for arbitrary timesteps, mass ratios or initial errors.
Existing joint island sleep, wake propagation and body-removal ownership apply.

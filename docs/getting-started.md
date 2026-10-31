# Getting started with rigid bodies

This guide covers the rigid-body part of the engine: building a world, choosing
units, stepping it from a game loop, and tuning it when a scene misbehaves.
[`examples/getting_started.cpp`](../examples/getting_started.cpp) builds the
guide's scene as one program: a box stack, a bouncing ball with a collision
listener, a sensor zone, a pendulum, sleeping and a ray cast. CTest runs it as
`getting_started_smoke` and fails if the scene stops behaving as described.

For short programs of common game patterns (platformer, ragdoll, vehicle, destructible wall, trigger zones), see the [cookbook](cookbook.md).

See the [README](../README.md) for building and linking the library.

## A first world

```cpp
#include <physics/physics.h>
using namespace PhysicsEngine;

World world;
world.addUniversalForce(std::make_unique<Gravity>(Vector2{0, -9.81f}));

// The last argument makes the ground static: it never moves and has infinite mass.
auto ground = std::make_shared<RigidBody>(Polygon::MakeBox(20, 1), Material{}, Vector2{0, -0.5f}, true);
world.addBody(ground);

auto box = std::make_shared<RigidBody>(Polygon::MakeBox(1, 1), Material{}, Vector2{0, 3});
world.addBody(box);

for (int frame = 0; frame < 120; ++frame)
    world.step();               // one fixed step of 1/60 s
// box->position.y is now 0.5: the box rests on the ground.
```

Bodies copy their shape, so temporary shapes are fine. The world shares
ownership of bodies and joints with you through `std::shared_ptr`.

## Shapes and materials

| Shape | Constructor | Notes |
| --- | --- | --- |
| Circle | `Circle(radius)` | |
| Box | `Polygon::MakeBox(width, height)` | Centered on the body position |
| Convex polygon | `Polygon({v0, v1, ...})` | Strictly convex, either winding. Use `Recentered()` for dynamic bodies; see [polygon centering](polygon-centering.md) |
| Capsule | `Capsule(halfLength, radius)` | A segment from `(-halfLength, 0)` to `(halfLength, 0)`, inflated by `radius`; see [capsules](capsules.md) |

`Material{density, restitution, staticFriction, dynamicFriction}` sets mass per
area in kg/m², bounciness from 0 to 1, and Coulomb friction. The default is
`{1, 0.5, 0.6, 0.4}`. A contact uses the larger restitution and the geometric
mean of the two frictions.

Set the initial pose with `SetPosition`, `SetOrientation` (radians,
counterclockwise), `SetVelocity` and `SetAngularVelocity`. Apply loads with
`ApplyForce`, `ApplyTorque`, or `ApplyImpulse(impulse, contactVector)`, where
`contactVector` is the point relative to the body position. Forces last for
one step, so a constant force must be reapplied every step or registered as a
force generator with `World::addForce`.

## Units and scale

The engine has no unit system of its own, but its default tolerances are fixed
lengths and speeds chosen for SI units: metres, kilograms and seconds.

| Setting | Default | Meaning |
| --- | ---: | --- |
| `penetrationSlop` | 0.005 m | Overlap left uncorrected so resting contacts stay touching |
| `maxPositionCorrection` | 0.2 m | Largest overlap repaired by one position iteration |
| `restitutionVelocityThreshold` | 1 m/s | Slower impacts do not bounce |
| `maxLinearSpeed` | 200 m/s | Speed clamp (disable with `enableLinearVelocityLimit = false`) |
| `maxAngularSpeed` | 30 rad/s | Spin clamp (disable with `enableAngularVelocityLimit = false`) |
| `sleepEnergyThreshold` | 0.0005 m²/s² | Kinetic energy per unit mass below which a body may sleep |

These defaults work best for moving bodies between about 0.1 m and 10 m across.
Do not use pixels as the engine unit. A 64-pixel sprite simulated as a 64 m
box falls at the right rate for its size but looks like slow motion. Pick a
scale such as 50 pixels per metre, divide by it when creating bodies, and
multiply by it when drawing.

If your objects really are much smaller or larger, scale the tolerances with
them. For a length factor `s` (for example 0.01 for centimetre-sized parts in
a world measured in metres), multiply `penetrationSlop`,
`maxPositionCorrection`, `restitutionVelocityThreshold` and `maxLinearSpeed`
by `s`, and `sleepEnergyThreshold` by `s²`. Leave angular settings alone.

## Stepping from a game loop

`world.step()` advances one fixed step of `SimulationConfig::fixedTimeStep`
(1/60 s by default). Do not pass the frame time to `world.step(dt)`: a varying
timestep changes the results and can destabilize stacks. Use `Engine`, which
accumulates real time and runs whole fixed steps:

```cpp
Engine engine;
// ... add bodies with engine.addBody(...)
while (running) {
    FixedStepResult progress = engine.advance(frameSeconds);
    render(progress.interpolationAlpha);   // blend previousPosition and position
}
```

`advance` runs at most `maxSubstepsPerAdvance` steps per call and keeps the
rest of the backlog, so a long frame never makes the simulation lose time. See
[fixed timestep](fixed-timestep.md).

## Fast bodies

Contacts are predicted at the start of each step, so bodies that will collide
during the step are stopped at the surface rather than detected after they
overlap. At the default 60 Hz, boxes falling at 13 m/s onto other boxes stop
with less than 2 cm of overlap, and resting contacts stay within the 5 mm
slop.

Small, very fast bodies such as bullets can still pass through thin walls in
one step. Enable continuous collision detection on them:

```cpp
bullet->SetCcdEnabled(true);
```

CCD sweeps a translating circle against circles, polygons and static
capsules. Polygon pairs and rotation are not swept, so give fast polygons a
smaller `fixedTimeStep` instead. A CCD body touching a capsule in any other
combination makes `step` throw before it changes the world; filter those pairs
apart. See [collision events and CCD](collision-lifecycle-and-ccd.md).

## Collision events and filtering

Subclass `ICollisionListener` to hear about contacts. Events arrive after each
step, as value snapshots that remain valid after the bodies are removed.

```cpp
class LandingCounter : public ICollisionListener {
public:
    void onCollisionBegin(const CollisionEvent& event) override { ++landings; }
    int landings = 0;
};

LandingCounter counter;
world.addCollisionListener(&counter);   // you keep ownership; remove it before destroying it
```

`onCollisionPersist` fires every step while the bodies keep touching, and
`onCollisionEnd` fires when they separate.

To stop two kinds of bodies from colliding, give each body a category bit and a
mask of the categories it collides with. A pair collides only when each body's
mask contains the other's category. See [collision filtering](collision-filtering.md).

## Sensors and user data

A sensor detects overlaps but never pushes or gets pushed. Use one for
trigger zones, pickups and goal areas:

```cpp
auto goal = std::make_shared<RigidBody>(Polygon::MakeBox(2, 2), Material{}, Vector2{10, 1}, true);
goal->SetSensor(true);
goal->SetUserData(42);            // your own id, index or pointer cast to std::uintptr_t
world.addBody(goal);
```

A sensor's overlaps arrive through the same `onCollisionBegin`, `onCollisionPersist`
and `onCollisionEnd` callbacks, with `event.sensor == true`. Two sensors never
report each other. Sensors do not wake sleeping bodies, and CCD does not sweep
them. A dynamic sensor falls under gravity and passes through everything.
`GetUserData()` returns the value you stored. The engine never reads it.
See [sensors and user data](sensors.md).

## Joints

```cpp
auto pivot = std::make_shared<RigidBody>(Circle(0.1f), Material{}, Vector2{6, 5}, true);
auto bob = std::make_shared<RigidBody>(Capsule(0.5f, 0.15f), Material{}, Vector2{7, 5});
world.addBody(pivot);
world.addBody(bob);
// Anchors are in each body's local frame: the pivot's center and 1 m left of the bob's center.
world.addJoint(std::make_shared<RevoluteJoint>(pivot, bob, Vector2{0, 0}, Vector2{-1, 0}));
```

| Joint | Keeps | Extras |
| --- | --- | --- |
| `DistanceJoint(a, b, length, anchorA, anchorB)` | Anchor distance | `setSpring(frequency, damping)`, `setLengthRange(min, max)` (a rope with `min = 0`) |
| `RevoluteJoint(a, b, anchorA, anchorB)` | Shared anchor point | `setMotor(enabled, speed, maxTorque)`, `setLimits(enabled, lower, upper)` |
| `PrismaticJoint(a, b, axisA, anchorA, anchorB)` | Sliding along an axis of A, no relative rotation | `setMotor(enabled, speed, maxForce)`, `setLimits(enabled, lower, upper)` |
| `MouseJoint(ground, body, anchor, target, frequency, damping, maxForce)` | A soft, force-capped pull of the body's anchor toward a world target; `ground` is any static body in the World | `setTarget(target)` (wakes the body), `setSpring(frequency, damping)`, `setMaxForce(force)` |
| `WeldJoint(a, b, anchorA, anchorB)` | Shared anchor point and relative angle | `setLinearSpring(frequency, damping)`, `setAngularSpring(frequency, damping)` (0 = rigid) |
| `WheelJoint(a, b, axisA, anchorA, anchorB)` | B slides along an axis of A and spins freely: a car wheel | `setSpring(frequency, damping)`, `setMotor(enabled, speed, maxTorque)`, `setLimits(enabled, lower, upper)` |

Jointed bodies do not collide with each other unless you call
`joint->setCollideConnected(true)`; a mouse joint is the exception and starts
with collision on. See [joints and sleeping](joints-and-sleeping.md),
[prismatic joints](prismatic-joints.md) and
[mouse, weld and wheel joints](joints-mouse-weld-wheel.md).

## Sleeping

Sleeping is off by default. Enabling it lets settled groups of bodies stop
simulating until something touches them:

```cpp
SimulationConfig config = world.getSimulationConfig();
config.enableSleeping = true;
world.setSimulationConfig(config);
```

A body moving into a sleeping group wakes it before the impact. Setters such
as `SetPosition` and `SetVelocity`, impulses and forces wake the body, and the
next step wakes everything it touches. If you write a field such as
`body->position` directly, call `Wake()` yourself.

## Spatial queries

```cpp
std::optional<WorldRayHit> hit = RayCastNearest(world, {-3, 10}, {-3, -10});
if (hit) { /* hit->body, hit->hit.point, hit->hit.normal, hit->hit.fraction */ }
std::vector<RigidBodyPtr> under = QueryPoint(world, mouseWorldPosition);
std::vector<RigidBodyPtr> nearby = QueryCircle(world, center, radius);
```

`RayCastAll`, `SweepCircleNearest` and `SweepCircleAll` cover the remaining
cases. Every query takes an optional `QueryFilter` with the same category and
mask rule as collisions. See [spatial queries](spatial-queries.md).

## When a scene misbehaves

| Symptom | Try |
| --- | --- |
| Towers taller than about 20 boxes lean or fall over | `solverSubsteps = 8` and `contactHertz = 120` (stiffer soft contacts, about twice the contact cost) |
| Small fast bodies pass through walls | `SetCcdEnabled(true)` on those bodies |
| Bodies jitter or look floaty | Check the scale: moving bodies should be 0.1 m to 10 m across |
| A body stops accelerating at 200 m/s, or stops spinning up at 30 rad/s | Disable or raise the velocity limits |
| Settled piles keep using CPU | `enableSleeping = true` |
| Chains or ragdolls stretch under load | Raise `solverIterations` (for example to 20) or lower `fixedTimeStep` |
| Stacked bodies bounce instead of settling | Lower restitution; impacts slower than `restitutionVelocityThreshold` never bounce |

All settings and their valid ranges are in
[simulation configuration](simulation-configuration.md). Change them on a copy
from `getSimulationConfig()` and pass it back with `setSimulationConfig()`.

## Performance

The step is single-threaded. On an AMD Ryzen 7 5700X, a settled pile of boxes
and circles in a walled bin takes about 1.1 ms per 60 Hz step with 250 bodies,
2 ms with 500, 4.4 ms with 1000 and 10.5 ms with 2000 (`rigid_step_benchmark`
reproduces these). `solverSubsteps = 1` skips the substepped contact solve and
is about 10% cheaper at 2000 bodies, at the cost of the tall-tower stability
described below. With sleeping enabled,
settled islands skip integration and the solver, though contact detection
still runs for them. Statistics for the last step are in
`world.getLastStepStatistics()`; see [simulation statistics](simulation-statistics.md).

## Known limits

- At 60 Hz, towers of up to 20 boxes with alternating offsets and rotations
  stand (a 20-box tower moves under 1 cm in 30 seconds). A 25-box tower falls
  after about 5 seconds, also at 120 Hz; with `solverSubsteps = 8` and
  `contactHertz = 120` it stands but drifts about 3 cm. At 30 Hz the soft
  contacts are capped at 30 Hz and a 20-box tower falls; `solverSubsteps = 8`
  holds it.
- A perfectly elastic ball (restitution 1) dropped from 1 m rebounds to
  0.998 m on its first bounce, then keeps that height.
- Dynamic polygons rotate about their shape origin, so build them centered (see
  [polygon centering](polygon-centering.md)).

The [rigid-contact benchmark](rigid-contact-benchmark.md) measures stacking,
friction and impact accuracy against physical oracles, and the
`physics_scenarios` test checks 99 scenario metrics on every build.

## Seeing it

The SFML visualizer in [visualization](../visualization/README.md) has ten
interactive scenes: stacks, pyramids, dominoes, chains, capsules, a drivable car
with a sensor goal, and a joints gallery. You can drag bodies with the mouse and
spawn new ones. The [web playground](web-playground.md) runs the same scenes in
a browser on the WebAssembly build.

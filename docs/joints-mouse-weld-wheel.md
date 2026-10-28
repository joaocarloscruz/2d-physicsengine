# Mouse, weld and wheel joints, and distance springs

Three more joints for game scenes, plus spring and range options on
`DistanceJoint`. Include `physics/physics.h`; the classes live in
`physics/core/mouse_joint.h`, `weld_joint.h` and `wheel_joint.h`. All of them
implement `IJoint`, so they go through `world.addJoint`, are solved in the same
velocity passes as contacts and the other joints, are warm-started with
`warmStartFactor`, and throw `std::invalid_argument` for invalid inputs before
changing any state. Body validation, double-precision internals and
`std::overflow_error` for unrepresentable results follow
[joints and sleeping](joints-and-sleeping.md).

## Soft constraints

The mouse joint, the wheel suspension, the soft rows of a weld and the distance
spring use one formulation: Box2D v3's soft step (`b2MakeSoft`, from Erin
Catto's Solver2D notes). For a row with effective mass `M`, frequency `f` Hz,
damping ratio `zeta` and step `h`:

```
omega = 2 pi f
a1 = 2 zeta + h omega,  a2 = h omega a1,  a3 = 1 / (1 + a2)
biasRate = omega / a1,  massScale = a2 a3,  impulseScale = a3
lambda = -massScale * M * (Cdot + biasRate * C) - impulseScale * accumulated
```

This is the backward (implicit) Euler step of `x'' = -omega^2 x - 2 zeta omega x'`
on mass `M`, so it is stable for any stiffness and step. The frequency is that
of the row's own effective mass. For a point at a body's center of mass that is
the body's mass; for an off-center anchor or a joint between two dynamic
bodies it is the reduced mass of the row, and the stiffness is
`k = (2 pi f)^2 * M`.

Backward Euler adds numerical damping and lowers the frequency slightly when
`h * omega` is not small. The tests compare each run against both the
backward-Euler image (eigenvalue `1/(1 - h s)` of each continuous root `s`) and
the requested values. Measured, free body, no gravity:

| Case | Step | Measured f, zeta | Backward Euler f, zeta | Requested |
| --- | --- | --- | --- | --- |
| Mouse, weld, distance spring | 1/60 s | 1.9626 Hz, 0.2002 | 1.9625 Hz, 0.2002 | 2 Hz, 0.1 |
| Same | 1/1000 s | 1.9987 Hz, 0.1062 | 1.9987 Hz, 0.1062 | 2 Hz, 0.1 |
| Wheel suspension | 1/60 s | 2.8601 Hz, 0.3379 | 2.8602 Hz, 0.3379 | 3 Hz, 0.2 |
| Wheel suspension | 1/1000 s | 2.9942 Hz, 0.2090 | 2.9942 Hz, 0.2090 | 3 Hz, 0.2 |

The extra damping is about `h * omega / 2`: at 60 Hz a 2 Hz spring gains 0.1.
Ask for less damping than you want at large steps, or step smaller. Soft rows
do no position projection; the spring is the correction. A soft row needs
elapsed time, so on a zero-duration step it applies nothing.

A soft row's result in one step does not depend on its warm-start value (the
`impulseScale` term cancels it), so `warmStartFactor` changes only how fast the
other constraints in the island converge around it.

## MouseJoint

```cpp
auto handle = std::make_shared<RigidBody>(Circle(0.05f), Material{}, Vector2{}, true);
handle->SetCollisionMaskBits(0);
world.addBody(handle);
// Grab the box at its center; pass the grabbed point in box coordinates to grab elsewhere.
auto drag = std::make_shared<MouseJoint>(handle, box, Vector2{}, cursor,
    5.0f /*Hz*/, 0.7f /*damping ratio*/, 1000.0f * box->GetMass() * 9.81f /*max force*/);
world.addJoint(drag);
// every frame:
drag->setTarget(cursor);
// on release:
world.removeJoint(drag);
```

`MouseJoint(ground, body, localAnchor, target, frequency, dampingRatio, maxForce)`
pulls `body`'s anchor (in body coordinates) toward `target` (world coordinates).
`IJoint` joins two bodies, so the first argument is a static body that must
already be in the World: the level's ground, or a small static body with
collision mask 0 kept for dragging. Its pose is ignored. Passing a dynamic
ground throws. An internal hidden body was not used because `World::addJoint`
requires both bodies to belong to the World and an unlisted body would bypass
its island and removal bookkeeping. Unlike the other joints,
`collideConnected` defaults to true, so a body dragged across the ground body
still collides with it.

- `frequency > 0`, `dampingRatio >= 0`, `maxForce >= 0`, all finite.
  `setSpring(frequency, dampingRatio)` and `setMaxForce(maxForce)` change them.
- The spring force is capped by `maxForce`: the accumulated impulse vector is
  clamped to `maxForce * h` each solve, including the carried warm-start value.
  `getReactionForce()` reports the last step's force on the body. Box2D's
  suggested scale is `1000 * mass * g`.
- `setTarget` wakes the body when the target changes. The joint never keeps the
  island awake by itself: a body that comes to rest at the target may sleep,
  and the next target change wakes it.
- Hanging at rest under gravity the anchor sits `g / omega^2` below the target
  (measured to 1e-4 relative).

Tunneling. Speculative contacts size their margin from the velocity at the
start of the step, and the joint adds velocity inside the step. Measured with
the default config, a 0.25 kg box at Box2D's force scale (at most 163 m/s of
speed change per 60 Hz step) dragged at a 0.5 m static wall: a 5 Hz drag to a
target 7 m past the wall overlapped the wall by at most 5 mm; a 30 Hz flick to
a target 97 m past it reached 163 m/s, overlapped by 0.26 m in one step and
was pushed back out; the body center never entered the wall and both came to
rest against it. Much larger force caps can carry a body through thin walls in
one step. Keep `maxForce` near that scale for thin geometry.

## WeldJoint

`WeldJoint(a, b, localAnchorA = {}, localAnchorB = {})` locks B's anchor to A's
anchor and the relative angle to its value at construction (`getAngle()` reports
the difference, `getSeparation()` the anchor distance).

- Rigid (default): the point and angle rows are solved together as one 3x3
  system, through the Schur complement of the angular row, in both the
  velocity and position passes. A single weld is exact in one iteration.
- `setLinearSpring(frequency, dampingRatio)` and
  `setAngularSpring(frequency, dampingRatio)` soften either part; frequency 0
  is rigid. With a soft row the joint solves the angular row and then the
  point row, as Box2D v3 does, and projects only rigid rows. The frequency is
  per row (point mass for the linear part, `1/(invIa + invIb)` for the angular
  part), so it is exact when B's anchor is at its center of mass.
- Welds never keep an island awake.

Measured (10 s at 60 Hz, gravity, a 4 m x 0.2 m beam welded to a static wall):

| Beam | Iterations | Worst tip sag | Sag after 10 s |
| --- | --- | --- | --- |
| One body | 10 | 0 | 0 |
| Four 1 m welded segments | 10 | 0.121 m (first 0.2 s) | 0.09 mm |
| Four 1 m welded segments | 30 | 0.030 m | 0.33 mm |
| Four segments, `warmStartFactor = 0` | 10 | | 2.98 m |

A chain of welds is solved by Gauss-Seidel over the joints, so on first load it
rings like a stiff beam until warm starting carries the load; more iterations
reduce the ring. Without warm starting the chain never holds.

Two welded 1x1 boxes against one 2x1 box (same mass and inertia) launched with
the same rigid motion, spinning at 2 rad/s: momentum is exact, and the pair's
spin falls behind by a first-order splitting error. Each half moves on a chord
of its arc and projection pulls it back radially, which scales the orbital
angular momentum by `1 - (omega h)^2 / 2` per step. Predicted spin loss over
2 s: `0.6 * omega^3 * h * T / 2`; measured 0.0749 rad/s against 0.08 at 1/60 s
and 0.0197 against 0.02 at 1/240 s. The center of mass stayed within 1.5 mm
(1/60 s) and 0.4 mm (1/240 s). The revolute joint shares this property, as
any joint between separately integrated bodies does.

## WheelJoint

`WheelJoint(a, b, localAxisA = {0, 1}, localAnchorA = {}, localAnchorB = {})`:
A is the chassis, B the wheel. B's anchor slides along the axis (fixed in A) and
B rotates freely. Translation is the signed projection of B's anchor minus A's
anchor on the axis, as for `PrismaticJoint`; with the axis pointing up, a
compressed suspension has positive translation.

- `setSpring(frequency, dampingRatio)` pulls the translation toward 0. The
  default frequency 0 leaves the axis free: set a spring (or limits) for a car.
  Stiffness is `(2 pi f)^2` times the axial effective mass
  `1/(1/mA + 1/mB + invIa*sA^2 + invIb*sB^2)` with `sA = (d + rA) x axis`, which
  for a light wheel under a heavy chassis is close to the wheel's mass. Static
  sag is the supported weight over that stiffness. `getSpringForce()` reports
  the last step's spring force on B along the axis.
- `setMotor(enabled, speed, maxTorque)` drives B's angular velocity relative to
  A (rad/s, counterclockwise positive), with the accumulated impulse capped at
  `maxTorque * h`. `getMotorTorque()` reports the last step's torque. A
  driving motor (enabled, positive torque, nonzero speed) keeps the island
  awake; a zero-speed motor is a brake and lets it sleep.
- `setLimits(enabled, lower, upper)` bounds the translation with one-sided stops
  that are speculative in the velocity pass and projected in the position pass.
- The point-to-line row is rigid in velocity and projected for drift.

The rows follow Box2D v3's `wheel_joint.c`: motor, spring, stops, then
point-to-line, each a scalar row in Gauss-Seidel order.

Measured: a 2 kg wheel of inertia 0.25 driven at 10 rad/s with 2 N m reached
the speed after exactly 75 steps (1.25 s = `I omega / T`), its torque never
exceeded the cap and dropped to 0 once at speed. A 20 kg chassis on two 2 kg
wheels (4 Hz, damping 0.7, radius 0.4 m) settled with 0.0950 m of sag on both
springs (static model 0.0962 m). With a rear motor at -15 rad/s and 40 N m it
reached 6.004 m/s after 5 s (rolling speed `omega r` = 6 m/s), pitching at most
0.10 rad. A zero-speed brake on both wheels stopped it and the island slept 162
steps later.

## DistanceJoint spring and range

Additive options; defaults leave the joint rigid and bit-identical to before.

- `setSpring(frequency, dampingRatio)` makes the length soft around `length`
  (frequency 0, the default, is rigid). A hanging bob on a 1 Hz spring sits at
  `length + g / omega^2` (measured to 1e-4 relative).
- `setLengthRange(minLength, maxLength)` adds hard stops, with
  `0 <= minLength <= length <= maxLength`, finite. Without a spring the length
  is free between the stops: `setLengthRange(0, ropeLength)` is a rope (a
  slack bob falls exactly like an unjointed body until the rope is taut). With
  a spring the stops cap its travel. The default range is `[length, length]`.
- `getCurrentLength()` returns the anchor separation.

Switching between rigid and soft modes drops the other mode's accumulated
impulse at the next step.

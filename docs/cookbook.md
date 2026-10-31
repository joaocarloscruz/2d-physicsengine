# Rigid-body cookbook

Five short, headless programs for common game patterns. Each one prints what
happened, checks its own behaviour and exits nonzero if the engine stops doing
what the page says. CTest runs them as `<name>_smoke`. They build with the other
examples (`PHYSICS_BUILD_EXAMPLES`, on by default). For the basics they assume,
read [getting started](getting-started.md) first.

All five use the default configuration except where noted.

## platformer_character

[`examples/platformer_character.cpp`](../examples/platformer_character.cpp)

A box character on a floor and a raised platform. A sensor "foot" tells the
game when it is grounded, so jumping is allowed only on the ground. Horizontal
control sets the velocity directly, and a pickup sensor is collected once.

Key calls:

- `SetSensor(true)` on the foot, with `SetCollisionCategoryBits` and
  `SetCollisionMaskBits` so it sees only the world, never the character or the
  coin. A listener adds one on `onCollisionBegin` and removes one on
  `onCollisionEnd`; `grounded` means the count is above zero.
- The foot is a dynamic sensor. A static sensor never reports against static
  bodies, so the foot copies the character's velocity each frame and falls with
  it. See [sensors](sensors.md).
- Character friction is 0 (a contact uses the geometric mean of both
  frictions), so it never sticks to walls. It stops because the game sets its
  speed to 0. Restitution is 0 on both sides so landings do not bounce.
- The coin is a static sensor with `SetUserData(7)`. The application scores it
  on the first Begin, then calls `removeBody` after the step.

Checks: it lands at the expected height, jumps to the apex `v^2 / 2g` within
0.05 m, refuses every jump request made in the air (the key is held for the
whole flight), takes exactly two jumps in total, and the coin produces one
Begin event and one point.

## ragdoll

[`examples/ragdoll.cpp`](../examples/ragdoll.cpp)

A head, torso, two arms and two legs (thigh and shin) made of capsules, joined
by `RevoluteJoint` with angle limits, thrown down eight steps onto a floor.

Key calls:

- `Capsule(halfLength, radius)` rotated by 90 degrees with `SetOrientation`
  for vertical limbs.
- `RevoluteJoint(parent, child, localAnchorParent, localAnchorChild)` and
  `setLimits(true, lower, upper)`. Limits are the child's angle relative to
  the parent, measured from the pose at construction. The knee allows bending
  one way only.
- `collideConnected` is left at its default (off), so limbs that overlap at a
  shared anchor do not fight the joint.
- `SimulationConfig::enableSleeping = true`, so the settled ragdoll stops
  simulating.

Checks: no joint angle is outside its limits by more than 0.1 rad, no joint's
two anchors are more than 5 cm apart in any step, no part moves faster than
40 m/s, every coordinate stays finite, the torso ends on the floor past the last
step, and every part is asleep before step 1400. Measured: 0.007 rad and
2.7 mm, asleep at step 210.

## vehicle

[`examples/vehicle.cpp`](../examples/vehicle.cpp)

A 20 kg chassis on two wheels drives over three bumps and up an 8 m ramp onto a
plateau, then brakes. See [mouse, weld and wheel joints](joints-mouse-weld-wheel.md#wheeljoint).

Key calls:

- `WheelJoint(chassis, wheel, {0, 1}, anchorOnChassis, {})`: the wheel slides
  along an axis fixed in the chassis and spins freely.
- `setSpring(4, 0.7)` for the suspension and `setLimits(true, -0.25, 0.25)` for
  its travel. With the axis pointing up, a compressed suspension has positive
  translation, read with `getTranslation()`.
- `setMotor(true, -12, 40)` on the rear wheel: speed is B relative to A, so a
  negative value rolls the car toward +x. `setMotor(true, 0, 40)` on both
  wheels is a brake.

Checks: braking starts once the chassis reaches x = 32 on the plateau; the
suspension translation stays within its limits (measured -0.004 to 0.179 m);
the chassis records no solid contact begin event; it never tilts past 0.5 rad;
it stops within 4 s and 3 m of where braking began, with the wheels at rest at
the plateau's wheel height.

## destructible_wall

[`examples/destructible_wall.cpp`](../examples/destructible_wall.cpp)

A wall of 30 boxes held together by 49 `WeldJoint`s, shot with a heavy CCD ball.
The application removes the welds of any body that takes too hard a hit.

`WeldJoint` has no reaction-force getter (it exposes `getSeparation()` and
`getAngle()`, which measure drift, not load), and collision events carry no
impulse. The example therefore estimates the impulse from the events. For each
contact between two moving bodies it computes the approach speed along the
event normal from the velocities saved before the step, and multiplies by the
pair's reduced mass: the impulse of an inelastic hit between free bodies. This
is an estimate, not the solver's value. Bodies above `kBreakImpulse` (8 N s)
lose their welds after the step.

Key calls: `WeldJoint(a, b, anchorA, anchorB)` with anchors on the shared edge,
`SetCcdEnabled(true)` on the ball, `CollisionEvent::normal` (it points from
body A to body B, A being the lower id), `world.removeJoint` after `step`.
Removed welds leave their bodies in the world, free to fall.

Checks: before the shot every weld is within 2 mm; the first weld breaks within
20 steps of the shot; 4 to 16 welds are removed, each touching a box in the first three
columns; the 7 welds of the far upper-right corner survive; the joint list
matches the application's; all coordinates stay finite for the rest of the run;
and the ball ends in front of the wall.

The break happens one step late: the weld is still in place during the step of the
impact, so the welded wall takes that push. The constants matter. With the same
wall hit by a ball of density 20 at 40 m/s, all 30 boxes moved more than 0.5 m
and only one box lost its welds. The example uses a ball of density 10 at
30 m/s, which cracks the wall locally and leaves it standing.

## trigger_zones

[`examples/trigger_zones.cpp`](../examples/trigger_zones.cpp)

Three static sensor zones and three balls. Zone A accepts every category, B only
red, C only blue. Red goes through all three and comes back, blue goes through
all three and stops, and a "ghost" ball has a mask that excludes the zones'
category. A listener counts entries and exits per zone and body and prints an
event log.

Key calls: `SetSensor(true)`, `SetCollisionCategoryBits` / `SetCollisionMaskBits`
on zones and balls (a zone reports a body only when both masks accept each
other's category), `onCollisionBegin` / `onCollisionEnd` with
`CollisionEvent::sensor`, and body ids to tell zone from ball because `bodyAId`
is always the lower id. No gravity is registered, so the balls keep the
velocity given to them.

Checks: every zone and body pair has exactly the expected number of entries and
exits (for example red crosses zone A twice, blue once, ghost never), and the
zones' user data is intact.

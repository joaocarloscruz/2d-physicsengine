# Prismatic joints

Native and [WebAssembly](webassembly.md) APIs share these constraint semantics.

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

## Motor and travel limits

`setMotor(enabled, speed, maxForce)` drives signed relative axial speed in world
units per second. `maxForce` must be finite and nonnegative. The accumulated
motor impulse over all velocity iterations of one world step is capped at
`maxForce * dt`; increasing solver iterations does not multiply the drive force.
`getMotorForce()` returns that signed accumulated impulse divided by the last
step duration, or zero for a zero-duration step. It reports the motor only;
constraint and stop reaction forces are not force limited. Zero speed acts as a
brake, zero force has no effect, and disabling the motor preserves free sliding.

`setLimits(enabled, lower, upper)` uses finite ordered translations in world
units. Equal endpoints lock axial motion. Lower and upper stops constrain
outward velocity while allowing inward release; speculative velocity limits
also account for remaining travel over the next timestep. Initial violations
are corrected in bounded position iterations. Limits may be outside the
construction translation. A motor pushing against a stop remains force capped.
Drive and stops solve the coupled transverse, angle and axial system. After
clamping an axial impulse, the transverse and angular impulses are recomputed
for the actual impulse, including long off-center lever arms.

Changing motor or limit settings wakes both endpoints. An enabled nonzero-speed
motor with positive force prevents its island from sleeping, including when a
stop blocks motion. A zero-speed brake does not keep a settled island awake.
Settings reject invalid values before changing the existing configuration.

For an initially stationary fixed-support carriage of mass m driven by a
saturated motor F, velocity increases by F*dt/m each step. Since World advances
positions before joint velocity solving, after N steps its displacement is
F/m * dt^2 * N*(N-1)/2. This converges to continuous acceleration as dt decreases;
the first driven step changes velocity without advancing position.

## Relative geometry and constraint arithmetic

Anchor separation is formed as `(positionB-positionA) + (rotatedAnchorB-rotatedAnchorA)`.
Small local offsets therefore survive a common world translation that would
erase them from separately assembled world anchors. Public body positions still
use floats; offsets already lost in the inputs cannot be recovered.

The angular row is eliminated before accumulating the velocity right-hand side.
Let `n` be the transverse normal, `e` the slide axis, `d = positionB-positionA`,
`s = d cross n`, `x = d cross e`, and `ia`, `ib` the inverse inertias. The reduced
velocity targets are `-n dot (vB-vA) + s*w` and
`targetSpeed-e dot (vB-vA) + x*w`, where
`w = (ib*omegaA + ia*omegaB)/(ia+ib)`. This avoids subtracting enormous common
anchor-speed terms to recover a small linear velocity. The reduced matrix is
`m*I + weight*[s,x]*[s,x]^T`, with `m` the inverse-mass sum and
`weight = ia*ib/(ia+ib)`. Its positive determinant and expanded adjugate avoid
subtracting large nearly equal mass-matrix products.

Both angular corrections are also recovered from the eliminated row and lever
differences, rather than adding a huge anchor torque to its opposing angular
impulse. Capped motor/stop impulses use the same reduction. Corrections stage
both bodies before publishing either, retain static state, and reject nonzero
static inverse properties. This is per-correction atomicity; previous World
integration, other joints and earlier corrections are not rolled back.

Regression cases compare free sliding, locked/one-sided stops and capped drives
with analytical velocities, including shared anchors up to `1e30`, mass scales
`1e-30` through `1e30`, rotated axes and common translations. An isolated unequal
mass pair checks linear momentum, angular momentum and the expected kinetic
energy loss independently. These checks do not establish full World accuracy at
arbitrary scales: float state, rotational linearization and position correction
remain limiting factors.

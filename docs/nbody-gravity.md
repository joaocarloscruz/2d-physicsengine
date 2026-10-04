# Planar Newtonian N-body gravity

`NBodyGravity`, exposed through `physics/physics.h`, evolves independent point
masses in double-precision `Vector2d` coordinates. It uses ordinary three-dimensional
inverse-square gravity restricted to a plane, rather than a two-dimensional
Poisson/logarithmic potential. There is no coupling to `World`, fluids, soft bodies
or electromagnetic particles, and no contact, accretion, cosmological expansion,
relativistic correction. An owned standalone JavaScript binding is described in
[WebAssembly usage](webassembly.md#n-body-gravity).

For separation d from particle i to j and softening length epsilon:

```
rho = sqrt(dot(d,d) + epsilon^2)
force_on_i = G * mass_i * mass_j * d / rho^3
force_on_j = -force_on_i
pair_potential = -G * mass_i * mass_j / rho
```

Plummer softening therefore modifies both force and potential consistently. With
epsilon zero this is Newtonian point gravity; positive epsilon removes the
coincidence singularity and lets point particles pass through one another. It is
a chosen model length, rather than a physical collision radius.

## Configure and run

```cpp
#include "physics/physics.h"
#include <cmath>
using namespace PhysicsEngine;
NBodyGravityConfig config;
config.gravitationalStrength = 1.0;
config.maxSubstep = 0.002;
NBodyGravity orbit(config);
orbit.addParticle({-0.5, 0}, {0, -std::sqrt(0.5)}, 1);
orbit.addParticle({ 0.5, 0}, {0,  std::sqrt(0.5)}, 1);
orbit.step(0.1);
const auto metrics = orbit.getDiagnostics();
```

Default `G=1` means reduced units. Choose a consistent mass, length and time
system, with G in length^3/(mass*time^2). SI applications must explicitly supply
their chosen measured G in m^3/(kg*s^2); no SI numerical constant is implied by
the default. Softening and positions share length units; velocities use length/time.
Mass, position, velocity and all configuration numbers must be finite. Mass is
strictly positive, while G and softening are nonnegative. Particle indexes are
stable and append-only. `setState` changes position and velocity together;
`applyImpulse` changes velocity by impulse/mass. They validate before mutation.
Copies own independent state, and observers expose only const particle vectors.

The headless `gravity_binary` example advances equal unit masses through one
analytic circular period, printing phase-position error, energies and work counts.
It builds with `PHYSICS_BUILD_EXAMPLES=ON` and runs as a CTest smoke check.

## Integration and encounters

Velocity Verlet applies half a kick, a drift, and half a kick using new forces.
Every pair contributes equal and opposite central forces, preserving linear and
angular momentum up to rounding. Fixed-step conservative integration has
second-order trajectory convergence and approximately bounded, oscillatory energy
error; it does not preserve energy exactly. Adaptive step changes can introduce
additional long-term energy error. Tests cover orbital phase/period, refinement,
energy, angular momentum, unequal-mass center-of-mass drift and deterministic replay.

Substeps satisfy `maxSubstep` and the local tidal estimate

```
bound = max_i sum_(j!=i) 4 * G * mass_j / rho_ij^3
h * sqrt(bound) <= frequencySafety
```

The factor four bounds diagonal/off-diagonal blocks of the softened force
Jacobian. This is a local timescale control, not a guarantee of arbitrary-timestep
stability or a prescribed error. A trial drift can change each relative position
vector by at most one quarter of its initial softened distance. Larger motion
halves the trial step. For zero softening, this prevents a drift from jumping
through a singular coincidence. Unresolved encounters reject the complete outer
step when precision, substep, trial or pair-work limits run out. Positive softening
allows a resolved crossing on the chosen softened force scale. Exactly coincident
unsoftened states are rejected by `step` and potential diagnostics when G>0.
With G=0, forces and potential are zero, coincidence is allowed and motion is free.

The frequency duration is rounded downward, and every actual substep is at most
the representable local limit. Integer partition tolerance never permits a larger
duration. Within an outer step the chosen duration is retained until a tighter
bound or trial guard requires reducing it; the final duration may be shorter.

## Bounded work and diagnostics

Defaults are `maxParticles=1024`, `maxSubsteps=4096`, `maxPairWork=8000000`,
`maxSubstep=0.01`, `frequencySafety=0.1` and `softening=0`. Frequency safety must
lie in `(0,1]`. Each accepted substep permits at most 32 trial drifts. Every visited
pair in initial/final force evaluation, trial checks and final potential diagnostics
counts against one shared per-step pair-work budget. Thus a large particle count
multiplied by many substeps or rejected trials cannot silently bypass the limit.
The model is deterministic O(N^2), without a tree or mesh approximation. Allocation
and O(N) integration work are additionally bounded by particle and substep caps.

Diagnostics provide total mass, center of mass, linear momentum, angular momentum
about the coordinate origin, kinetic energy, potential energy, total energy, and
successful substeps/pair work from the last step. Public `getDiagnostics` computes
potential in O(N^2) under its own `maxPairWork` cap, without changing the last-step
counters. No external force or mechanical-work ledger is implied: impulses and
state/configuration edits are caller-controlled changes outside conservative
integration. Successful zero-duration or empty steps report zero work counts.

## Numerical and failure limits

Invalid arguments throw `std::invalid_argument`, missing indexes throw
`std::out_of_range`, and particle/configuration-size limits throw `std::length_error`.
Unresolved encounters and work limits throw `std::runtime_error`; nonrepresentable
arithmetic throws `std::overflow_error`. Failed steps preserve every particle and
the previous successful-step work counters, including failures in final aggregate
diagnostics. Lowering `maxPairWork` may make subsequent steps or diagnostics fail.

Scaled products avoid premature overflow/underflow in force/mass and energy
calculations. Aggregate positive energy terms are combined before final subnormal
rounding. Finite inputs can still produce nonrepresentable separations, tidal
rates, accelerations, momenta or energies, which are rejected. Cancellation of
individually overflowing signed terms is not supported. Subnormal motion below
double coordinate resolution can be lost; meaningful units and timestep
convergence checks remain necessary. Reproducibility is expected for identical
inputs/builds/platforms, rather than bitwise across different platforms.

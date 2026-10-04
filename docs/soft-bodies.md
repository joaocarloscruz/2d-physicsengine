# Mass-spring soft bodies

`SoftBody` is a standalone, two-dimensional mass-spring simulator exposed by
`physics/physics.h`. It runs independently of `World`, rigid bodies and fluid
solvers. It supports ropes and planar cloth-like spring networks, finite physical
masses, fixed anchors, uniform acceleration, per-particle forces and impulses. It does
not yet implement self-collision, rigid/fluid contact, volume or area preservation,
plasticity, tearing, material calibration or WebAssembly bindings. A spring
network can fold, intersect and change enclosed area.

## Construct and step a rope

The executable `softbody_rope` builds with `PHYSICS_BUILD_EXAMPLES=ON` and runs
the following ten-link rope headlessly, printing its final diagnostics.

```cpp
#include "physics/physics.h"
using namespace PhysicsEngine;

SoftBody rope;
rope.addParticle({0.0f, 2.0f}, {}, 1.0, true); // Fixed anchor.
for (int i = 1; i <= 10; ++i) {
    rope.addParticle({0.0f, 2.0f - 0.2f * i}, {}, 0.1);
    rope.addSpring(i - 1, i, 0.2, 200.0, 1.0);
}
rope.setUniformAcceleration({0.0f, -9.81f});
rope.applyImpulse(10, {0.05f, 0.0f});
for (int frame = 0; frame < 1200; ++frame) rope.step(1.0 / 120.0);
const auto metrics = rope.getDiagnostics();
const auto& particles = rope.getParticles();
```

Particle and spring indexes are append-only and stable. There are no removal
operations. Observers return const vectors in insertion order. Copies own their
state and can be simulated independently. `setParticleState` can move a fixed
anchor, but fixed particles must have zero velocity. `setFixed(index, true)`
zeros velocity. Impulses on fixed particles are ignored after validating their
inputs. Unpinning preserves zero velocity until further forces or impulses act.

`applyForce(index, Vector2)` or `applyForce(index, forceX, forceY)` accumulates
finite external force components in double precision. `getAccumulatedForce`
and each const particle's `force` expose this pending load. The load contributes
`force/mass` throughout every substep of the next positive-duration `step`, then
clears on success. It is not divided by the number of substeps. A failed step
retains all loads, and a zero-duration step retains them because no integration
occurs. `clearForces()` explicitly clears all loads; `clearForces(index)` clears
one. State and pin setters retain pending loads. Fixed nodes accept loads but do
not integrate them; a successful positive-duration step consumes their loads
along with those of dynamic nodes. Uniform acceleration persists across steps.

For planar cloth, append particles in a regular grid and connect horizontal and
vertical neighbours. Connect both diagonals of each cell for shear resistance;
second-neighbour links can add a simple bending surrogate. Set rest lengths to
the initial link lengths (spacing or spacing times sqrt(2)). This is a spring
network model, rather than a continuum shell or a calibrated textile model.

## Force and damping model

For endpoints a and b, let `n = (xb-xa)/length`. The elastic force on a is
`k * (length-restLength) * n`; b receives its opposite. The potential energy is
`0.5 * k * (length-restLength)^2`. Positive stiffness resists both stretch and
compression. A zero-rest spring has the continuous force `k*(xb-xa)` and can
pass through coincidence. Strain diagnostics exclude zero-rest links because
relative strain would divide by zero.

The axial dashpot force on a is `c * dot(vb-va,n) * n`. It acts only on relative
axial motion, so translating the entire body does not introduce drag. Each
frozen-position damping operation solves this relative speed exactly:
`relativeSpeed *= exp(-c*(inverseMassA+inverseMassB)*h)`. The corresponding
equal and opposite impulse preserves linear and angular momentum for free
endpoints, and decreases kinetic energy. There is no global velocity damping.
Fixed nodes use inverse mass zero while retaining their finite physical mass;
anchors transmit external reaction forces, so momentum conservation does not
apply to a network with anchors or external acceleration.

## Integration and work limits

The elastic integrator is velocity Verlet. Half-step damping operations surround
it, with reverse spring order in the second half to give symmetric splitting.
The undamped two-mass oscillator tests verify Hooke force, analytic phase/period
and second-order timestep convergence. Floating-point rounding in `Vector2`
state means conservation is approximate across separate calls to `step`; work
inside a call and diagnostic sums use double precision. Identical inputs and
builds produce deterministic ordering and state; cross-platform bitwise
reproducibility is not promised.

Each substep obeys `maxSubstep` and a graph stiffness/mass frequency estimate:

```
curvature(s) = k * max(1, abs(1-restLength/currentLength))
frequencyBoundSquared = max_i(2 * inverseMass_i * sum_incident(curvature))
h <= stabilityFactor / sqrt(frequencyBoundSquared)
```

For zero-rest links, curvature is simply k. The graph bound conservatively
includes neighbouring masses through row sums. The transverse curvature term
also reduces the step size for compressed positive-rest springs. Bounds are
recomputed as the configuration changes. A trial drift may change a positive-rest
link's separation vector by at most one quarter of its current length; larger
motion halves the trial timestep. This prevents imposed high speeds from jumping
across the central-force singularity. There are at most 32 bounded trials per
accepted substep. These are local stability and motion controls, not a guarantee
of a prescribed trajectory error for arbitrary nonlinear deformations. Refine
`maxSubstep` for accuracy and check convergence for the application.

Defaults are `maxSubstep=0.01`, `stabilityFactor=0.25`, `maxSubsteps=4096`,
`maxParticles=100000` and `maxSprings=300000`. The stability factor must lie in
`(0,1]`. Collection budgets bound allocation, and `maxSubsteps` bounds the
accepted integration work per call. A conservative budget check may reject a
step even if later spring decompression would have permitted fewer substeps.
Reduce the outer timestep or deliberately increase the budget to handle stiffer
systems. Cost is O(particles + springs) per trial/substep; there is no spatial
search or collision pipeline.

## Validation, diagnostics and failure behavior

Positions, velocities, accelerations, forces and impulses must be finite. Mass must be
positive and have a finite reciprocal. Rest length, stiffness and damping must be
finite and nonnegative. Self-links, missing endpoint indexes, and duplicate
undirected links are rejected. Positive-rest active springs cannot be added with
coincident endpoints. State setters can subsequently create a singular
configuration, which `step` rejects. A zero-length, zero-rest link has zero elastic
force and no axial damping direction at that instant. Inactive links (`k=c=0`)
are also allowed.

Invalid arguments throw `std::invalid_argument`, invalid indexes throw
`std::out_of_range`, and topology/collection budget overflow throws
`std::length_error`. Numerical overflow, positive-rest collapse, motion retry or
substep budget exhaustion throw `std::runtime_error`. An unsuccessful `step`
leaves every particle and `lastSubsteps` unchanged. Impulse overflow also leaves
the particle unchanged. Accumulated force overflow rejects the entire new force
addition; damping rejects an overflowing sum of endpoint inverse masses rather
than silently skipping dissipation. A successful zero timestep or empty-body step reports
zero substeps. Extreme finite coefficients or masses can still overflow derived
calculations and be rejected.

Diagnostics report total finite physical mass (including anchors), linear
momentum components, kinetic energy, spring elastic energy, maximum absolute
relative strain for positive-rest links, and successful substeps in the last
call. They do not include gravitational potential energy. Diagnostic arithmetic
overflow throws instead of returning infinities. Conservative integration can
have bounded oscillatory energy error; combined damped motion need not have
strictly decreasing total energy after every discrete step. The damping-only
pair test verifies exact exponential dissipation, while the oscillator test
verifies long-term loss under damping.

# Two-way fluid and rigid-body coupling

`FluidRigidCoupler` connects WCSPH particles to dynamic or static circle,
convex-polygon and [capsule](capsules.md) rigid bodies. Advanced callers can invoke it directly, but the
recommended entry point is `CoupledFluidSimulation`, which owns the ordering:

1. advance the WCSPH solver;
2. enforce static fluid containers;
3. call `FluidRigidCoupler::couple` with the particles, bodies, and substep;
4. integrate the rigid bodies.

These operations run for every CFL-selected fluid substep. Rigid bodies receive
the fluid configuration's external acceleration during the same substep, so
buoyancy and gravity do not depend on rendered frame duration.

```cpp
PhysicsEngine::CoupledFluidSimulation simulation(smoothingLength);
simulation.step(particles, bodies, frameTime, tank);
const auto& statistics = simulation.getLastStatistics();
```

## Hydrodynamic impulses

For every particle within one smoothing length of a body surface, the coupler
samples the closest surface point and outward normal. In two dimensions, the
particle's effective boundary length is

```text
L_i = V_i / h_i
```

where `V_i` is particle area and `h_i` is smoothing length. With compact linear
surface influence `w_i`, the pressure impulse on the particle is

```text
J_pressure = n p_i L_i w_i pressureScale dt
```

The viscosity impulse moves the particle toward the rigid surface velocity:

```text
J_viscosity = (v_surface - v_i) mu_i V_i / h_i^2
              w_i viscosityScale dt
```

Each impulse applied to the particle is applied with the opposite sign to the
rigid body at the sampled surface point. This conserves coupled linear momentum
and creates rigid-body torque naturally when the interaction is off-center. The
design follows the equal-and-opposite hydrodynamic-force principle in
[Akinci et al., 2012](https://doi.org/10.1145/2185520.2185558).

## Contact and impermeability

When a particle center is closer than `particleRadius` to the surface, it is
projected to the surface offset. Any remaining inward relative normal velocity
is removed with a two-body impulse that includes rigid angular inertia. Default
restitution is zero, preventing the artificial bouncing that occurs when fluid
particles are treated as ordinary elastic rigid bodies.

Static bodies accept no reaction velocity, so they act as external momentum
sinks. Dynamic bodies receive the exact reaction impulse.

## Capsule bodies

A capsule is the circle of radius `r` swept along its spine, so both halves of
the coupling reduce to circle operations about the closest spine point.

**Surface sample.** For a particle at `x`, with body position `c`, spine
direction `u = (cos theta, sin theta)` and half-length `h`, the coupler takes
`t = clamp((x - c) . u, -h, h)` and the spine point `q = c + t u`. The outward
normal is `(x - q) / |x - q|`, the surface point `q + r n` and the signed
distance `|x - q| - r`, negative inside. A particle on the spine itself (within
`1e-6`) leaves through the local `+y` side, the tie-break of the capsule point
queries. Moment arms are `surfacePoint - c` as for every shape, and buoyancy
comes from the sampled pressure, not from a stored area.

**Boundary samples.** `SampleRigidBodyBoundaries` fills a capsule like a
circle, in layers at depth `k * spacing` below the surface for
`k < ceil(supportRadius / spacing)`. Layer `k` is the inset capsule outline of
radius `rho = r - k * spacing` around the same spine, perimeter
`4h + 2 pi rho`, with `ceil(perimeter / spacing)` samples evenly spaced by arc
length starting at the `+x` tip, so neighbours are at most `spacing` apart and
the pattern is symmetric about the spine. Once `rho` falls to `1e-6` or less,
the layer is the spine itself, sampled at `ceil(2h / spacing)` segment
midpoints like a polygon edge, and deeper layers stop. Sample volume,
`pressureScale = 0`, rigid surface velocity, float-range checks, work budget
and all-or-nothing publication match circles and polygons.

**Circle limit.** With `h = 0` the sampler calls the circle sampler, and the
clamp yields a signed zero so `q = c` exactly: samples and surface queries are
bit-identical to the circle of the same radius. Only the body's mass and
inertia can differ, because `Capsule` rounds `pi r^2` from double and `Circle`
computes it in float; for `r = 0.3` they agree to the bit and a coupled tank
run is bit-identical, for `r = 0.27` the body height after 0.4 s differs by
`6e-8` m.

Unsupported shape types are still rejected with `std::invalid_argument`
before any particle or body is changed.

## Validation and tolerances

The headless coupling regressions require:

- total dynamic-body plus particle linear momentum before and after a mixed
  pressure/viscosity interaction to agree within `1e-5` per axis;
- an off-center pressure sample to produce linear motion and torque with the
  expected signs;
- a circular hydrostatic pressure field to accelerate a body lighter than the
  fluid upward and a denser body downward after gravity;
- a complete sampled-wall tank run in which a density-500 circle rises from
  `1.0` to approximately `1.164`, while a density-1500 circle sinks to
  approximately `0.880` after `0.4` simulated seconds;
- penetrating contact to be projected out with no inward relative velocity.

The capsule regressions (`tests/test_fluid_capsule.cpp`) require:

- every surface sample within `2e-7` relative of distance `r` from the spine,
  with an outward unit normal, `ceil(perimeter / spacing)` samples per layer
  and neighbouring chords between `0.998` and `1` arc step; interior samples on
  whole layers with exactly `ceil(2h / spacing)` on the spine;
- a zero half-length capsule to reproduce the circle's samples bit for bit, its
  one-pass coupling within `1e-5` relative, and its tank trajectory
  bit-identical when mass and inertia agree to the bit and within `1e-4` m
  otherwise;
- surface hydrostatic pressure on a capsule at three orientations and three
  densities to give the Archimedes impulse `rho g A dt` within 0.2%, sideways
  impulse and torque below 0.2% of it, and the float/hover/sink velocity
  `g dt (rho / rho_body - 1)`;
- in the sampled-wall tank, for both kernel families and capsules broadside,
  tilted and end-on with the area of the 0.3 m circle: light rises, heavy
  sinks, neutral lies between and moves less than the circle's own rise or
  sink, and the existing `light > heavy + 0.05` criterion. Broadside and
  tilted, neutral and heavy end within 2 cm of the circle. The light capsule
  rises slower than the circle broadside (0.074 m against 0.131 m with
  Poly6/Spiky) and faster end-on (0.341 m), from its width, not its buoyancy;
- a ring of pressure samples around a rotated capsule to conserve linear and
  angular momentum within `1e-5`, push each particle along the outward normal,
  and give a torque impulse equal to the reaction moments within `1e-4`
  relative; with viscosity and contact added, linear momentum within `1e-5`;
- contact on a rotated capsule's side and cap to project to `r +
  particleRadius` with no inward relative velocity.

The hydrostatic check derives buoyancy from the sampled pressure field; it does
not use a density switch or a special "floating" flag. Coupling statistics expose
interaction and correction counts, accumulated pressure and viscosity impulse
magnitudes, and maximum penetration for profiling and diagnostics.

This reference implementation currently checks every particle/body pair. Its
fluid-fluid work remains grid-local, but large scenes should add rigid-body
broad-phase candidate generation before using coupling at production scale.

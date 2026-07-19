# Two-way fluid and rigid-body coupling

`FluidRigidCoupler` connects WCSPH particles to dynamic or static circle and
convex-polygon rigid bodies. Advanced callers can invoke it directly, but the
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

The hydrostatic check derives buoyancy from the sampled pressure field; it does
not use a density switch or a special "floating" flag. Coupling statistics expose
interaction and correction counts, accumulated pressure and viscosity impulse
magnitudes, and maximum penetration for profiling and diagnostics.

This reference implementation currently checks every particle/body pair. Its
fluid-fluid work remains grid-local, but large scenes should add rigid-body
broad-phase candidate generation before using coupling at production scale.

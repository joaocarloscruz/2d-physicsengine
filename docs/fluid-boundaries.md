# Static fluid containers

The reference fluid boundary layer provides impermeable static circle and
strictly convex polygon containers. It is a geometric non-penetration model:
after each WCSPH substep, particle centers outside the permitted region are
projected to the nearest valid boundary plane or circle radius.

The default response has zero restitution. Outward normal velocity is removed
instead of reflected, while an optional normalized friction coefficient damps
tangential velocity. This prevents a resting particle from receiving a new
upward impulse every time gravity brings it back to the floor.

```cpp
PhysicsEngine::FluidBoundarySettings settings;
settings.particleRadius = 0.05f;
settings.restitution = 0.0f;
settings.friction = 0.05f;

PhysicsEngine::FluidConvexPolygonContainer tank(
    {
        {-2.0f, 0.0f},
        { 2.0f, 0.0f},
        { 2.0f, 4.0f},
        {-2.0f, 4.0f},
    },
    settings
);

solver.step(particles, frameTime, tank);
```

Polygon input may use clockwise or counter-clockwise winding. Construction
normalizes winding, rejects degenerate or concave geometry, and precomputes one
inward unit normal per edge. Corner projection iterates the most violated plane
until the particle satisfies every half-space including its configured radius.

Circle containers constrain particle centers to `containerRadius -
particleRadius`. Both boundary types expose `contains()` for validation and
return correction depth statistics. `WcsphStatistics` accumulates corrected
particle count and maximum penetration across the substeps of the latest call.

## Sampled density support

The geometric response is complemented by deterministic SPH boundary samples.
`SampleFluidContainerBoundary` places layers outside a container, while
`SampleRigidBodyBoundaries` places layers inside circles and convex polygons.
The default coupled simulation uses spacing `h / 2` and enough layers to cover
one smoothing length. Moving rigid samples carry the body's surface velocity.

Each sample represents area `spacing^2` and contributes rest-density-equivalent
mass to nearby fluid density estimates:

```text
rho_i += rho0_i V_b W_density(x_i - x_b, h_i)
```

The solver hashes samples into the same-sized spatial cells used by the fluid
grid, so it only visits local candidates. Statistics expose boundary sample and
candidate counts. Geometric projection remains the final impermeability guard;
the samples repair truncated kernel support and prevent the artificial pressure
loss that otherwise occurs next to walls and immersed bodies.

This is a deterministic reference boundary treatment. It does not extrapolate
wall pressure as in the more elaborate generalized condition of
[Adami, Hu, and Adams](https://doi.org/10.1016/j.jcp.2012.05.005).

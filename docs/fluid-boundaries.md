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

This model guarantees geometric impermeability but does not synthesize SPH
dummy particles to repair kernel support near walls. Pressure-extrapolated
dummy-particle methods such as the generalized wall condition of
[Adami, Hu, and Adams](https://doi.org/10.1016/j.jcp.2012.05.005) are a more
elaborate alternative when near-wall pressure accuracy is more important than
the small, deterministic reference implementation provided here.

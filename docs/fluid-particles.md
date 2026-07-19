# Fluid particles and neighborhoods

Fluid state is separate from the generic `Particle` type. `FluidParticle`
stores the quantities required by continuum solvers: density, pressure,
smoothing length, rest density, viscosity, and volume, in addition to position,
velocity, force, and mass. Construction validates the numerical properties and
initializes density to rest density and volume to `mass / restDensity`.

```cpp
PhysicsEngine::FluidParticleProperties properties;
properties.mass = 1.0f;
properties.restDensity = 1000.0f;
properties.smoothingLength = 0.5f;
properties.viscosity = 0.01f;

std::vector<PhysicsEngine::FluidParticle> particles;
particles.emplace_back(
    PhysicsEngine::Vector2(0.0f, 1.0f),
    PhysicsEngine::Vector2(),
    properties
);
```

`FluidParticleSpatialGrid` provides deterministic radius queries. Rebuild it
after particle positions or the particle count changes, then request pairs for
the solver's interaction radius:

```cpp
PhysicsEngine::FluidParticleSpatialGrid grid(properties.smoothingLength);
grid.rebuild(particles);
const auto pairs = grid.findNeighborPairs(
    particles,
    properties.smoothingLength
);
```

Every returned pair is ordered as `(lowerIndex, higherIndex)`, the complete
result is lexicographically sorted, and duplicate pairs are removed. The query
uses the exact Euclidean radius after its grid-cell candidate pass, so its
result matches brute force even when the cell size and interaction radius
differ.

`getLastStatistics()` exposes particle count, occupied cells, candidate pairs,
accepted neighbor pairs, and the maximum neighbors observed for one particle.
The statistics describe the most recent rebuild/query and are intended for
solver diagnostics and performance measurements.

The reference continuum solver using this state and neighborhood layer is
documented in [wcsph-solver.md](wcsph-solver.md).

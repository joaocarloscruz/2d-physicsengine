# Particle systems

`ParticleSystem` provides a contiguous, lightweight simulation path for large
collections that do not need rigid-body shapes, orientation, inertia, or
collision materials.

Each `Particle` stores position, velocity, accumulated force, mass, and inverse
mass. `ParticleSystem::step` applies a configurable uniform acceleration and
integrates every particle. A system can be stepped directly or registered with
`World`, which advances it once per world step.

`ParticleSpatialGrid` prepares particles for neighborhood-based algorithms such
as fluids. Rebuild the grid after particle positions change, then call
`findPotentialPairs` with the interaction radius. Returned index pairs are
unique, ordered, and filtered by actual distance.

Particle indexes belong to the current contiguous vector. Removing a particle
shifts subsequent indexes, so callers should not retain indexes across removal.

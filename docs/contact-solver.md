# Constraint-based contact solver

Each `World::step` detects collision manifolds once and prepares a contact
constraint for every active manifold point. Preparation converts contact
positions to body-local anchors and precomputes the normal and tangent
effective masses. Stable feature IDs reconnect each point to its cached normal
and friction impulses from the previous step.

The solver then runs three ordered phases:

1. Warm-start persistent contacts with their scaled cached impulses.
2. Iterate velocity constraints to enforce non-penetration, restitution, and
   Coulomb friction.
3. Iterate position constraints independently to repair remaining overlap.

`SimulationConfig::solverIterations` controls both iterative phases. Increasing
it improves convergence through stacks and chains of touching bodies, at the
cost of additional constraint work. Broad-phase and narrow-phase detection do
not repeat for each solver iteration.

Low-speed contacts use `restitutionVelocityThreshold` to suppress bounce.
`velocityTolerance` treats tiny tangential motion as settled,
`penetrationSlop` permits a small positional tolerance, and
`maxPositionCorrection` limits the repair performed by a single position
iteration. These values are validated by `SimulationConfig` and are available
through the WebAssembly configuration object.

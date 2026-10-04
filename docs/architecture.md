# Architecture and numerical conventions

The core uses a right-handed 2D coordinate system with positive Y upward. Positions
are in metres, time in seconds, angles in radians and mass in kilograms when inputs
use SI units. Rigid-body density is mass per area (kg/m²); inertia is kg·m² and
torque N·m. Fluid density and particle mass are also interpreted per 2D area;
pressure coefficients should be calibrated for this 2D discretization rather than
copied uncritically from a 3D material. All dynamic state uses single precision.

## Rigid-body pipeline

`Engine` is a convenience facade over `World` and `FixedStepRunner`. A world wakes
connected islands, applies registered forces to awake bodies, integrates state,
optionally processes swept circle impacts, generates broad-phase pairs, filters
pairs, constructs contact manifolds, and solves each independent dynamic island.
Static bodies anchor islands without joining otherwise independent dynamic systems.
Velocity and position constraints run in separate phases. Contact impulses are
cached by stable body IDs and contact feature IDs. Events dispatch after solving.

Integration assumes constant force over each step: position advances by
`v*dt + 0.5*a*dt²`, then velocity by `a*dt`. This is not a general higher-order
Verlet solver for position-dependent forces. Contact and joint iterations are
approximate. Prefer small fixed timesteps and monitor constraint error; increasing
iterations cannot repair arbitrary large-step trajectories. Friction and restitution
are contact approximations. CCD sweeps the integrated linear displacement; it does
not analytically integrate curved trajectories under changing forces.

## Fluids and particles

Particle systems, WCSPH, and DFSPH use separate particle storage. The fluid spatial
grid provides deterministic neighbor pairs. WCSPH offers summation/continuity
density modes, containers and two-way rigid coupling. DFSPH currently implements
free-surface fluid-only density/divergence projections. The common `IFluidSolver`
interface supports interchangeable fluid-only experiments; WCSPH's boundary and
coupling overloads remain method-specific.

## Limits

Only circles and convex nondegenerate polygons are supported. Polygon vertices
are local to the body origin; use centered shapes for the intended inertia model.
The engine has no arbitrary meshes, deformables, implicit rigid integrator, motors,
joint limits or multithreaded solver. CCD supports circle/circle and translating
circle/polygon pairs; rotating polygons and polygon/polygon sweeps remain discrete.
DFSPH has no boundary or rigid coupling implementation yet. Free surfaces can be
underdense; diagnostics distinguish this from volume compression.

Avoid extreme world coordinates, tiny smoothing lengths and float overflow. Use
consistent unit scaling. The test suite and analytical/numerical validation are
the basis for each supported behavior, not a guarantee for every scale or scene.

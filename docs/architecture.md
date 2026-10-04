# Architecture and numerical conventions

The core uses a right-handed 2D coordinate system with positive Y upward. Positions
are in metres, time in seconds, angles in radians and mass in kilograms when inputs
use SI units. Rigid-body density is mass per area (kg/m²); inertia is kg·m² and
torque N·m. Fluid density and particle mass are also interpreted per 2D area;
pressure coefficients should be calibrated for this 2D discretization rather than
copied uncritically from a 3D material. Rigid-body, fluid and particle positions and
velocities use single precision. Soft-body substeps, loads and diagnostic sums use
double precision, with positions and velocities converted to float on publication.

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

The legacy poly6/spiky kernel combination remains the default. An explicit
matched cubic family improves nominal lattice rest in the tested regime while
preserving caller masses and rest densities. Perturbed neighborhoods and sampled
wall startup retain limitations; see [kernel experiments](cubic-kernel-experiments.md).
The nine legacy physical targets tracked in issue #44 remain expected failures.

## Deformable bodies

`SoftBody` is an independent mass-spring network with fixed anchors, elastic links,
axial damping and external loads. It uses adaptive velocity Verlet with symmetric
damping operations. Stiffness and motion controls bound each substep; failed steps
retain the previous state and queued forces. It is not registered with `World` and
does not automatically collide or exchange forces with rigid bodies or fluids.
See [soft-body integration and validation](soft-bodies.md).

## Thermal and electromagnetic modules

`ThermalNetwork` integrates a lumped heat-capacity graph with conductive links,
external power and fixed-temperature reservoirs. Bounded explicit steps retain
state and queued loads on failure; energy diagnostics account for heat entering
through loads and reservoirs. Temperatures and heat capacities use double precision.
See [thermal networks](thermal-networks.md) for units and convergence tests.

`ChargedParticle` analytically evolves a nonrelativistic point charge in prescribed
uniform electric and magnetic fields. Its double-precision state supports planar
cyclotron motion and crossed-field drift. These standalone modules have no automatic
mechanical or fluid coupling; the electromagnetic module does not evolve fields.
See [electromagnetic particles](electromagnetic-particles.md).

## Gravity and scalar waves

`NBodyGravity` integrates independent point masses in double precision with
Newtonian inverse-square gravity restricted to a plane and optional Plummer
softening. Symmetric central pair forces and velocity Verlet support orbital
momentum/energy checks; local encounter limits and a shared pair-work budget
bound a staged step. This is an O(N²) model with reduced-unit G=1 by default,
separate from World's prescribed uniform `Gravity` force generator.
See [N-body gravity](nbody-gravity.md) for units and numerical limits.

`WaveMembrane` evolves a uniform linear membrane on a double-precision rectangular
grid with fixed or periodic edges. A symmetric spatial stencil, Verlet stepping
and damping split are constrained by the two-dimensional wave CFL bound and
cell/substep work budgets. Physical discrete energy and analytical mode tests
measure accuracy; the integrator does not exactly conserve that energy.
See [scalar waves](wave-membranes.md) for layout, forcing and convergence.

Both modules own independent state. They do not automatically exchange loads
with World, fluids, thermal graphs or deformable bodies.

## Limits

Only circles and convex nondegenerate polygons are supported. Polygon vertices
are local to the body origin; use centered shapes for the intended inertia model.
`Polygon::GetCentroid()` and `Recentered()` support this explicit setup; see
[polygon centering](polygon-centering.md) for preserving the world pose and anchors.
The engine has no arbitrary mesh collision, implicit rigid integrator or
multithreaded solver. Revolute joints support torque-limited motors and principal
angle stops; large angular displacements still require small timesteps. Soft bodies
currently have no self-contact, solid/fluid coupling, area constraint or fracture.
CCD supports circle/circle and translating
circle/polygon pairs; rotating polygons and polygon/polygon sweeps remain discrete.
DFSPH has no boundary or rigid coupling implementation yet. Free surfaces can be
underdense; diagnostics distinguish this from volume compression.

Avoid extreme world coordinates, tiny smoothing lengths and float overflow. Use
consistent unit scaling. The test suite and analytical/numerical validation are
the basis for each supported behavior, not a guarantee for every scale or scene.

# Physics capabilities and validation

The engine contains several numerical models with different physical assumptions.
Select a model by its equations and validated regime. A passing build, finite
trajectory or plausible animation does not establish accuracy for a new scene.

| Model | Implemented scope | Evidence | Current boundary of support |
| --- | --- | --- | --- |
| [Rigid bodies](contact-solver.md) | Planar circles and strictly convex polygons, friction/restitution, iterative contacts and position correction | Analytical impacts, momentum/energy checks, manifold geometry oracles, [stack/friction matrix](rigid-contact-benchmark.md) | Tall stacks and high-friction equilibrium can drift; arbitrary meshes, 3D and rotating-polygon CCD are absent |
| [Joints](joints-and-sleeping.md) | Distance, revolute motors/stops and [prismatic slides](prismatic-joints.md) | Analytical constrained motion, torque/force caps, momentum and timestep checks | Iterative articulated systems; no general compliance or exact large-rotation integration |
| [Particles](particles.md) | Independent point particles, loads and integration | Ballistic/force checks and finite state validation | No automatic granular contact, fluid equation of state or material model |
| [WCSPH](wcsph-solver.md) | Summation or continuity density, pressure, viscosity, sampled containers, explicit rigid coupling | [Consistency](fluid-consistency-diagnostic.md), [kernel](cubic-kernel-experiments.md) and [wall](fluid-wall-audit.md) diagnostics, coupling momentum tests | Nine legacy [physical targets](https://github.com/joaocarloscruz/2d-physicsengine/issues/44) remain expected failures; disorder and wall quadrature are unresolved |
| [DFSPH](dfsph-solver.md) | Fluid-only free-surface compression/divergence projection with bounded iterations | Independently measured predicted compression, achieved residuals and WCSPH comparison | Expansion and surface underdensity remain; no container or rigid-body coupling |
| [Periodic MAC grid](periodic-mac-projection.md) | Double face velocities, periodic pressure projection and [implicit constant viscosity](periodic-mac-diffusion.md) | Discrete Fourier modes, temporal/spatial refinement, means, pressure gauge, actual stored residuals and energy/dissipation identities | Separate projection/diffusion operations; no advection, obstacles or free surface |
| [Incompressible flow](periodic-incompressible-flow.md) | Owning periodic constant-density velocity timestep: conservative staggered donor advection, implicit viscosity and projection | Independent dense coupled updates, discrete transported-shear symbols, separate Taylor–Green velocity/pressure/energy refinement, stored energy ledgers and shared-budget rollback | Native only; first-order splitting and numerical diffusion, finite projection residuals; no forcing, obstacles, free surfaces, variable density or rigid coupling |
| [Scalar transport](periodic-scalar-transport.md) | Conservative periodic cell-average density transport with frozen MAC face velocities | Independent donor matrices and Fourier modes, first-order refinement, mass, positivity and CFL checks | First-order numerical diffusion; no velocity advection, automatic pressure projection, obstacles or complete fluid timestep |
| [Ideal-gas flow](periodic-euler-gas.md) | Periodic compressible Euler cell averages, conservative Rusanov fluxes, first-order default or explicit native [second-order reconstruction/SSPRK2](periodic-euler-second-order.md), strict stage CFL and stored positivity audits | Independent contact/nonlinear-wave spatial and temporal refinement, exact Sod/two-rarefaction comparisons, periodic-image controls, conservation, charged retries and late rollback | Limiting reduces local order near extrema and discontinuities; homogeneous ideal gas only, no viscosity, heat conduction, reactions, rigid walls or multiphase flow |
| [Soft bodies](soft-bodies.md) | Mass-spring networks, axial damping, fixed anchors and external loads | Oscillator phase/refinement, momentum, damping and bounded step tests | Spring networks are not calibrated continuum solids; no self-contact, tearing or fluid coupling |
| [Elastic waves](elastic-wave-grid.md) | Homogeneous isotropic periodic plane strain, staggered velocity/stress, P and S waves | Independent negative-adjoint incidence and Fourier operators, second-order continuum refinement, fixed-step modified energy, stress compatibility and means | Linear small strain; no displacement tracking, interfaces, forcing, damping, free surfaces, plasticity or fracture |
| [Thermal networks](thermal-networks.md) | Lumped heat capacities, conduction, reciprocal Stefan–Boltzmann exchange, queued power and fixed-temperature reservoirs | Analytical exponential/radiative cooling, first-order refinement, maximum principle and energy accounting | Caller-supplied exchange coefficients; no geometry/view-factor solver, spectral transport, moving heat transport, phase change or automatic mechanical coupling |
| [Charged particles](electromagnetic-particles.md) | Nonrelativistic planar motion in prescribed uniform electric and magnetic fields | Cyclotron radius/handedness, crossed-field drift, work and step composition | Fields do not respond to particles; no relativistic dynamics or particle-field coupling |
| [Electrostatic grid](periodic-electrostatic-grids.md) | Static homogeneous periodic Poisson solve for prescribed neutral grid charge, zero potential gauge and zero harmonic electric field | Independent dense/Fourier solves, second-order continuum refinement, stored Gauss/curl residuals, bounded reported neutrality correction and field/source/residual energy identity | No nonneutral background, point-charge deposition, interfaces, particle feedback or clock evolution |
| [Maxwell grid](maxwell-grids.md) | Homogeneous periodic TMz, synchronous double Ez/Hx/Hy fields, lossless or explicitly requested [Ohmic evolution](maxwell-ohmic.md) | Independent Fourier/damped modes, second-order temporal/spatial and Joule refinement, fixed-step modified energy/contraction, div H and separate work accounts | No imposed sources, charge coupling, material/conductor interfaces, absorbing boundaries, temperature feedback or 3D components |
| [N-body gravity](nbody-gravity.md) | Planar Newtonian point masses, optional Plummer softening, bounded pair work | Binary phase/refinement, potential-gradient forces, momentum and orbital energy | O(N²), no collision/merging, 3D or relativity; distinct from World's uniform Gravity force |
| [Scalar membrane waves](wave-membranes.md) | Uniform linear membrane, fixed/periodic edges, damping and acceleration loads | Discrete eigenmodes, continuum refinement, energy envelope and damped analytical modes | Small-displacement scalar equation; no bending stiffness, general elasticity or automatic fluid/solid coupling |

Native APIs are installed through `PhysicsEngine::Engine` and the
`physics/physics.h` entry point. Rigid bodies/joints, particles, soft bodies,
thermal networks, charged particles, gravity, membrane and elastic waves, MAC projection/viscosity,
scalar transport, ideal-gas flow, electrostatics, lossless/Ohmic Maxwell fields and spatial queries have [JavaScript bindings](webassembly.md).
The complete incompressible timestep, SPH solvers and rigid-fluid coupling currently
require the native API.
Ideal-gas browser stepping currently exposes the first-order method; the explicit
second-order option is native only.
Binding coverage does not imply all native observers or internal details are exposed.
The [WASM exception boundary](wasm-exception-boundary.md) preserves resources
across repeated validation failures and uses a pinned SDK with stress checks.

Standalone modules own separate state. An application that exchanges loads
between them must define synchronization, units, reaction forces and energy
accounting. Simply sharing positions does not implement a physical coupling.
WCSPH's [rigid coupling](fluid-rigid-coupling.md) is an explicit supported path;
the experimental [planar reflected-source operator](planar-reflected-experiment.md)
remains a diagnostic prototype with documented consistency limits.
The [bounded planar contact interval](planar-contact-interval.md) experiment
resolves stopping and restart times for eligible prescribed-plane inputs;
it has no World bridge and does not resolve production stack regressions.

## Reproduce and extend

The standard CTest run includes analytical/regression tests and bounded headless
examples. Hosted CI builds Linux and Windows native consumers, WebAssembly,
the separate SFML visualizer, and sanitizer configurations. Expected-failing
fluid cases are reported separately and remain unresolved even when CI is green.
See [numerical validation](numerical-validation.md) for detailed commands.

The [coverage roadmap](https://github.com/joaocarloscruz/2d-physicsengine/issues/52)
tracks additions and open numerical work. New models should specify their
equations, units, boundaries, independent physical oracle, refinement behavior,
conservation/dissipation properties and bounded failure behavior before being
described as supported. Nonlinear solids, general gas equations of state,
spectral radiation transport, phase change, 3D, relativistic and quantum models
require separate formulations.

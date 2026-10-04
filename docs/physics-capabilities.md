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
| [Periodic MAC grid](periodic-mac-projection.md) | Double face velocities and periodic pressure projection | Discrete Fourier modes, spatial refinement, means, pressure gauge and residual-aware energy identities | Projection alone is not a complete fluid time step; no advection, obstacles or free surface |
| [Soft bodies](soft-bodies.md) | Mass-spring networks, axial damping, fixed anchors and external loads | Oscillator phase/refinement, momentum, damping and bounded step tests | Spring networks are not calibrated continuum solids; no self-contact, tearing or fluid coupling |
| [Thermal networks](thermal-networks.md) | Lumped heat capacities, conduction, queued power and fixed-temperature reservoirs | Exponential pair relaxation, refinement, maximum principle and energy accounting | No moving heat transport, radiation, phase change or automatic mechanical coupling |
| [Charged particles](electromagnetic-particles.md) | Nonrelativistic planar motion in prescribed uniform electric and magnetic fields | Cyclotron radius/handedness, crossed-field drift, work and step composition | Fields do not respond to particles; no relativistic dynamics or particle-field coupling |
| [N-body gravity](nbody-gravity.md) | Planar Newtonian point masses, optional Plummer softening, bounded pair work | Binary phase/refinement, potential-gradient forces, momentum and orbital energy | O(N²), no collision/merging, 3D or relativity; distinct from World's uniform Gravity force |
| [Scalar membrane waves](wave-membranes.md) | Uniform linear membrane, fixed/periodic edges, damping and acceleration loads | Discrete eigenmodes, continuum refinement, energy envelope and damped analytical modes | Small-displacement scalar equation; no bending stiffness, general elasticity or automatic fluid/solid coupling |

Native APIs are installed through `PhysicsEngine::Engine` and the
`physics/physics.h` entry point. Rigid bodies/joints, particles, soft bodies,
thermal networks, charged particles, gravity, waves, MAC projection and spatial
queries have [JavaScript bindings](webassembly.md). SPH solvers and rigid-fluid
coupling currently require the native API. Binding coverage does not imply all
native observers or internal solver details are exposed.

Standalone modules own separate state. An application that exchanges loads
between them must define synchronization, units, reaction forces and energy
accounting. Simply sharing positions does not implement a physical coupling.
WCSPH's [rigid coupling](fluid-rigid-coupling.md) is an explicit supported path;
the experimental [planar reflected-source operator](planar-reflected-experiment.md)
remains a diagnostic prototype with documented consistency limits.

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
described as supported. General solid elasticity, compressible gases, radiation,
phase change, 3D, relativistic and quantum models require separate formulations.

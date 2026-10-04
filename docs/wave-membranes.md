# Scalar waves and uniform membranes

`WaveMembrane` is a standalone rectangular-grid solver exported by
`physics/core/wave_membrane.h` and the supported `physics/physics.h` entry point.
It models linear, small transverse displacement of a uniform taut membrane:

\[
u_{tt}=c^2(u_{xx}+u_{yy})-2\gamma u_t+a,\qquad c^2=T/\sigma.
\]

Owned JavaScript grids are also available through [WebAssembly bindings](webassembly.md#owned-scalar-wave-grids), with copied snapshots and checked integer inputs.

Displacement u is in metres, velocity in m/s, queued acceleration a in m/s²,
tension T in N/m and surface density sigma in kg/m². `damping` is gamma in 1/s;
the velocity decay coefficient is twice gamma. T and sigma must be strictly
positive and finite. Damping must be finite and nonnegative. This scalar model
has no automatic World/Engine coupling and does not model large-deformation
membranes, full elastodynamics, compressible-fluid acoustics or Maxwell fields.

```cpp
#include <physics/physics.h>
using namespace PhysicsEngine;
WaveMembraneConfig config;
config.tension = 12;
config.surfaceDensity = 3; // c = 2 m/s
config.damping = 0.1;
WaveMembrane membrane(33, 25, 0.03125, 0.03125, config);
membrane.setCellState(16, 12, 0.001, 0);
membrane.queueAcceleration(16, 12, -0.2);
membrane.step(0.01);
auto energy = membrane.getDiagnostics();
const auto& displacement = membrane.getDisplacements();
```

The immutable geometry consists of width Nx, height Ny and separate positive
spacings dx and dy. Arrays are row-major, index y*Nx+x. Bulk `setState` takes
complete displacement and velocity vectors; per-cell setters use (x,y).
Observers return const vectors; copy them when a persistent snapshot is needed.
`setState` and `setCellState` reject nonfinite values before changing state.
`setConfig` validates the complete configuration and boundary invariants before
publication; lowering `maxCells` below the existing geometry is rejected.

`FixedZero` (default) includes the boundary nodes at x=0 and x=(Nx-1)dx,
y=0 and y=(Ny-1)dy. Both dimensions must contain at least three nodes. Edge
displacement and velocity remain exactly zero; nonzero edge state or acceleration
is rejected. `Periodic` requires at least two nodes per dimension and has periods
Nx*dx and Ny*dy, with no duplicate endpoint. Switching to `FixedZero` requires
all existing edge displacement, velocity and queued acceleration to be zero.

`queueAcceleration` adds a prescribed acceleration, constant throughout the next
accepted positive step. It is an acceleration rather than a total force: multiply
by sigma*dx*dy to obtain the force on an evolving grid node.
`getQueuedAccelerations`, `clearAcceleration(x,y)` and `clearAccelerations`
inspect and clear these loads. `step(0)` changes no state or loads and resets only
last-step work counters. Any failed step leaves all state, loads, time and prior
diagnostic counters unchanged. A successful positive step consumes the loads.

## Discretization, stability and work limits

The Laplacian uses centered symmetric differences with distinct dx/dy weights.
Velocity Verlet advances the undamped, forced system, enclosed by exact damping
half steps multiplying velocity by exp(-gamma*h). This symmetric split is second
order for smooth solutions, including forcing with damping. Constant periodic
undamped fields have exact free-motion/constant-acceleration evolution up to
roundoff. Uniform damped velocity follows exp(-2*gamma*t); its displacement and
the damped forced solution are second-order approximations rather than exact
solutions. Large gamma*h can reduce accuracy despite stability.

The two-dimensional wave bound is
c²*h²*(1/dx²+1/dy²) <= 1, as derived by [Langtangen and Linge](https://hplgit.github.io/fdm-book/doc/pub/book/html/._fdm-book008.html).
`getStableTimeStep` rounds the computed physical bound
`cflSafety/(c*sqrt(1/dx²+1/dy²))` down, then takes the smaller of that bound and
the exact configured `maxSubstep`. Safety must be strictly
between zero and one; its default is 0.9. Each requested dt is partitioned into
equal steps no larger than that bound. Substep count is rounded up without
relaxing the CFL bound, so a request exactly on a floating-point limit may need
one extra substep. Damping does not relax the wave CFL requirement.

Default budgets are 1,000,000 cells, 4,096 substeps, and 64,000,000 grid-cell
substeps per call. `maxCellWork` bounds Nx*Ny*substeps before any step-state
allocation or computation. Each cell substep performs two stencil evaluations
and a fixed number of integration operations; staging and diagnostics add O(Nx*Ny)
work. Dimension products are checked before grid allocation. Budget exhaustion
rejects the call, without partial advancement or discarded loads. Configured
budgets may be changed explicitly. The default maximum substep is 0.01 seconds.

Derived stencil and energy coefficients must remain positive and finite in
double precision. Inputs outside that representable range are rejected explicitly.
Coefficient construction can also reject extreme parameter combinations when
an intermediate overflows or underflows, even if a rescaled formulation could
represent the final coefficient. Individually tiny energy terms may underflow
to zero during evaluation or be lost when added to a much larger total; energy
diagnostics do not promise arbitrary-scale accuracy.
Steps reject nonfinite arithmetic, unrepresentable final physical energies,
underflowed half-step durations and time increments too small to advance the
double clock. Finite input state can still have unrepresentable physical energy;
diagnostic queries reject this rather than reporting infinity. State is always
double precision; use adequate spatial resolution as well as a small timestep.

## Energy and diagnostics

Discrete kinetic energy is sigma*dx*dy/2 times sum(v²). Strain energy is
T*dx*dy/2 times the sum of squared forward edge slopes, using dx or dy as
appropriate. Each ordinary interface and each periodic wrap interface is counted
once. On a two-node periodic axis, two distinct directional interfaces connect
the same nodes; both are required to match the centered Laplacian. Fixed-edge
velocities are zero, so equal node masses give the evolving interior energy.

These are physical discrete energies, not an exactly conserved quantity of the
Verlet integrator. Stable undamped motion has bounded energy oscillation;
damping reduces motion while forcing can add energy. No exact continuous-energy,
force-work or damping-loss accounting is claimed. `getDiagnostics` reports kinetic,
strain and total energy, maximum absolute state values, simulation time, the
current timestep bound, and the last accepted substep size/count and cell work.

Build/run `wave_membrane_demo` for a headless fixed-edge sine-mode example.
Tests use independent discrete Fourier/sine frequencies and continuum solutions
to measure temporal/spatial order, including anisotropic grids and damping.

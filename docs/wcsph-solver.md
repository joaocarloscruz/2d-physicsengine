# Reference weakly-compressible SPH solver

`WcsphSolver` is the engine's CPU reference implementation of
weakly-compressible smoothed particle hydrodynamics. It is intentionally
headless: boundary geometry, rigid-body coupling, and rendering are separate
layers built on top of this solver.

The formulation follows the conservative SPH equations described in
[Cossins' review](https://arxiv.org/abs/1007.1245) and the WCSPH equation of
state and timestep approach documented by
[DualSPHysics](https://github.com/DualSPHysics/DualSPHysics/wiki/3.-SPH-formulation).

## Default model

The default `WcsphConfig` is the configuration validated by the WCSPH physical
targets in `tests/test_fluid_physics_validation.cpp` (issue #44), all of which
pass at their original thresholds. Each piece answers a measured defect:

| Setting | Default | Why |
| --- | --- | --- |
| `densityMode` | `Continuity` | Summation density cannot represent a resting column: a uniform lattice has no pressure and free-surface particles read 16-21% low by construction. |
| `kernelFamily` | `WendlandC2` | At `h = 2dx` (eight neighbours) the symmetric pressure term's uniform-pressure stiffness `sum_j H(r_ij)(1 - cos k.r_ij)` has negative transverse eigenvalues for the spiky and cubic gradients, so a pressurized lattice buckles (predicted growth 15.8 and 23.9 1/s; columns slide vertically against each other near the floor). Wendland C2 is positive semidefinite there. |
| `timeIntegration` | `SymplecticEuler` | The continuity rate is evaluated from the kicked velocity, then position and density drift together. The legacy `ExplicitDensity` order made the velocity/density acoustic pair a forward-Euler map whose growth depended on `dt` (87 J vs 1.3 J at `dt` 0.001 vs 0.0005 in a ramped column). |
| `acousticDamping` | `0.5` | Bulk viscosity: the momentum equation uses `p + beta*h*c*(D rho/Dt)`, equivalent to `zeta = beta*h*c*rho` acting only on velocity divergence, so it always removes energy and leaves shear untouched. It damps the startup sound wave of a column released uncompressed (amplitude about `g*H/c`), which neither physical viscosity nor density diffusion removes. Adds the timestep limit `0.125*h/(beta*c)`, equal to the acoustic CFL limit at the default. |
| `particleShifting` | `0.02` | Fully supported particles drift by `-s*h*c*dt*grad(C)`, `C = sum_j V_j W_ij` over fluid and wall volumes. It heals disorder that pressure cannot see (the checkerboard mode has zero linear density response). Free surfaces are detected from the smallest eigenvalue `lambda` of the kernel moment matrix, which wall samples complete: no shift for `lambda < 0.55`, normal component removed for `0.55 <= lambda < 0.9`. A perfect lattice, including its free surface, does not move. Adds the limit `0.05*h/(s*c)`. Shifting behaves as diffusion only when the kernel stiffness is positive semidefinite (Wendland at `h = 2dx`); set it to 0 with Poly6/Spiky or cubic. Shifted particles do not receive a Taylor correction of density or velocity. |
| `hydrostaticInitialization` | `RestDensityParticles` | See [hydrostatic initialization](#hydrostatic-initialization-and-wall-completion): `prepare(particles, boundaryParticles)` seeds wall-supported fluid at rest density. |
| `wallConsistency` | `ConstantPressureCompletion` | Makes a uniform pressure exert no net wall force where wall samples nearly complete the stencil, so the floor force balance converges with spacing. |

The previous behaviour remains available explicitly:
`densityMode = Summation`, `kernelFamily = Poly6Spiky`,
`timeIntegration = ExplicitDensity`, `acousticDamping = 0`,
`particleShifting = 0`, `hydrostaticInitialization = Disabled`,
`wallConsistency = RawSamples`. Tests and diagnostics that characterize those
operators select them explicitly. For summation density, prefer
`CubicSpline`: its square-lattice sum at `h = 2dx` is 1.0009, against 1.0146 for
Poly6 and about 1.038 for Wendland C2 (or calibrate masses with
`SphKernels2D::SquareLatticeMassScale`).

Measured results at the validated defaults (Windows Release Clang 23.1.4).
"Before" values come from issue #44, or were measured at commit ad06fe4
(continuity density, old kernel and integrator) where the issue gives none.

| Target | Threshold | Before | Now |
| --- | --- | --- | --- |
| Uniform lattice interior density | error < 1% | 1.46% | exact (continuity state) |
| Zero-gravity block at rest | speed < 0.05 m/s, displacement < 5 mm | 0.227 m/s | 0, 0 |
| Disorder heals | final RMS < 0.9 x initial | ratio 1.000 | 0.467 |
| Resting column hydrostatic pressure | RMSE < 0.1, acceleration < 1 m/s^2 | 0.548, 9.81 | 0.086, 0.050 |
| Dummy wall balances column weight | residual < 0.1 | 0.63 | 0.0019 |
| Wall balance converges (dx 0.1/0.05/0.025) | -20% per step, < 0.1 | 0.018/0.165/0.236 | 0.0032/0.0014/0.00085 |
| Sampled-wall column quiet, uncompressed start | speed < 0.2 m/s, bulk density < 5% | bulk density 5.2% | 0.167, 0.96% |
| Calibrated column bounded | speed < 0.5 m/s, density < 5% | density 27.8% | 0.174, 0.91% |
| Gravity ramp reduces startup impulse | peak < 0.7x, density < 0.8x immediate | ramp increased peak speed | 0.32x, 0.60x |

The dynamic results are unchanged between `dt = 0.001` and `0.0005` (final speeds
0.1665/0.1669 and 0.1737/0.1738 m/s; ramp ratios identical), so they are not
tuned to one timestep. `acousticDamping = 0.5` is the one chosen coefficient:
final speeds at 0.5 s for beta 0/0.3/0.4/0.5 are 0.325/0.202/0.179/0.161 m/s.
Wall columns oscillate slightly out of phase with the interior because the wall
continuity term carries only normal velocity; a tangential wall-velocity
treatment could let the damping drop.

## State preparation

For each particle `i`, density is estimated from the particle itself, every
fluid neighbor, and optional sampled boundary support inside its smoothing
length:

```text
rho_i = sum_j m_j W_density(x_i - x_j, h_i)
```

Pressure uses the weakly-compressible Tait-style equation of state:

```text
B_i = rho0_i c^2 / gamma
p_i = B_i ((rho_i / rho0_i)^gamma - 1)
```

The defaults use `gamma = 7`. Negative pressure is clamped to zero by default
to avoid tensile attraction at an unresolved free surface; this policy can be
disabled explicitly in `WcsphConfig`.

## Symmetric pair forces

Pressure and viscosity are accumulated once per ordered neighbor pair. The
same force is added to particle `i` and subtracted from particle `j`, so
internal pair forces conserve linear momentum:

```text
F_pressure_ij = -m_i m_j
    (p_i / rho_i^2 + p_j / rho_j^2) grad W_pressure_ij

F_viscosity_ij = mu_ij m_i m_j / (rho_i rho_j)
    (v_j - v_i) laplacian W_viscosity_ij
```

The common pair pressure gradient above uses `(h_i+h_j)/2` for the legacy
family and for continuity density. Matched cubic **summation** density instead
uses each coefficient with its own density-kernel gradient:

```text
F_pressure_ij = -m_i m_j [
    (p_i / rho_i^2) grad W_cubic(x_i-x_j, h_i)
  + (p_j / rho_j^2) grad W_cubic(x_i-x_j, h_j)]
```

This is the negative gradient of the summation-density barotropic energy for
fixed supplied smoothing lengths, including unequal supports and one-sided
neighbours. Equal supports retain the original arithmetic path. It does not
provide density-adaptive `h`, grad-h corrections or a boundary energy model.
See [fixed-support pressure and diagnostics](sph-fixed-support-pressure.md).

External acceleration is applied as `m_i a_external`. Semi-implicit Euler then
updates velocity before position for each solver substep.

## CFL-aware stepping

Viscosity uses dynamic `mu`, a density-aware continuum limit and a prepared
neighbor diffusion bound; see [Explicit fluid viscosity and timesteps](fluid-viscosity.md)
for units, the discrete energy-stability derivation and compatibility details.

`getStableTimeStep()` limits explicit integration using the smallest smoothing
length, artificial sound speed, maximum particle speed, viscosity, the
configured CFL factor, and an absolute maximum timestep. `step()` subdivides a
larger caller timestep automatically and exposes the completed substep count.

```cpp
PhysicsEngine::WcsphConfig config;
config.speedOfSound = 20.0f;
config.cflFactor = 0.25f;

PhysicsEngine::WcsphSolver solver(0.5f, config);
solver.step(particles, frameTime);
const auto& statistics = solver.getLastStatistics();
```

Statistics include density range, maximum speed, the current stable timestep,
substeps, deterministic neighborhood metrics, and boundary sample/candidate
counts. Substep callbacks allow a higher-level coupled simulation to advance
rigid bodies at exactly the same cadence without exposing solver internals.

The regression suite includes fixed hydrostatic-column and dam-break particle
layouts. A simple test-only box clamp keeps those scenarios bounded until the
production boundary model is implemented; both runs are required to remain
finite and bitwise repeatable. The clamp is not part of `WcsphSolver` and is
not presented as a physical boundary treatment.

Production static containment is provided separately by the circle and convex
polygon models documented in [fluid-boundaries.md](fluid-boundaries.md).
For measured scaling and reproduction commands, see
[fluid-performance.md](fluid-performance.md).


## Kernel-family selection

Set `config.kernelFamily` to `SphKernelFamily::CubicSpline` or
`SphKernelFamily::WendlandC2` to opt into a
[matched scalar density weight and radial gradient](sph-kernels.md).
The default `Poly6Spiky` retains existing dynamics. An unknown enum value is
rejected during configuration validation. Support `h`, caller mass and rest
density are unchanged; lattice mass calibration remains a caller choice.
The Muller viscosity Laplacian and continuum/neighbor diffusion bounds are
independent of this selector; a changed computed density can still change its
viscosity coefficient and resulting stable timestep.

Diagnostics use the selected family's **common mean-support gradient**, even
for heterogeneous matched summation density. These rates are a comparison
operator, rather than the derivative of each particle's actual summation density.
WCSPH additionally includes prepared
wall mirror rates in its raw compression/divergence diagnostics, for each
family; previously these diagnostics omitted walls. Density diffusion is
excluded from that raw operator measurement. The legacy two-argument free
`MeasureFluidDiagnostics` retains the legacy family with no wall rates.
DFSPH retains its existing free-surface projection and has no boundary solver.
This selector is an experimental formulation choice, not a guarantee of
hydrostatic equilibrium, disorder healing or freedom from pairing/tensile
instabilities. See the [consistency diagnostic](fluid-consistency-diagnostic.md).

## Public diagnostic input checks

`MeasureFluidDiagnostics` validates the finite position and velocity and positive
finite mass, density, rest density and smoothing length that its measurement
consumes. It checks both indices before accessing a supplied pair, including
empty particle arrays. Bad fields or additional-rate shapes throw
`std::invalid_argument`; out-of-range indices throw `std::out_of_range`.
Nonfinite intermediate differences, rates, accumulation or normalized summaries
throw `std::overflow_error` instead of returning a misleading finite maximum.
Inputs are never mutated, including when an error follows earlier valid pairs.

The finite-input float arithmetic and ordering are retained. A caller-supplied
pair list is accumulated as written: repeated pairs count repeatedly, reversed
pairs represent the same interaction, and self pairs contribute zero. Producing
a complete, nonduplicated neighbor set remains the caller's responsibility.
Unused pressure, force, viscosity, volume and cached density-rate fields are not
validated by this observer. These checks do not change the comparison operator
or make it a true derivative of summation density.

## Hydrostatic initialization and wall completion

Continuity density is particle state, so a fresh lattice starts at rest density
with zero pressure. `prepare(particles, boundaryParticles)` (not `step`, and not
`step(0)`) seeds particles whose density is still exactly `restDensity` when
`hydrostaticInitialization = RestDensityParticles` (default). Each particle links
upward through fluid within half a spacing laterally; the free surface sits half a
spacing above the top of the chain. Only columns whose lowest particle has
pressure-bearing wall samples straddling it below are seeded; free-falling fluid
and explicitly set densities are left alone. Pressure follows a Tait column with
`g / M`, where `M` is the measured first moment of the kernel-gradient stencil
(doubled lower half, iterated with the seeded volumes); `M` is not one at finite
`h/dx` (about 0.89 at `h = 2dx`, 0.92 at `2.5dx` for Poly6/Spiky; it is computed
for the active kernel family), so this is what lets the discrete pressure force
carry the weight. Floor-supported particles
integrate the profile from the particle above.

`wallConsistency = ConstantPressureCompletion` (default) adds `p_i (S_fluid +
S_wall)` to fluid particles whose wall samples nearly complete their stencil
(mismatch below 5 %, fading out by 10 %), so a uniform pressure exerts no force.
Without it, the floor force of a compressed column beside rest-spaced, staggered
samples has a defect that does not shrink as `dx` is refined at fixed `h/dx`.
Measured on the refinement case (dx 0.1/0.05/0.025, `h = 2.5dx`): residual
0.0032/0.0014/0.00085, also first order at `h/dx` 2, 3 and 3.5. `RawSamples`
keeps the old operator (used by the wall-quadrature audit probes).

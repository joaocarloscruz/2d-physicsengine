# Reference weakly-compressible SPH solver

`WcsphSolver` is the engine's CPU reference implementation of
weakly-compressible smoothed particle hydrodynamics. It is intentionally
headless: boundary geometry, rigid-body coupling, and rendering are separate
layers built on top of this solver.

The formulation follows the conservative SPH equations described in
[Cossins' review](https://arxiv.org/abs/1007.1245) and the WCSPH equation of
state and timestep approach documented by
[DualSPHysics](https://github.com/DualSPHysics/DualSPHysics/wiki/3.-SPH-formulation).

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

Set `config.kernelFamily = SphKernelFamily::CubicSpline` to opt into the
matched cubic scalar density weight and radial pressure/continuity gradient.
The default `Poly6Spiky` retains existing dynamics. An unknown enum value is
rejected during configuration validation. Support `h`, caller mass and rest
density are unchanged; lattice mass calibration remains a caller choice.
The Muller viscosity Laplacian and continuum/neighbor diffusion bounds are
independent of this selector; a changed computed density can still change its
viscosity coefficient and resulting stable timestep.

Diagnostics use the selected gradient. WCSPH additionally includes prepared
wall mirror rates in its raw compression/divergence diagnostics, in both
families; previously these diagnostics omitted walls. Density diffusion is
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

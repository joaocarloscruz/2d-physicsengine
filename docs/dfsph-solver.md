# DFSPH solver

`DfsphSolver` and `WcsphSolver` implement `IFluidSolver::step` and `getDiagnostics`.
Use the same particle initialization and the same diagnostic operator to compare
methods. DFSPH uses a density projection before advection and a divergence projection
after rebuilding neighbors. Each pressure system uses relaxed Jacobi iterations,
with symmetric pair forces and an exact diagonal for unequal masses. The algorithm
follows the [DFSPH educational derivation](https://learn.physics-simulation.org/examples/dfsph.html)
and [Bender and Koschier's method](https://animation.rwth-aachen.de/media/papers/2015-SCA-DFSPH.pdf).

Negative pressure is clamped. This is a free-surface liquid formulation: it limits
compression but permits expansion and underdensity. It does not force every surface
particle to rest density or make all signed divergence vanish. Viscosity is explicit;
advection is symplectic Euler. Timestep limits cover advection, external acceleration
and viscosity; there is no acoustic speed-of-sound restriction.

Viscosity uses dynamic `mu`, a density-aware continuum limit and cached neighbor
diffusion rates. [Explicit fluid viscosity and timesteps](fluid-viscosity.md)
documents units, the discrete energy-stability bound and compatibility details.

## Diagnostics and convergence

- `maximumDensityError`: maximum absolute relative density error, including surface deficits.
- `maximumCompression`: maximum positive relative density error.
- `maximumAbsoluteDensityRate`: maximum absolute SPH density rate divided by rest density, in s⁻¹.
- `maximumCompressionRate`: the positive part of that rate, in s⁻¹.
- `densityResidual` and `divergenceResidual`: maximum final pressure-solve residuals across substeps.
- Iteration counts, substeps, and `converged`: bounded-work outcomes, not timing estimates.

Density tolerance refers to the *linearly predicted* compression. The subsequently
measured geometric density can differ because positions are updated nonlinearly.
Both are exposed. Iteration exhaustion returns `converged=false`; substep exhaustion
throws. Tight tolerances and larger patches require more iterations. Defaults are
200 iterations, 0.001 relative predicted compression and 0.01 s⁻¹ compression rate.

## Reproducible comparison

Run `fluid_solver_comparison comparison.csv`. It uses 0.1 spacing, 0.25 support,
lattice-calibrated mass, zero gravity/viscosity, inward velocity `v=-x`, and a 0.01 s
interval, at 121 and 441 particles. WCSPH uses sound speeds 5 and 20; DFSPH uses
0.0001 density tolerance and 0.01/0.001 s⁻¹ divergence tolerance with a 2,000-iteration cap.

In a Windows Clang release build, the 121-particle WCSPH cases reached about 2.0%
compression and 1.93–2.17 s⁻¹ compression rate. DFSPH reached about 0.37% compression
and the requested 0.01/0.001 s⁻¹ rate limits. The tighter projection used 368 divergence
iterations rather than 178. At 441 particles, it used 519 rather than 174. These
results demonstrate an accuracy/work tradeoff for this benchmark; DFSPH took longer
than WCSPH here. Elapsed timings are recorded by the executable and depend on hardware.

Maximum absolute density error remains roughly 44–46% at the free surface in these
small unsupported patches; a compression improvement is not a claim of eliminating
that deficit. Container boundaries, moving boundary samples, and two-way rigid
coupling are currently supported by WCSPH only. Do not switch a coupled WCSPH scene
to DFSPH without implementing and validating equivalent boundary handling.

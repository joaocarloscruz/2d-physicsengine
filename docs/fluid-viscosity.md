# Explicit fluid viscosity and timesteps

`FluidParticleProperties::viscosity` and `FluidParticle::viscosity` are **dynamic
viscosity** `mu`. In this two-dimensional model, mass is measured in kg, density
in kg/m² and dynamic viscosity in kg/s. Kinematic viscosity is `nu = mu/rho`,
measured in m²/s. To specify a kinematic value, supply `mu = rho * nu` using the
intended material density. The distinction follows the diffusion formulation
documented by [DualSPHysics](https://github.com/DualSPHysics/DualSPHysics/wiki/3.-SPH-formulation),
with dimensions adapted to this engine's 2D density and kernels.

WCSPH and DFSPH apply the same symmetric internal force for each neighbor pair:

```text
lambda_ij = mu_avg * m_i * m_j / (rho_i * rho_j) * laplacian(W_viscosity_ij)
F_ij = lambda_ij * (v_j - v_i)
F_ji = -F_ij
```

The coefficient is non-negative and has units kg/s. Opposite pair forces
conserve linear momentum, including unequal masses, densities and viscosities.
The continuum diffusion limit is computed per particle as
`dt <= 0.125 * h_i² * rho_i / mu_i`. Zero viscosity contributes no diffusion
restriction. Current prepared density is used, including density deficits at
free surfaces; rest density alone cannot describe the applied pair coefficient.

Both solvers also bound every prepared explicit step using the actual neighbor
operator:

```text
row_i = sum_j lambda_ij / m_i
dt <= 0.5 / max_i(row_i)
```

For frozen positions and densities, the velocity update is diffusion by the
graph Laplacian `L` with mass matrix `M`. `M^-1 L` is similar to the symmetric
positive-semidefinite matrix `M^-1/2 L M^-1/2`, and its largest eigenvalue is at
most `2 * max(row_i)`. The chosen step keeps every explicit Euler modal factor
between zero and one, so viscosity alone cannot increase the mass-weighted
kinetic energy. It also makes each velocity update a convex combination of
neighbor velocities. Pressure, external forces and boundary work have their own
energy effects; this bound does not certify total energy decay when they act.

WCSPH accumulates row rates during force assembly. Its public
`getStableTimeStep(particles)` reports continuum/CFL/force limits from the supplied
state; `prepare()` and `step()` also apply the neighbor row limit and expose the
combined bound in `getLastStatistics().stableTimeStep`. DFSPH caches each pair's
coefficient and the row bound after its full density summation, then reuses those
same coefficients in force assembly. Both solvers rebuild the operator after
advection, retaining their existing configured substep budgets.

Coefficients, rate sums and integration intermediates use double arithmetic.
Final force/state components outside float range throw `std::overflow_error`;
a timestep below the positive float domain throws `std::runtime_error` instead
of silently becoming zero. Representable timesteps round down to preserve the
conservative bound. Finite large intermediate products therefore remain valid
when their final values fit the state representation.

The particle API, default viscosity values and pair-force law retain their
existing meaning. The corrected timestep interpretation may add substeps in
low-density viscous states and remove unnecessary viscosity restrictions at high
density. Callers that previously treated `viscosity` as kinematic must convert it
to dynamic viscosity. This stability correction does not change pressure,
hydrostatic, or existing fluid-characterization thresholds, and does not resolve
the unsupported WCSPH regimes characterized in issue #44.

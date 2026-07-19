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

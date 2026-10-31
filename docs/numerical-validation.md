# Numerical validation and scalar precision

> **Rigid-body rows re-measured on `526570c` (2026-10-28), Windows x86_64,
> Release, Clang 23.1.4,** with `numerical_validation --mode fast` and
> `--mode extended` on the default configuration (speculative step order,
> `solverSubsteps = 4`). Both profiles pass. The rows that changed from the
> previously printed values are the free-fall position at 10 s and the three
> free-fall timestep rows; every other measured value, including the
> elastic-collision momentum and energy, the oscillator rows, the 100,000-step
> run, the float/double comparison and the extended profile's `1.1307` and
> `0.9921%`, is unchanged to the digits printed. The sections on module precision
> that follow do not depend on the rigid-contact solver and were not re-measured.

Precision is module-specific. Rigid bodies, ordinary particles and SPH use
`float` state and geometry; the fixed-step wall-clock accumulator uses `double`.
Soft-body positions/velocities also use float `Vector2`, with double masses,
spring coefficients, queued forces and diagnostic accumulation. Standalone
charged particles, N-body gravity, thermal networks, membranes, elastic and
electromagnetic fields, electrostatics, MAC/scalar grids and ideal-gas grids use
double state. These choices do not imply that either precision is sufficient
at every scale; each module documents its representability limits.

## Validation profiles

The release-mode `numerical_validation` executable checks the rigid/particle
float operating profile and writes machine-readable
JSON containing expected values, actual values, absolute errors, tolerances,
pass/fail status, and float-versus-double timing measurements.

```powershell
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target numerical_validation
build\numerical_validation --mode fast --output numerical-validation.json
```

Fast mode is registered as `numerical_validation_fast` in CTest, so every
normal native CI run gates on it and publishes its JSON in the GitHub job
summary. The manually triggered `Extended Numerical Validation` workflow runs
ten times as many drift and scalar-comparison steps and publishes a separate
job summary.

Its accuracy table below does not cover every standalone module. Their
independent physical oracles, refinement measurements, conservation/dissipation
audits and failure tests are linked from the
[capability map](physics-capabilities.md) and included in the broader native
test suite. Browser parity has separate physical and ownership checks.

## Published accuracy budget

These are the checked expectations. All errors are absolute unless marked
relative.

| Scenario | Expected | Fast tolerance | Measured locally |
| --- | ---: | ---: | ---: |
| Free-fall position after 10 s | `-440.5` | `0.01` | `0.0012817` error |
| Free-fall velocity after 10 s | `-94.1` | `0.002` | `0.0006271` error |
| Elastic-collision momentum | `3.0` | `0.0001` | `4.77e-7` error |
| Elastic-collision energy | `18.5` | `0.001` | `2.38e-7` error |
| Free-fall position, dt `1/30` | analytical | `0.001` | `5.13e-6` error |
| Free-fall position, dt `1/60` | analytical | `0.001` | `7.10e-7` error |
| Free-fall position, dt `1/240` | analytical | `0.001` | `1.97e-5` error |
| Oscillator position, dt `0.1` | `cos(10)` | `0.25` | `0.2284` error |
| Oscillator position, dt `0.05` | `cos(10)` | `0.125` | `0.1057` error |
| Oscillator position, dt `0.025` | `cos(10)` | `0.0625` | `0.05091` error |
| Fine/coarse oscillator error ratio | `0` | `0.5` | `0.2229` |
| Free motion, 100,000 steps | analytical | `0.05` | `0.005913` error |
| Float/double position difference | `0%` | `0.5%` relative | `0.2724%` |

The finer constant-acceleration runs expose a useful limitation: truncation
error is already negligible, so accumulated float roundoff does not shrink as the
step gets smaller (`7.10e-7` at `1/60`, `1.97e-5` at `1/240`; the fast profile's
fine-minus-coarse roundoff check measured `1.45e-5`). The oscillator case exercises changing acceleration and confirms
real timestep convergence: reducing the step from `0.1` to `0.025` reduces the
position error by more than a factor of four.

The extended local profile measured `1.1307` units of drift after one million
free-motion steps against a `1.5`-unit budget, and a `0.9921%` float/double
relative difference against a `5%` extended budget. These deliberately harsh
profiles make long-duration degradation visible without defining that regime
as the normal operating envelope.

## Precision decision

Single precision remains the rigid-body, ordinary-particle and SPH scalar:

- normal-scale trajectories and collision invariants fit the published error
  budget;
- changing every public vector, shape, material, and WebAssembly value would be
  an ABI/API break;
- compact float state remains useful for the planned particle and fluid data
  sets;
- the fixed-step accumulator is already double precision, preventing frame-time
  bookkeeping from adding avoidable float error.

This decision is not based on a speed advantage. Across repeated local release
runs, the arithmetic microbenchmark measured float/double time ratios from
about `0.85` to `1.07`; the difference is small and noisy.
Precision should be reconsidered, or made configurable, if a normal CI profile
exceeds these budgets, if supported world scales grow materially, or before an
application requires million-step trajectories without rebasing.

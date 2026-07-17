# Numerical validation and scalar precision

The engine uses `float` for simulation state and geometry, and `double` for the
fixed-step wall-clock accumulator. This is an explicit operating policy rather
than a claim that single precision is universally sufficient.

## Validation profiles

The release-mode `numerical_validation` executable writes machine-readable
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

## Published accuracy budget

These are the checked expectations. All errors are absolute unless marked
relative.

| Scenario | Expected | Fast tolerance | Measured locally |
| --- | ---: | ---: | ---: |
| Free-fall position after 10 s | `-440.5` | `0.01` | `0.0036011` error |
| Free-fall velocity after 10 s | `-94.1` | `0.002` | `0.0006271` error |
| Elastic-collision momentum | `3.0` | `0.0001` | `4.77e-7` error |
| Elastic-collision energy | `18.5` | `0.001` | `2.38e-7` error |
| Free-fall position, dt `1/30` | analytical | `0.001` | `7.16e-6` error |
| Free-fall position, dt `1/60` | analytical | `0.001` | `1.43e-5` error |
| Free-fall position, dt `1/240` | analytical | `0.001` | `4.21e-5` error |
| Oscillator position, dt `0.1` | `cos(10)` | `0.25` | `0.2284` error |
| Oscillator position, dt `0.05` | `cos(10)` | `0.125` | `0.1057` error |
| Oscillator position, dt `0.025` | `cos(10)` | `0.0625` | `0.05091` error |
| Fine/coarse oscillator error ratio | `0` | `0.5` | `0.2229` |
| Free motion, 100,000 steps | analytical | `0.05` | `0.005913` error |
| Float/double position difference | `0%` | `0.5%` relative | `0.2724%` |

The finer constant-acceleration runs expose a useful limitation: truncation
error is already negligible, so accumulated float roundoff grows as the step
gets smaller. The oscillator case exercises changing acceleration and confirms
real timestep convergence: reducing the step from `0.1` to `0.025` reduces the
position error by more than a factor of four.

The extended local profile measured `1.1307` units of drift after one million
free-motion steps against a `1.5`-unit budget, and a `0.9921%` float/double
relative difference against a `5%` extended budget. These deliberately harsh
profiles make long-duration degradation visible without defining that regime
as the normal operating envelope.

## Precision decision

Single precision remains the engine scalar for the current API:

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

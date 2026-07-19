# Fluid performance benchmark

`fluid_benchmark` measures a complete WCSPH update separately from preparation
of one batched six-vertex quad per particle. It uses deterministic square
lattices, a `1/120 s` requested update, four CFL substeps, three warmups by
default, and reports median and 95th-percentile time over ten samples.

The render metric covers CPU vertex-buffer preparation only. It deliberately
excludes GPU upload, draw, presentation, and window-system timing. This keeps
simulation cost separate and establishes the intended one-batch rendering
workload without making SFML a dependency of the engine benchmark.

## Reproduce

```powershell
cmake -S . -B build-release -DCMAKE_BUILD_TYPE=Release
cmake --build build-release --target fluid_benchmark --parallel
.\build-release\fluid_benchmark.exe `
  --particles 1000,3000,10000 `
  --warmup 3 `
  --samples 10 `
  --output build-release\fluid-benchmark.json
```

CTest also runs a fast 1,000-particle benchmark smoke test. Unit tests enforce
local candidate-pair bounds through 10,000 particles; wall-clock thresholds are
not used in CI because shared-runner timing is not stable enough for a fair
cross-platform limit.

## Local Release measurements

Measurements below were collected on 2026-07-19 using MinGW GCC 16.1 on the
project's Windows development machine. Each row advances `1/120 s` using four
solver substeps.

| Particles | Baseline median | Optimized median | Change | Optimized p95 | Batched preparation median |
|---:|---:|---:|---:|---:|---:|
| 1,000 | 3.591 ms | 3.019 ms | -15.9% | 3.745 ms | 0.005 ms |
| 3,000 | 12.239 ms | 9.244 ms | -24.5% | 13.467 ms | 0.020 ms |
| 10,000 | 38.431 ms | 32.141 ms | -16.4% | 69.012 ms | 0.085 ms |

The optimization preserves lexicographic deterministic output while sorting
small per-particle neighbor ranges instead of one global pair vector and reuses
the previous pair capacity. The 10,000-particle p95 remains noisy and is not a
real-time guarantee. At 3,000 particles the measured solver fits within a
`1/60 s` display budget on this machine when simulation advances at `1/120 s`;
actual renderer draw/present time must still be measured in the visualization.

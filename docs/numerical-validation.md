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

## Seeded property tests

`tests/test_fuzz_narrow_phase.cpp` and `tests/test_fuzz_world.cpp` generate
random inputs from a fixed seed with SplitMix64, so a run is identical on every
platform and standard library. A failure prints the seed, the case index and
every shape, pose and margin as hex floats; pasting those values into a test
reproduces it exactly. The default `[fuzz]` cases run in CI. `[.fuzz-long]`
repeats them over more seeds and steps and is run by hand:
`run_tests "[.fuzz-long]"`.

### Oracle

`tests/fuzz_support.h` treats every shape as a convex core (a point, a segment
or a polygon) rounded by a radius, and evaluates it in double from the exact
float inputs. It shares no code with the engine. Separated pairs use the
brute-force distance between all core edges. Overlapping pairs use the minimum
projected overlap over every core edge normal, which is exact because the
Minkowski difference of the cores is a polygon containing the origin. A
self-check compares this against 20,000 sampled directions.

### Narrow phase, queries and sweeps

Shapes are circles, convex polygons with 3 to 8 vertices in either winding
(including 1/1000 slivers, outlines offset from the local origin and a vertex
within 1e-4 rad of its neighbour) and capsules (a fifth with half-length 0).
Pairs are placed at a common offset of 0, up to 10, 1000 or 1e4, on a 2^-10
grid so integer shifts are exact. Each case checks `CheckCollision` in both
orders, shifted by up to 1e4, and with a random speculative margin.

| Property | Tolerance and reason |
| --- | --- |
| Overlap agrees with the oracle | outside a band of the depth tolerance around exact touch |
| Penetration equals the minimum translation; translating along the normal by it separates | `3e-5` times the larger shape extent: the intentional face allowances are `1e-5` of the polygon width and `2e-5` of the radius ([polygon manifolds](polygon-manifold-numerics.md), [capsules](capsules.md)); everything else is double on exact float inputs |
| Unit normal | `1e-6` |
| Swap: normal negated, same depth and count, same contacts | depth `1e-6` relative; contacts within the position tolerance; circle pairs move their surface-of-A contact by the depth, and coincident circles keep the `+x` fallback ([contact manifolds](contact-manifolds.md)) |
| Translation invariance | depth `1e-6` relative, normal `1e-6`, contacts within the position tolerance |
| Margin: a gap below the margin reports minus the gap, a larger gap no contact | the depth tolerance |
| Contact points | inside both shapes or no farther than their own gap, no deeper than the penetration (plus twice the capsule half-length, since capsule midpoints are measured along the normal); circle pairs on the circle surface, polygon pairs on the incident edge or at the midpoint of a vertex-to-vertex gap, capsule pairs at the midpoint of two surface points the contact depth apart |
| `ContainsPoint`, `OverlapsCircle` (and `CheckCollision` with that circle), `RayCast`, `SweepCircle` | `1e-6` of the extent plus four float ulps of the coordinates; ray and sweep fractions are checked through the clearance at the reported fraction, which stays meaningful for grazing hits |
| `SweepCircleCircle`, `SweepCirclePolygon`, `SweepCircleCapsule` against 2048-step time stepping refined by bisection | as above, plus the float rounding of `SweepHit::fraction` along the travel |

The position tolerance is the depth tolerance plus two float ulps of the
largest world coordinate, because contact points are stored as float world
coordinates.

### World

Each scene seals 50 to 200 random bodies (all three shapes, random density,
restitution and friction) in a static box with 2 m walls, adds revolute,
distance and prismatic joints, applies random impulses and teleports into free
space every 40 steps, toggles sleeping halfway and fires CCD circles at 150 m/s.
Each seed runs in both step orders for 600 steps (1200 in the long run), twice.
The checks are: no exception, finite state, mechanical energy at most 10% (plus
0.5 J) above the initial energy plus everything the test injected, no CCD
budget exhaustion on 5 consecutive steps, bit-identical step-by-step hashes on
the repeat run, and no body escaping the box. Escape is only counted for CCD
bodies and for bodies that never exceeded 60 m/s, half a wall per step, the
range [getting started](getting-started.md#fast-bodies) covers without CCD.

The default bullets hit only walls and each other, are light, and polygons are
well-conditioned. Regression cases in the same file run the harder variants
(bullets that hit bodies, heavy bullets, random outlines) for 600 steps. The
World bugs these runs found, all fixed:

- World CCD threw on a valid polygon: its float-rounded world outline was
  validated again and could fail strict convexity. World CCD now sweeps that
  outline without revalidating it ([CCD](collision-lifecycle-and-ccd.md)).
- CCD impacts spent the whole budget on contacts that were already touching
  and held by a neighbour (two CCD circles stacked on the floor in the default
  run, a light CCD circle under a falling polygon): pairwise impulses passed
  the remaining approach back and forth without advancing time, so every body
  stood still while velocities kept growing. A pair touching again after its
  own impulse is now left to the contact solve, with only the circles' cores
  swept. The same rule ends a loop where an impulse at a spinning polygon's
  contact point could not stop the centres' approach.
- A CCD impact on a sleeping body made the speculative step throw ("Island
  has no solve group").
- Joint and contact impulses could carry a body past the velocity limits; a
  stiff MouseJoint drag reached thousands of m/s. The limits are now applied
  again after the solve.

The long run passes on all 12 seeds. The default `[fuzz][world]` case takes
about 4 s and the long World run 50 to 90 s (machine load) in a Release
build.

### Documented limits, not bugs

- Circle-polygon and circle-circle contacts stay on a circle surface, so a deep
  or sharp-corner contact can lie outside the other shape by up to the depth.
- Polygon clip points lie on the incident edge between the reference face's
  side planes and can lie outside the reference polygon near a sharp corner.
- `SweepHit::fraction` is a float and loses impact-time detail on long travel.

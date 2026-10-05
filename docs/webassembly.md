# WebAssembly build

The WebAssembly target exposes the engine's basic simulation API to JavaScript
through Emscripten's Embind library.

Native validation failures become owned JavaScript `Error` objects. The module
uses the pinned Emscripten 6.0.3 SDK and an [exception-safe binding boundary](wasm-exception-boundary.md)
that also snapshots bounded grid array inputs before native allocation.

Scene queries are available through `queryPoint`, `queryCircle`, `rayCastAll`,
`rayCastNearest`, `sweepCircleAll` and `sweepCircleNearest`. They accept an Engine
and return owned collections with exact BigInt body IDs, retained body handles
and copied hit geometry. See [query arguments, filtering and object cleanup](spatial-queries.md#javascript-queries-and-result-ownership).

## Owned periodic ideal-gas Euler grids

`PeriodicEulerGasGrid` evolves periodic cell averages of density, two momentum
densities and total energy density. `step()` uses the native first-order Rusanov
scheme, including its numerical diffusion and strict multidimensional CFL
bound. The explicit `stepSecondOrder()` option adds conserved MC reconstruction
and SSPRK2 with a stricter bound at each stage. It models one homogeneous ideal
gas; there is no viscosity, reaction,
multiphase flow, rigid wall, or automatic thermal/World coupling. See the
[native equations, units, positivity conditions and range limits](periodic-euler-gas.md).

```javascript
const gas = new physics.PeriodicEulerGasGrid({
    columns: 32, rows: 16, spacingX: 1 / 32, spacingY: 1 / 16, gamma: 1.4
});
let retained;
try {
    const state = gas.state(); // Reduced-unit uniform rho=1, p=1, zero velocity.
    state.density.fill(2);    // Momentum and total energy stay unchanged.
    gas.setState(state);
    const report = gas.step(0.05, {
        cflSafety: 0.9, maxSubstep: 0.1,
        maximumSubsteps: 10000, maximumCellVisits: 100000000
    });
    retained = {state: gas.state(), primitives: gas.primitives(), report};
} finally {
    gas.delete();
}
console.log(retained.primitives.pressure); // Owned arrays remain valid.
```

Default construction uses 16×16 cells, unit spacings and `gamma=1.4`.
`config()` returns all five immutable geometry/model fields. Explicit config
and step options require every field; `step(duration)` supplies the four defaults
shown above. Dimensions are exact integers at least two, with at most 262144
total cells. Substep/work budgets are exact nonnegative integers, capped at
1000000 and 1000000000 respectively. Counts are checked as doubles before native
integer conversion, so large values cannot wrap. Positive stepping charges
`(4+6*substeps)*cellCount` visits and can reject after staged intermediate work.

`state()` and `setState(state)` use four row-major arrays: `density`,
`momentumX`, `momentumY`, `totalEnergy`. `primitives()` returns five arrays:
`velocityX`, `velocityY`, `pressure`, `soundSpeed`, `internalEnergy` (the latter
is an energy density). State inputs must be ordinary dense finite-number arrays
of exactly `columns*rows` entries. The boundary checks all four shapes before
reading entries and snapshots their values before native allocation, using a
captured native geometry getter. Foreign accessor/proxy errors preserve their
JavaScript identity; nested calls and receiver deletion during conversion are
checked before native receiver access. Native validation additionally requires
positive density, positive representable internal energy and finite derived
primitives. Arbitrary finite inputs can still exceed the supported numerical
range; no floors, clamps, or hidden energy corrections are applied.

`step()` returns a complete copied `EulerGasDiagnostics`; `lastStep()` returns
the last successful report and `time()` the owned clock. Both nested `initial`
and `final` summaries expose all twelve native fields, including area-integrated
mass, both momenta, total/internal/kinetic energy, absolute momentum integrals
and density/pressure bounds. The report includes all four conservation defects
and roundoff allowances, duration/clock/substep timing, both maximum signal
speeds, maximum CFL, substeps, cell visits and `zeroDurationNoOp`. These are
plain values with no borrowed views or snapshot cleanup.

`setState()` retains time and historical `lastStep()`. A successful `step(0)`
publishes fresh initial/final summaries while preserving state/time, costs
`3*cellCount` visits and permits a zero substep budget; it can reject an
insufficient work budget. Every failed state update or step retains the state,
clock and previous report. Delete each grid handle once; copied config, arrays,
primitives and reports remain valid after deletion. The Node suite independently
checks convex flux splitting, anisotropic/two-cell grids, exact translating
contact cell averages, nonlinear simple waves on both axes, Sod Riemann averages, refinement, conservation,
positivity, budget/range/clock rollback, deterministic replay and lifetime stress.

`stepSecondOrder(duration)` uses six defaults; its configured overload requires
the complete object below. Its duration and geometry rules are the same as
`step()`, and its native work accounting includes every rejected stage attempt.

```javascript
const gas2 = new physics.PeriodicEulerGasGrid();
try {
    const report = gas2.stepSecondOrder(0.05, {
        cflSafety: 0.9, maxSubstep: 0.1,
        maximumSubsteps: 10000, maximumCellVisits: 100000000,
        maximumAttempts: 20000, maximumRetriesPerSubstep: 16
    });
    const history = gas2.lastSecondOrderStep(); // Owning copy, including summaries.
    console.log(report.substeps, history.attempts);
} finally {
    gas2.delete();
}
```

The four counts are finite exact nonnegative integers checked as doubles before
conversion. `maximumAttempts` has hard maximum 1000000 and counts every initial
candidate and retry; `maximumRetriesPerSubstep` has hard maximum 64 and counts
discarded stage-CFL candidates before acceptance. The sufficient positivity
bound is `2*h*(alphaX/dx+alphaY/dy) <= cflSafety < 1` at both reconstructed stages.
Only a stage-CFL failure retries, with bounded work; arithmetic or stored-state
failures reject the whole duration. There are no floors or energy repairs.

The returned owning `EulerGasSecondOrderDiagnostics` includes every base field
plus `attempts`, `rejectedAttempts`, `reconstructionPreparations`,
`reconstructionTrials`, `limitedSlopeCells`, `positivityLimitedCells`,
`rangeLimitedCells`, `zeroSlopeFallbackCells`, `forwardEulerStages`, `blendPasses`,
`minimumSlopeScale` and `maximumRejectedCfl`. `lastStep()` holds the base report
of either method; `lastSecondOrderStep()` holds the most recent second-order
record across `setState()` and first-order calls. Initially the second-order
record is zero with `minimumSlopeScale=1`. A successful `stepSecondOrder(0)`
publishes fresh records in both observers and charges exactly 3N visits, even
with zero substep/attempt/retry budgets. A failure preserves both records,
fields and clock, including failures after accepted unpublished substeps.

The independent browser second-order suite checks physical MC/Rusanov SSPRK2,
axis exchange/two-cell grids/extrema, identical-input 32/64/128 contact and
nonlinear simple-wave refinements, padded-cubic and independently refined RK4
temporal oracles, cold positivity/fallback, actual stage retries, exact work,
late rollback, clock/range, owning history and all direct duration/options
getter/valueOf deletion paths. Probe-enabled physical smoke and 1000 stress
batches compare exact heap/stack/uncaught/emval counts; the production profile
also checks helper absence. Smooth higher order does not imply second-order
accuracy at shocks or clipped extrema. See [the second-order physical, resource,
range and reproduction contract](periodic-euler-second-order.md).

## Owned periodic electrostatic grids

`PeriodicElectrostaticGrid` solves `-permittivity*Lap(phi)=charge` on an immutable
periodic rectangle, with `E=-grad(phi)` and zero-mean potential gauge. It is a
static solve of prescribed grid charge density: it has no clock, particle
deposition, particle feedback or automatic World/Maxwell coupling. See the
[native equations, staggering, compatibility policy and scale limits](periodic-electrostatic-grids.md).

```javascript
const grid = new physics.PeriodicElectrostaticGrid({
    columns: 2, rows: 2, spacingX: 1, spacingY: 1, permittivity: 2
});
let snapshot;
try {
    const diagnostics = grid.solve([1, -1, 1, -1], {
        absoluteGaussTolerance: 1e-10, relativeGaussTolerance: 1e-10,
        maximumIterations: 1000, maximumCellVisits: 100000000
    });
    snapshot = grid.getSnapshot(); // phi=[.125,-.125,.125,-.125]
    console.log(diagnostics.finalGaussRms, diagnostics.fieldEnergy); // 0, .25
} finally {
    grid.delete();
}
console.log(snapshot.field.xFaces); // [-.25,.25,-.25,.25], still owned.
```

Default construction uses 16×16 cells, unit spacings and permittivity 1.
`solve(charge)` supplies the four solve defaults shown above; an explicit options
object must contain all four fields. `getConfig()` returns all five geometry
fields. Each dimension must be an exact integer at least two, with at most
262144 total cells. Iteration/work counts are checked as doubles before integer
conversion, with hard maxima 1000000/1000000000. Wrapped, fractional, negative
and nonfinite counts reject. Zero iterations are valid when the actual zero-field
candidate meets the chosen tolerance; zero work cannot certify even zero source.
A zero-source success charges 52 logical cell visits per cell. JavaScript copies
and allocation add bounded linear work outside the native stencil-visit count.

Charge is a dense ordinary array of finite primitive numbers, row-major with
exactly `columns*rows` entries. The synchronous boundary uses a privately
captured native geometry getter, checks the complete shape before entries, and
copies charge and the four primitive option fields before wiring the receiver
into native code. Throwing getters/proxies retain their original JS exception;
nested synchronous calls and receiver deletion during capture are checked safely.
Native rejection becomes an owned `Error`. See the supported
[exception boundary scope](wasm-exception-boundary.md).

The snapshot is a plain owning object containing `originalCharge`,
`effectiveCharge`, `potential`, `field:{xFaces,yFaces}`, `gaussResidual`, `curl`
and the complete native `diagnostics`. Every array has one entry per cell;
fields are staggered at faces and curl at corners. Inputs and returned snapshots
can be mutated or retained after deletion without changing the native grid.
No borrowed memory, typed views, vector wrappers or snapshot cleanup are used.
Every successful solve publishes the entire snapshot together; any failed input,
work, convergence, range or physical audit retains the previous snapshot, apart
from any explicit mutation performed by a user callback.

Periodic compatibility requires zero integrated charge. Only the fixed native
roundoff band admits a measured mean correction, with original/effective source,
both integrated charges, removed mean, maximum correction and its allowance
reported. Material nonneutrality rejects. `finalGaussRms` audits the actual stored
field against the effective source; `originalGaussRms` includes the reported
source correction. Absolute Gauss tolerance has charge-density units; relative
tolerance is dimensionless. A deliberately loose tolerance can accept an
inaccurate zero field, so energy agreement alone does not establish accuracy.
Diagnostics include field/source energies, signed residual-energy correction,
its bound and the scale-aware energy identity error/allowance, along with gauge,
field means, curl and bounded work statistics. In SI, charge is C/m³,
permittivity F/m, potential V, field V/m, integrated charge C/m and energy J/m
per unit out-of-plane depth. Unrepresentable derived quantities reject; finite
inputs do not promise arbitrary-range accuracy.

The shared Node smoke helper checks analytic two-cell and anisotropic Fourier
fields, an independent reduced dense solve, continuum refinement, physical stored
Gauss/curl/energy audits, source scaling, neutrality, ownership, replay and late
rollback. Probe-enabled stress repeats foreign/native failures and array/options
reentrancy/deletion while checking exact stack, live allocation and exception
counter stability.

Local validation on 2026-10-04 used Emscripten 6.0.3 and Node 22.16.0 on Windows,
with at most two build workers. Full smoke/stress passed in optimized Release
with probes ON and Debug `-O1 -fsanitize=address,undefined -fno-omit-frame-pointer`
with linker `-fsanitize=address,undefined -g`, using
`ASAN_OPTIONS=halt_on_error=1` and
`UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1`. Electrostatic stress runs
1000 batches/23000 listed rejections plus array/options nesting and receiver
deletion. Its warmed Release stack/live-heap/uncaught counters remained
142912/16416/0; sanitized counters remained 312269264/4965/0. These compare each
build against itself. The physical helper also checks four-cell weighted-Gauss
cases with `dx=dy=3e-154`, `permittivity=1e-308` and charge amplitudes `1e150`
and `1e154`: the latter has an unrepresentable raw opposing-face difference,
while normalized potential, field and energy match the analytic answer.
Potential/face-field continuum refinement ratios approach 4.003/4.002.
A separate Release build with probes OFF passed full smoke and verified that
all boundary test classes/functions and the stack observer are absent. The
[reproduction commands below](#owned-plane-strain-elastic-waves) cover the same
shared smoke/stress entry points; no larger stack or test-only reset was used.

## Owned periodic scalar transport

`PeriodicScalarTransport` owns periodic cell-average density `q` and prescribed,
frozen MAC face velocities. Default construction uses 16×16 unit-spaced cells;
the configured constructor takes the complete geometry object below. It solves
`q_t + div(u*q) = 0` using first-order unsplit donor-cell transport. Compressible
flow changes an initially uniform density while conserving its integral.
It has numerical diffusion and does not advance/project velocities or couple
to Engine, World, SPH, thermal state or a shared clock. See the
[native equations, accuracy, units and representability limits](periodic-scalar-transport.md).

```javascript
const scalar = new physics.PeriodicScalarTransport({
    columns: 2, rows: 2, spacingX: 0.25, spacingY: 0.5
});
let values, diagnostics;
try {
    scalar.setState([1, 2, 1, 2]);
    scalar.setVelocities([0.25, 0.25, 0.25, 0.25], [0, 0, 0, 0]);
    diagnostics = scalar.step(0.1, {
        cflSafety: 0.9, maxSubstep: 0.1,
        maximumSubsteps: 10000, maximumCellVisits: 100000000
    });
    values = scalar.getState(); // [1.1, 1.9, 1.1, 1.9]
    console.log(scalar.getTime(), scalar.getVelocities());
} finally {
    scalar.delete();
}
console.log(values, diagnostics); // Owned snapshots survive deletion.
```

`getConfig()`, `getState()`, `getVelocities()` and `getLastStep()` return plain
copied objects/arrays with no borrowed WASM memory or snapshot cleanup.
`getTime()` returns the independent clock in time units. Geometry is immutable.
`setState(values)` and `setVelocities(xFaces, yFaces)` copy inputs; later input
or snapshot mutation cannot affect the simulation. Setters preserve the clock
and complete last successful diagnostic, which describes that earlier operation.
Delete the simulation handle once when finished.

All three arrays have `columns*rows` entries indexed `i + columns*j`.
Cell averages are centered at `((i+.5)*dx,(j+.5)*dy)`, x faces at
`(i*dx,(j+.5)*dy)`, and y faces at `((i+.5)*dx,j*dy)`.
Periodic faces have no duplicate final row/column. Inputs must be dense plain
JS arrays of finite numbers; typed arrays, holes, strings and nonfinite values
are rejected. Both face lengths are checked before either array's entries are
read, and all entries are copied in JS before native allocation. Privately
captured native sizing observers bypass shadowed public getters. Foreign
accessor/proxy exceptions preserve their thrown value, and native rejections
become owned `Error` objects through the same exception-safe boundary.

`step(duration)` uses the four defaults shown above; its configured overload
requires all four fields. Dimensions and budgets enter as doubles and must be
finite exact nonnegative integers before conversion. Each dimension is at least
two, the product at most 262144 cells, `maximumSubsteps` at most 1000000 and
`maximumCellVisits` at most 1000000000. Zero budgets are valid options but fail
when required work is positive. Spacings must be positive/finite with
representable area/domain extents, duration finite/nonnegative, `cflSafety`
strictly between zero and one and `maxSubstep` positive/finite.

Positive duration charges `(7+4*substeps)*cells` logical cell visits; native
allocation/destruction and JS input/output copies are additional bounded linear
work. `step(0)` still validates/audits and requires exactly `3*cells` visits. It
publishes coherent zero-duration diagnostics while retaining state and clock;
even a zero-duration call fails with insufficient work budget. Any failed step
retains scalar, velocities, clock and the full previous diagnostic. Unrepresentable
rates, transfers, summaries or updates fail explicitly without hidden clamping,
rescaling or an absolute scalar floor.

Every native diagnostic is returned by `step` and `getLastStep`:

| Fields | Units / meaning |
| --- | --- |
| `substeps`, `cellVisits` | Accepted partition and charged logical work |
| `duration`, `timeBefore`, `timeAfter`, `lastSubstep` | Time |
| `maximumOutflowRate`, `outflowRateBound`, `maximumAbsDivergence` | 1/time; conservative outflow bound and measured advector divergence |
| `maximumCfl` | Dimensionless accepted substep × outflow bound |
| `initialIntegratedScalar`, `finalIntegratedScalar`, `initialAbsoluteIntegral`, `finalAbsoluteIntegral`, `integratedScalarDrift`, `conservationRoundoffAllowance` | Scalar × length²; absolute integral uses `abs(q)` |
| `initialMinimum`, `initialMaximum`, `finalMinimum`, `finalMaximum`, `rangeRoundoffAllowance` | Scalar units |
| `nonnegativeInput`, `discreteDivergenceFree`, `zeroDurationNoOp` | Initial positivity, exact computed zero-divergence and no-op flags |

The roundoff allowances are scale-aware numerical guards, not physical error
tolerances. Positivity applies to nonnegative input; the old-range bound and
constant preservation additionally require discrete divergence-free velocity.
For compressible flow, density can exceed its initial maximum. Node smoke tests
independently check donor rows/Fourier amplification, two-cell anisotropic
grids, compression, mass, positivity and first-order temporal/spatial refinement.
Probe-enabled boundary stress also checks repeated native/foreign failures,
reentrant getters, shadowed sizing methods and receiver deletion with exact
stack/live-allocation/uncaught-counter stability.

## Owned periodic MAC grids

`PeriodicMacGrid` owns a standalone periodic velocity grid with separate
projection and constant-viscosity diffusion operations.
`new physics.PeriodicMacGrid()` uses 16 columns, 16 rows and unit spacings.
The configured constructor accepts a complete plain object with `columns`,
`rows`, `spacingX` and `spacingY`. Geometry is fixed after construction.
It does not implement advection, walls, free surfaces, applied forces, a complete
Navier–Stokes step or Engine/World/SPH coupling. See the
[native operators, physical units and accuracy limits](periodic-mac-projection.md).

```javascript
const grid = new physics.PeriodicMacGrid({
    columns: 8, rows: 6, spacingX: 0.1, spacingY: 0.17
});
let projection;
try {
    const c = grid.getConfig();
    const n = c.columns * c.rows;
    const xFaces = Array.from({length: n}, (_, k) =>
        0.3 + Math.sin(2 * Math.PI * (k % c.columns) / c.columns));
    grid.setVelocities(xFaces, Array(n).fill(-0.2));
    const diagnostics = grid.project({
        density: 1000, timeStep: 0.01,
        absoluteDivergenceTolerance: 1e-10,
        relativeDivergenceTolerance: 1e-10,
        maximumIterations: 1000, maximumCellVisits: 100000000
    });
    projection = grid.getLastProjection();
    console.log(grid.getVelocities(), grid.getDivergence(), diagnostics);
} finally {
    grid.delete();
}
console.log(projection.pressure); // Plain copied arrays survive grid deletion.
```

Each array uses index `i + columns*j` and has `columns*rows` entries.
`xFaces` stores u at `(i*dx, (j+0.5)*dy)`; `yFaces` stores v at
`((i+0.5)*dx, j*dy)`. Potential, pressure and divergence are cell-centered at
`((i+0.5)*dx, (j+0.5)*dy)`. Periods are `columns*dx` and `rows*dy`;
periodic faces are stored once, without a duplicate last row or column.

`getConfig()`, `getVelocities()`, `getDivergence()` and
`getLastProjection()` and `getLastDiffusion()` return plain copied objects/arrays, including the nested
`diagnostics` object. Changing inputs after `setVelocities`, mutating a snapshot,
or deleting the grid cannot change other snapshots. No borrowed WASM memory,
typed views or vector wrappers are returned; snapshots require no deletion.
Delete the owned grid handle once when finished.

`setVelocities(xFaces, yFaces)` accepts two plain JS arrays with finite numeric
entries. Both lengths are checked before either native copy is allocated or
read; typed arrays, sparse arrays, numeric strings and nonfinite entries are
rejected. Both components are copied and committed together. The last successful
projection snapshot remains available after a velocity setter; it describes
that earlier projection until another projection succeeds.

`project()` uses density 1, pressure-conversion interval 1, absolute and relative
divergence tolerances `1e-10`, 1000 iterations and 100000000 cell visits.
`project(options)` requires all six fields shown above. The accepted target is
`max(absoluteDivergenceTolerance, relativeDivergenceTolerance*initialDivergenceRms)`.
The stored face velocities must meet the recomputed target. Exactly zero
divergence preserves velocities and publishes zero potential/pressure with zero
iterations; diagnostic passes still consume work. `timeStep` converts potential
to pressure and does not advance a clock. All failed calls retain velocities and
the complete last successful projection snapshot.

Dimensions and work/iteration counts are checked as JS doubles before native
integer conversion: negative, fractional, nonfinite and wrapping values are
rejected. Each dimension is at least 2 and the product is at most 262144 cells,
checked before allocation. Iterations are bounded by 1000000 and cell visits by
1000000000. Zero budgets are valid inputs and fail if the operation needs more
work. Native geometry, tolerance, pressure-scale and arithmetic validation also
applies; there is no hidden tolerance floor.

Diagnostics expose every native field with these units (m and s when using SI):

| Fields | Units / meaning |
| --- | --- |
| `iterations`, `cellVisits`, `zeroDivergenceNoOp` | Accepted iteration/pass work counts; exact-zero initial-divergence flag |
| `density`, `timeStep` | Accepted density and pressure-conversion interval in s |
| `initialDivergenceRms`, `finalDivergenceRms`, `targetDivergenceRms`, `removedDivergenceMean` | 1/s |
| `potentialMean` and snapshot `potential` | m²/s, zero-mean gauge |
| `pressureMean` and snapshot `pressure` | `density*potential/timeStep`: Pa for kg/m³ density, N/m for kg/m² density |
| `initialMeanX`, `initialMeanY`, `finalMeanX`, `finalMeanY` | Mean face velocity, m/s |
| `initialKineticEnergy`, `finalKineticEnergy`, `correctionKineticEnergy` | J per meter of depth for kg/m³ density; J for kg/m² density |
| `velocityCorrectionInnerProduct`, `divergencePotentialInnerProduct`, `residualEnergyBound`, `storageEnergyError`, `roundoffEnergyAllowance` | Same energy units, including the cell-area and density weights |

Finite tolerance allows the measured residual energy term; success alone does
not imply exact orthogonality or strict energy decrease. The roundoff allowance
is a scale-aware heuristic guard, with no absolute energy floor. Consult the
native derivation before interpreting these diagnostic pairings as physical work.

## Owned periodic MAC diffusion

`diffuse()` and `diffuse(options)` call the bounded native backward-Euler
constant-viscosity solve on both periodic face components. Projection remains a
separate operation: diffusion damps divergence modes but does not eliminate
divergence or automatically project the result. See the [discrete equation,
energy identity, means and numerical limits](periodic-mac-diffusion.md).

```javascript
const grid = new physics.PeriodicMacGrid({
    columns: 2, rows: 2, spacingX: 0.25, spacingY: 0.5
});
let copiedDiffusion;
try {
    grid.setVelocities([1, -1, 1, -1], [0.5, 0.5, 0.5, 0.5]);
    const diagnostics = grid.diffuse({
        kinematicViscosity: 0.25, timeStep: 0.5, density: 1,
        absoluteVelocityTolerance: 1e-10, relativeVelocityTolerance: 1e-10,
        maximumIterations: 1000, maximumCellVisits: 100000000
    });
    copiedDiffusion = grid.getLastDiffusion();
    console.log(grid.getVelocities(), diagnostics);
} finally {
    grid.delete();
}
console.log(copiedDiffusion.finalKineticEnergy); // Plain value, no delete().
```

The configured overload requires **all seven fields** shown above. Counts are
received as JS doubles, validated as finite nonnegative integers before native
conversion, and checked against the same hard ceilings: 1000000 total iterations
and 1000000000 cell visits. Fractions and wrapping counts are rejected. The
iteration cap is shared across both components. Geometry/allocation limits remain
those of the owned grid. Zero budgets are accepted inputs but fail if more work
is needed; diagnostic passes also count as work.

Default `diffuse()` uses viscosity 0, dt 0, density 1, absolute/relative velocity
tolerances `1e-10`, 1000 total iterations and 100000000 cell visits. Exact zero
viscosity **or** zero dt leaves both velocities unchanged and publishes coherent
no-op diagnostics with zero residual and iterations. Constant fields are exact
fixed points even with positive transport. Density, options and derived arithmetic
still require validation; no simulation clock advances.

The fixed RMS velocity residual target is
`max(absoluteVelocityTolerance, relativeVelocityTolerance*initialVelocityRms)`.
The actual stored equation is audited after both component solves. Means and
energy must meet the native scale-aware checks without an absolute energy floor.
Loose tolerances permit the measured residual-work allowance; success alone is
not a claim of exact energy nonincrease or a complete fluid step.

Both `diffuse`'s return value and `getLastDiffusion()` are independent plain
copied objects exposing every native diagnostic field. They need no deletion,
remain valid after later operations and grid deletion, and cannot mutate the
grid. `setVelocities` and `project` leave the last successful diffusion snapshot
intact. `diffuse` leaves the last successful projection snapshot intact. Invalid
configuration, shared-iteration/cell-work exhaustion, unrepresentable arithmetic
and unattainable stored accuracy retain both velocities and both prior snapshots.

| Diffusion fields | Units / meaning |
| --- | --- |
| `iterations`, `iterationsX`, `iterationsY`, `cellVisits` | Shared total, separate component iterations, charged cell visits |
| `kinematicViscosity`, `timeStep`, `density` | Accepted length²/time, diffusion interval, and energy density |
| `initialVelocityRms`, `finalResidualRms`, `targetResidualRms` | Velocity units; both components use N cells in the combined RMS |
| `initialMeanX`, `initialMeanY`, `finalMeanX`, `finalMeanY` | Mean face velocities |
| `meanRoundoffAllowanceX`, `meanRoundoffAllowanceY` | Velocity units; explicit separate mean-drift bounds |
| `initialKineticEnergy`, `finalKineticEnergy`, `incrementKineticEnergy`, `gradientDissipation` | Energy; J per meter of depth for kg/m³ density, J for kg/m² density |
| `residualWork`, `residualEnergyBound`, `storageEnergyError`, `roundoffEnergyAllowance` | Same energy units; signed residual pairing, its bound, identity discrepancy and arithmetic allowance |
| `zeroTransportNoOp` | True only for exact zero viscosity or zero dt |

## Owned scalar-wave grids

`WaveMembrane` exposes the standalone native uniform membrane solver. Construct
an owned grid with `(width, height, spacingX, spacingY)` for native defaults, or
pass a complete configuration as a fifth argument. The default boundary is
`physics.WaveBoundary.FixedZero`; the other supported value is
`physics.WaveBoundary.Periodic`.

```javascript
const defaults = new physics.WaveMembrane(3, 3, 1, 1);
const config = defaults.getConfig();
defaults.delete();
config.boundary = physics.WaveBoundary.Periodic;
config.tension = 12;          // N/m
config.surfaceDensity = 3;    // kg/m²; wave speed is sqrt(tension/density).
config.damping = 0.1;         // gamma in 1/s; PDE damping is -2*gamma*velocity.
const wave = new physics.WaveMembrane(8, 6, 0.1, 0.2, config);
let snapshot;
try {
    const displacement = Array(48).fill(0);
    const velocity = Array(48).fill(0);
    displacement[2 * 8 + 3] = 0.001; // Row-major: y*width+x, metres.
    wave.setState(displacement, velocity);
    wave.queueAcceleration(3, 2, -0.2); // m/s², held during the next accepted step.
    wave.step(0.01);
    snapshot = wave.getCell(3, 2); // {displacement, velocity, queuedAcceleration}
    console.log(snapshot, wave.getDiagnostics());
} finally {
    wave.delete();
}
console.log(snapshot); // Plain copied values remain safe after deletion.
```

`getWidth`, `getHeight`, `getSpacingX`, `getSpacingY` and `getCellCount` describe
the immutable geometry. `getCell(x,y)` returns a copied scalar snapshot.
`getDisplacements`, `getVelocities` and `getQueuedAccelerations` return fresh
plain JS arrays in row-major order, with double-precision values. These are
copies rather than WASM-memory views; mutation does not modify native state,
and arrays/snapshots require no `delete()`.

`setState(displacements, velocities)` accepts two plain JS arrays, each exactly
width*height long with numeric finite entries. Both lengths are checked before
native array allocation, and input values are copied. Typed arrays and array-like
objects are currently rejected. `setCellState(x,y,displacement[,velocity])`
updates one cell; omitted velocity defaults to zero. `queueAcceleration` adds
to the pending cell load, `clearAcceleration(x,y)` clears one and
`clearAccelerations()` clears all. Displacement and velocity use metres and m/s.

Use the complete plain object returned by `getConfig` for `setConfig` or the
configured constructor. It contains `tension`, `surfaceDensity`, `damping`,
`boundary`, `cflSafety`, `maxSubstep`, `maxCells`, `maxSubsteps` and `maxCellWork`.
Dimensions, coordinates and budget counts arrive as doubles and must be exact
finite nonnegative integers within the WASM integer range before conversion.
Coordinates must refer to existing cells; grid/boundary minima and native budget
limits still apply. Fractional, negative, nonfinite and wrapping counts are
rejected. Invalid state/configuration changes preserve existing state.

Fixed-zero grids include their boundary nodes and require zero displacement,
velocity and loads on every edge. Periodic grids omit duplicated endpoints;
their periods are width*spacingX and height*spacingY. `step(0)` retains loads,
resets last-work counters and does not advance time. Failed positive steps retain
state, queued acceleration, time and prior diagnostics; accepted positive steps
consume acceleration. `getStableTimeStep` reports the CFL/configuration bound.
`getDiagnostics` copies physical kinetic/strain/total energy, maximum absolute
state values, time, stable timestep, last substep size/count and grid-cell work.
The Verlet integrator does not conserve physical energy exactly.

The object owns its grid independently of `Engine`/`World`; no automatic rigid,
fluid or multiphysics coupling is provided. Delete the owned grid once when
finished. See [the native membrane model, CFL, resources and representability
limits](wave-membranes.md) for the supported numerical regime.

## Periodic TMz Maxwell fields

`MaxwellGrid` owns a standalone homogeneous periodic TMz grid. Its default
`step` is lossless; the explicit `stepOhmic` adds homogeneous scalar conductivity.
`new physics.MaxwellGrid()` uses the complete default configuration below;
the configured constructor requires every field. Configuration stays immutable.
Counts arrive as JavaScript numbers and must be positive finite integers within
the native hard ceilings before conversion: 262144 cells, 1000000 substeps and
1000000000 cell visits. Both dimensions must be at least two.

```js
const grid = new physics.MaxwellGrid({
    columns: 16, rows: 16, spacingX: 1, spacingY: 1,
    permittivity: 1, permeability: 1, cflSafety: 0.9, maxSubstep: 0.1,
    maximumSubsteps: 10000, maximumCellVisits: 100000000
});
const fields = grid.getState(); // plain copied {ez: [], hx: [], hy: []}
fields.ez[0] = 1;
grid.setState(fields);
const h = grid.getStableTimeStep();
const invariantBefore = grid.getModifiedEnergy(h);
grid.step(h);
const diagnostics = grid.getDiagnostics();
const divergence = grid.getMagneticDivergence(); // plain copied array
const retained = grid.getState();
grid.delete(); // snapshots remain usable; they need no delete()
```

All fields are synchronous in time. At index `i + columns*j`, Ez is at
`(i*dx,j*dy)`, Hx at `(i*dx,(j+.5)*dy)` and Hy at `((i+.5)*dx,j*dy)`.
Periodic samples are stored once without duplicate end rows/columns. SI fields
are Ez in V/m and Hx/Hy in A/m, spacings in meters, permittivity in F/m and
permeability in H/m. Defaults eps=mu=1 use reduced units. Energy diagnostics are
J per meter of out-of-plane depth. `getMagneticDivergence()` and diagnostic
`magneticDivergenceRms`/`maxAbsMagneticDivergence` measure **div H**, in A/m².
The physical constraint is `div B = mu div H = 0` for uniform permeability;
initial divergence is observed and preserved rather than projected away.

`getConfig()`, `getState()`, `getDiagnostics()` and `getMagneticDivergence()`
return owned plain values. Input state requires three plain dense JavaScript
arrays of exactly `columns*rows` finite numeric entries; typed arrays and sparse
arrays are rejected. All three lengths are checked before allocating/copying
any field. Mutating input or output arrays never aliases the grid. A successful
`setState()` preserves the clock and resets previous step work/reference h.
Invalid state or positive-step failure preserves all fields, time and
diagnostics. `step(0)` is a complete no-op.

`getWaveSpeed()` returns `1/sqrt(eps*mu)`. `getStableTimeStep()` combines the
strict 2D CFL bound and configured maximum duration. `step(dt)` partitions dt
within bounded substep/cell work; accepted arithmetic visits are exactly
`columns*rows*(3*lastSubsteps+1)`, excluding bounded copies/input/getter scans.
Diagnostics copy electric/magnetic/total physical energy, modified energy and
its reference step, all three means/maxima, div H RMS/max, clock, stable duration
and previous actual substep/count/cell visits. Physical energy oscillates.
`getModifiedEnergy(h)` observes the fixed-h invariant; positive h must meet the
strict physical CFL and coefficient representability requirements. Compare
the same h before/after a run; changing h changes this quadratic form.

`stepOhmic(duration, conductivity)` takes finite nonnegative seconds and S/m and
returns an owning plain `MaxwellOhmicStepDiagnostics` object. It exposes all
native work fields: conductivity, duration, start/end time, initial/final physical
energy, `exactJouleEnergy`, `representedElectricEnergyLoss`,
`wavePhysicalEnergyChange`, `modifiedEnergyDissipation`,
`decayStorageEnergyChange`, `physicalBalanceResidual`, substep/count/cell visits.
It adds no configuration fields or persistent heat account. Positive conductivity
uses `N*(8*S+1)` bounded visits and exact-decay/wave/exact-decay splitting;
zero conductivity delegates to the original lossless path with identical fields,
diagnostics and `N*(3*S+1)` budget. Zero duration validates both scalars and is a
complete owner no-op. Invalid scalars, work/range/clock failure or late arithmetic
failure throws an ordinary JavaScript Error with all owner snapshots preserved.

```js
const grid = new physics.MaxwellGrid();
const report = grid.stepOhmic(.1, .8); // Explicit 0.8 S/m, no thermal feedback.
grid.delete();
console.log(report.exactJouleEnergy); // Copy survives deletion; no delete().
```

`exactJouleEnergy` is analytic **decay-subflow** work from represented pre-decay
fields, not exact unsplit-PDE work at finite h. Physical wave energy defect and
fixed-h modified-Q decay loss are separate quantities. The represented loss and
storage discrepancy also include energy-measurement roundoff. Tiny decrements
may report positive analytic work with unchanged rounded fields; no balancing
correction hides that observation. See [the Ohmic derivation, stage accounts,
strict-CFL contraction and finite-range restrictions](maxwell-ohmic.md).

Delete the grid exactly once when finished. It is independent of Engine/World;
there are no imposed charge/current sources, particle coupling, material
interfaces, absorbing boundaries, finite-conductor geometry or 3D components.
See [the native equations, invariant, resource accounting and representability
limits](maxwell-grids.md).

## Owned plane-strain elastic waves

`ElasticWaveGrid` owns a homogeneous isotropic small-strain periodic elastic
continuum, independent of `Engine`. Plane strain fixes `epsilonZZ=0` while the
out-of-plane stress is `sigmaZZ=lambda*(epsilonXX+epsilonYY)`. This is a velocity
and stress wave model, without displacement tracking, large deformation,
material interfaces, forcing, damping, contact, fracture or automatic coupling.
See the [native physical model and validation](elastic-wave-grid.md) for the
negative-adjoint operators, P/S dispersion, compatibility and fixed-step energy
invariant. Both JavaScript constructors are available:

```javascript
const defaults = new physics.ElasticWaveGrid();
const config = defaults.getConfig(); // Complete owning configuration copy.
defaults.delete();
config.columns = 32;
config.rows = 24;
config.spacingX = .0625;
config.spacingY = .125;
config.density = 2;
config.lambda = 3;
config.shearModulus = 2;
config.maxSubstep = .01;
const solid = new physics.ElasticWaveGrid(config);
let saved;
try {
    const state = solid.getState();
    // Fill all five staggered fields at a common initial physical time.
    state.vx.fill(.25); // Uniform translation example, unchanged by periodic stress.
    solid.setState(state);
    const invariant = solid.getModifiedEnergy(.01);
    solid.step(.01);
    saved = {state: solid.getState(), diagnostics: solid.getDiagnostics(),
        rates: solid.getSpatialRates(), compatibility: solid.getCompatibility(),
        sigmaZZ: solid.getOutOfPlaneStress(), invariant};
} finally {
    solid.delete();
}
console.log(saved.diagnostics.totalEnergy); // Copies survive native deletion.
```

The configured constructor requires every field of the complete configuration;
use the default grid's copy or supply this object explicitly:

```javascript
{
    columns: 16, rows: 16, spacingX: 1, spacingY: 1,
    density: 1, lambda: 1, shearModulus: 1,
    cflSafety: .9, maxSubstep: .1,
    maximumSubsteps: 10000, maximumCellVisits: 100000000
}
```

In SI, spacing is m, density kg/m^3, Lamé and shear moduli Pa, time s, velocity
m/s and stress Pa. Energy is J per metre of out-of-plane depth. Density and shear
modulus are positive; a finite negative `lambda` is allowed only for stable
auxetic material with `lambda+2*shearModulus/3>0`. The native derived-range and
strict anisotropic CFL checks remain in force. `getCompressionalSpeed()` reports
`sqrt((lambda+2*shearModulus)/density)`, `getShearSpeed()` reports
`sqrt(shearModulus/density)`, and `getStableTimeStep()` returns the configured
inward CFL/max-substep limit. No inputs or energies are normalized or clamped.

`getState()` returns `{vx,vy,sigmaXX,sigmaYY,sigmaXY}` with five plain dense number
arrays of `columns*rows` entries. Samples are indexed `i+columns*j`, with periodic
neighbors and no duplicated end sample:

| Field | Sample position |
| --- | --- |
| `vx` | `(i*dx,(j+.5)*dy)` |
| `vy` | `((i+.5)*dx,j*dy)` |
| `sigmaXX`, `sigmaYY` | `((i+.5)*dx,(j+.5)*dy)` |
| `sigmaXY` | `(i*dx,j*dy)` |

`setState(state)` validates **all five lengths before reading any entries**.
Every entry must be an own finite number; sparse and typed arrays are rejected.
The synchronous boundary snapshots those arrays before native conversion using
a privately captured native sizing observer; shadowing the public `getConfig`
method or its prototype cannot enlarge preprocessing. Dimensions and configured
budgets are checked as finite positive integers within their hard caps **before
integer conversion**. Each axis needs at least two cells and the cell cap is
262144; the hard substep and cell-visit caps are 1000000 and 1000000000.

All getters return plain owning copies, with no vector wrappers, views or snapshot
cleanup. `getSpatialRates()` returns five copied arrays: `accelerationX` and
`accelerationY` (m/s^2) at the velocity faces; `strainRateXX` and `strainRateYY`
(1/s) at normal-stress cells; `engineeringShearRate` (1/s) at shear corners.
Engineering shear rate is twice the off-diagonal strain rate.
`getCompatibility()` returns the local Saint-Venant strain/length^2 defect;
`getOutOfPlaneStress()` returns cell-sampled `sigmaZZ` in Pa.

`getDiagnostics()` copies every native field: `kineticEnergy`, `strainEnergy`,
`totalEnergy`, `modifiedEnergy`, `modifiedEnergyStep`, `physicalEnergyUpperBound`,
`meanVx`, `meanVy`, `meanSigmaXX`, `meanSigmaYY`, `meanSigmaXY`, `meanSigmaZZ`,
`maxAbsVelocity`, `maxAbsStress`, `maxAbsSigmaZZ`, `compatibilityRms`,
`maxAbsCompatibility`, `time`, `stableTimeStep`, `lastSubstep`, `lastSubsteps` and
`lastCellVisits`. `getModifiedEnergy(h)` observes the native reference energy;
`h=0` gives physical energy. The conserved-reference envelope applies to **fixed
h**, up to roundoff; changing h does not preserve one common discrete invariant.
Arbitrary supported stress and nonzero prestress means are accepted and observed,
not projected onto strains from periodic displacements. A vanishing local
compatibility defect alone does not establish periodic displacement compatibility.

`step(dt)` transactionally advances a bounded number of equal stress-half-kick /
velocity-drift / stress-half-kick substeps. Its work count is `N*(3*s+1)` center
visits as defined by the native model; array conversion/read-only copy costs are
additional. Failed operations retain all five native arrays, clock and previous
diagnostics; positive steps that exceed work, derived arithmetic or clock range
fail. Zero time is a complete no-op. Successful `setState` retains the clock and
resets last-step diagnostics. Subnormal aggregate energies and constant means
are preserved when representable; nonzero energy/means that underflow and mixed
ranges that erase normalized mean contributions are rejected. Do not interpret
finite inputs as unlimited float64 dynamic range.

Foreign getter/proxy exceptions preserve their exact JavaScript identity,
including primitive throws. A callback that deliberately mutates or deletes its
receiver has its normal user effects: failure is not a rollback of user code.
Reentrant receiver deletion is detected before entering the native setter. The
[exception boundary scope](wasm-exception-boundary.md) excludes user-modified
global intrinsics and arbitrary future native calls into foreign JavaScript.
Delete the owned grid handle once; copied observations remain usable afterward.

`smoke-test.cjs` invokes the focused `elastic-wave-tests.cjs` checks alongside the
full physics suite: 24 independent axis/oblique/auxetic/two-cell P/S phase modes,
six continuum refinement sequences, independent spatial adjoint power,
400 fixed-h energy/mean/compatibility steps, compression/shear/DC controls,
ownership/replay, subnormal aggregate energy/means and range/work/clock rollback.
With probes enabled, `boundary-stress.cjs` also runs 1000 elastic batches with
20000 rejected calls, requiring exact live-heap, stack and native exception
counters and unchanged owner snapshots. The same focused helper is copied into
each build with the other Node harness assets.

Local validation on 2026-10-04 used the pinned Emscripten 6.0.3, Node 22.16.0,
Windows x86-64 host and no more than two build workers. Release with probes ON
passed the full physics smoke and boundary stress; Debug with `-O1`,
`-fsanitize=address,undefined -fno-omit-frame-pointer` and linker
`-fsanitize=address,undefined -sASSERTIONS=1` passed the same full suites with
`ASAN_OPTIONS=halt_on_error=1` and
`UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1`. Both included the 512x512
constant subnormal velocity/stress aggregate-energy and mean controls. The
optimized elastic stress kept stack 140304, live heap 16104 and uncaught 0;
the sanitized elastic stress kept stack 312244176, live heap 5781 and uncaught 0.
These exact counters compare each warmed build against itself, not between builds.

A separate Release build with probes OFF passed full smoke and verified absence
of `boundaryTestStats`, `BoundaryTestProbe`, `BoundaryTestValue`,
`BoundaryTestStats`, `boundaryTestFunction` and `_emscripten_stack_get_current`.
Production contains the actual elastic API, without test-only reset/sizing
helpers or an enlarged stack. Hosted CI's existing smoke/stress entry points
include the new checks after normal integration; local validation does not
represent an unpublished hosted run.

Reproduce after activating the pinned SDK:

```sh
emcmake cmake -S . -B build-wasm -DCMAKE_BUILD_TYPE=Release -DPHYSICS_WASM_BOUNDARY_TEST_PROBES=ON
cmake --build build-wasm --target physics_engine_wasm --parallel 2
node build-wasm/wasm/smoke-test.cjs
node build-wasm/wasm/boundary-stress.cjs
emcmake cmake -S . -B build-wasm-sanitized -DCMAKE_BUILD_TYPE=Debug -DPHYSICS_WASM_BOUNDARY_TEST_PROBES=ON \
    -DCMAKE_CXX_FLAGS="-O1 -fsanitize=address,undefined -fno-omit-frame-pointer" \
    -DCMAKE_EXE_LINKER_FLAGS="-fsanitize=address,undefined -sASSERTIONS=1"
cmake --build build-wasm-sanitized --target physics_engine_wasm --parallel 2
ASAN_OPTIONS=halt_on_error=1 UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1 node build-wasm-sanitized/wasm/smoke-test.cjs
ASAN_OPTIONS=halt_on_error=1 UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1 node build-wasm-sanitized/wasm/boundary-stress.cjs
emcmake cmake -S . -B build-wasm-production -DCMAKE_BUILD_TYPE=Release -DPHYSICS_WASM_BOUNDARY_TEST_PROBES=OFF
cmake --build build-wasm-production --target physics_engine_wasm --parallel 2
node build-wasm-production/wasm/smoke-test.cjs
```

On PowerShell, set the sanitizer variables using `$env:ASAN_OPTIONS` and
`$env:UBSAN_OPTIONS` for the current process instead of shell-prefix assignments.

## Prerequisites

Install and activate **Emscripten 6.0.3**, then make sure `emcmake` and `cmake`
are available in the current PowerShell session. The binding cleanup adapter
depends on this SDK's runtime ABI; CMake rejects other versions until the adapter
has been reviewed and its stress suite rerun for an upgrade.

## Build and verify

```powershell
.\build-wasm.ps1
node .\build-wasm\wasm\smoke-test.cjs
```

The build produces `physics_engine.js` and `physics_engine.wasm` in
`build-wasm/wasm`. It also copies the browser and Node smoke harnesses into that
directory. Incremental builds track each source asset: editing a harness or
removing its generated copy refreshes it when `physics_engine_wasm` is built,
without requiring a module relink.
Serve the directory over HTTP and open `index.html` to run it:

```powershell
emrun .\build-wasm\wasm\index.html
```

## JavaScript API

The module exposes `Engine`, `Circle`, `Polygon`, `RigidBody`, `ParticleSystem`,
`Material`, and `Vector2`. Use `createRigidBody` and `createParticleSystem` to
create the shared handles expected by the corresponding `Engine` methods.

```javascript
const physics = await createPhysicsEngineModule();
const engine = new physics.Engine();
const simulationConfig = engine.getSimulationConfig();
simulationConfig.solverIterations = 16;
simulationConfig.fixedTimeStep = 1 / 120;
simulationConfig.maxSubstepsPerAdvance = 8;
simulationConfig.enableLinearVelocityLimit = false;
engine.setSimulationConfig(simulationConfig);
const shape = new physics.Circle(1);
const body = physics.createRigidBody(
    shape,
    {
        density: 1,
        restitution: 0.5,
        staticFriction: 0.6,
        dynamicFriction: 0.4,
    },
    { x: 0, y: 0 },
    false,
);

body.setVelocity({ x: 3, y: 0 });
body.setCollisionCategoryBits(0x00000001);
body.setCollisionMaskBits(0x00000006);
engine.addBody(body);
engine.step(0.5);
console.log(body.getPosition());

// For variable frame time, prefer the backlog-preserving fixed-step runner.
const progress = engine.advance(frameTimeSeconds);
console.log(progress.stepsPerformed, progress.remainingTime);

engine.delete();
body.delete();
shape.delete();
```

Simulation iteration/substep/CCD counts must be positive integers within the
native signed 32-bit range. Particle-system indices and reserve capacities must
be nonnegative integers within the native unsigned 32-bit range; indices must
also identify an existing particle. Fractional, non-finite and overflowing
values throw before mutation instead of truncating or wrapping. `reserve(0)`
is valid. Configuration getters retain the same plain object fields.

Collision filtering uses 32-bit category and mask fields. Two bodies collide
only when each body's category is included in the other body's mask. New bodies
default to category `0x00000001` and mask `0xFFFFFFFF`, preserving the original
collide-with-everything behavior.

`engine.stepFixed()` performs exactly one configured fixed step.
`engine.advance(elapsedTime)` caps work at `maxSubstepsPerAdvance` and returns a
`FixedStepResult`; any excess time remains available through
`engine.getAccumulatedTime()` and is processed by later calls. The runner never
silently discards elapsed time.
Because the cumulative counter is 64-bit, `engine.getTotalStepCount()` returns
a JavaScript `BigInt` (for example, `120n`). Per-call `stepsPerformed` remains a
regular number.

After `step`, `stepFixed`, or `advance`,
`engine.getLastStepStatistics()` returns per-step integration, broad-phase,
contact, solver, and fluid counters. See `docs/simulation-statistics.md` for the
counting and reset semantics.

`RigidBody` owns a cloned shape. The JavaScript shape handle may be deleted
immediately after `createRigidBody`. World and joint shared handles keep bodies
alive independently of JavaScript handles. Delete each JavaScript handle once.
`engine.removeBody(body)` also removes attached joints; `engine.clearBodies()`
removes all bodies and joints.

`createDistanceJoint(a, b, length, localAnchorA, localAnchorB)` and
`createRevoluteJoint(a, b, localAnchorA, localAnchorB)` return shared joint handles
for `engine.addJoint`/`removeJoint`. CCD and waking use `body.setCcdEnabled`,
`isCcdEnabled`, `wake` and `isAwake`. Sleeping is configured through the object
returned by `getSimulationConfig`. Use that complete object when changing fields.
`engine.exportJson(time)` and `engine.exportCsv(time)` return state/statistics text.
Fluid solvers and collision listener subclasses currently have native C++ APIs only.

The revolute factory returns a `RevoluteJoint` handle extending `Joint` with
`setMotor(enabled, speed, maxTorque)`, `setLimits(enabled, lower, upper)`,
`getAngle()` and `getMotorTorque()`. Motor speed uses radians/second and limits
use radians relative to the construction pose. Query settings with
`isMotorEnabled`, `getMotorSpeed`, `getMaxMotorTorque`, `areLimitsEnabled`,
`getLowerLimit` and `getUpperLimit`. The handle remains accepted by
`engine.addJoint` and `engine.removeJoint`; delete it once when finished.
See [joint behavior and limitations](joints-and-sleeping.md).

`createPrismaticJoint(a, b, localAxisA, localAnchorA, localAnchorB)` returns a
`PrismaticJoint` shared handle accepted by the same engine joint methods. It
supports `setMotor(enabled, speed, maxForce)`, `setLimits(enabled, lower, upper)`,
their setting getters, `getMotorForce`, `getTranslation`, `getTranslationSpeed`,
`getTransverseError`, `getAngle`, `getReferenceAngle`, `getAxis` and `getLocalAxis`.
Translation uses signed world distance along A's axis; limits do not subtract
the construction translation. See [slider constraints and controls](prismatic-joints.md).

`ChargedParticle` independently integrates a test charge in prescribed uniform
fields. Use either its default neutral constructor or all four explicit arguments:

```javascript
const charge = new physics.ChargedParticle({x: 0, y: 0}, {x: 2, y: 0}, 3, 6);
charge.step(0.1, {electric: {x: 0, y: 0}, magnetic: 2});
console.log(charge.getPosition(), charge.getVelocity(), charge.getKineticEnergy());
charge.delete();
```

`getPosition` and `getVelocity` return independent plain JavaScript values with
double-precision coordinates. `setState(position, velocity)` validates both values.
Supply the field argument on every `step` call; zero fields give free motion.
The object is independent of `Engine` and owns no borrowed handles. See
[the physical scope, units and numerical limits](electromagnetic-particles.md).

## Owned soft-body simulations

`SoftBody` is an independent mass-spring simulation, with default construction
or a complete configuration object. It does not join `Engine` storage or
automatically collide/couple with rigid bodies, particles or fluids.

```javascript
const defaults = new physics.SoftBody();
const config = defaults.getConfig();
defaults.delete();
config.maxSubstep = 0.001;
const cloth = new physics.SoftBody(config);
try {
    const a = cloth.addParticle({x: 0, y: 0}, {x: 0, y: 0}, 1, true);
    const b = cloth.addParticle({x: 1.2, y: 0}, {x: 0, y: 0}, 1, false);
    cloth.addSpring(a, b, 1, 4, 0);
    cloth.applyForce(b, 0, -2);
    cloth.step(0.1);
    console.log(cloth.getParticle(b), cloth.getSpring(0), cloth.getDiagnostics());
} finally {
    cloth.delete();
}
```

Use `getParticleCount`/`getSpringCount` and `getParticle(index)`/`getSpring(index)`
to inspect topology. They return plain copied JS objects, including nested
position, velocity and force values; snapshots require no `delete()` and remain
safe after the simulation is deleted. `getConfig`, `getUniformAcceleration`,
`getAccumulatedForce(index)` and `getDiagnostics` also return copies.

`addParticle(position)` supplies native defaults; its full form takes position,
velocity, mass and fixed status. `addSpring(first, second, restLength, stiffness)`
defaults damping to zero, or accepts damping as a fifth argument.
`setParticleState(index, position[, velocity])`, `setFixed`, `applyImpulse`,
`setUniformAcceleration` and `setConfig` retain native validation. `applyForce`
accepts either `(index, {x, y})` or `(index, doubleX, doubleY)`; `clearForces()`
clears all pending forces and `clearForces(index)` clears one node.

Indices and configuration counts must be finite exact nonnegative integers
within the WASM index range; indices must also refer to existing elements.
Fractional, negative, nonfinite and wrapped values are rejected before any cast.
Native configuration, topology and step budgets remain in force. Modify a
complete object returned by `getConfig` when changing settings. A failed step
preserves state and queued loads; `step(0)` retains loads without integration,
and a successful positive step consumes them. See [native units and numerical
scope](soft-bodies.md). Delete each owned simulation once when finished.

## Owned thermal networks

`ThermalNetwork` is a standalone heat-capacity graph, independent of `Engine`
storage. It does not automatically heat rigid bodies, deform a soft body or
couple to fluids. Use default construction or a complete configuration object:

```javascript
const heat = new physics.ThermalNetwork();
try {
    const hot = heat.addNode(400, 2); // Kelvin, J/K; fixed defaults to false.
    const cold = heat.addNode(300, 3, false);
    heat.addLink(hot, cold, 1); // W/K
    heat.addRadiationLink(hot, cold, 1e-8); // Effective reciprocal W/K^4.
    heat.applyPower(cold, 4); // W, queued for the next positive step.
    heat.step(0.1);
    console.log(heat.getNode(cold), heat.getLink(0), heat.getDiagnostics());
    const config = heat.getConfig();
    config.maxSubstep = 0.005;
    heat.setConfig(config);
    // new physics.ThermalNetwork(config) also constructs a configured graph.
} finally {
    heat.delete();
}
```

Topology is append-only: `getNodeCount`/`getLinkCount` and `getNode(index)`/
`getLink(index)` expose copied snapshots. `setTemperature(index, kelvin)`,
`setFixed(index, fixed)`, `applyPower(index, watts)`, `clearPowers()` and
`clearPowers(index)` preserve native state/load validation. `getDiagnostics`
reports energies, thermostat heat accounting, temperature bounds and accepted
substeps. `getConfig` returns a complete copied object for `setConfig` or the
configured constructor. All getters produce plain JS values with no borrowed
references, vector wrappers or snapshot cleanup; they remain safe after deletion.

`addRadiationLink(first, second, coefficient)` adds reciprocal
Stefan–Boltzmann exchange with a finite nonnegative effective coefficient in
W/K^4. `getRadiationLinkCount()` and `getRadiationLink(index)` return its separate
append-only count and copied `{first,second,coefficient}`. Conductive and
radiative pairs may coexist; their combined count uses `maxLinks`. With
radiation present, adaptive substeps recompute the stability bound after heating
and diagnostics also report `lastRadiativeVisits`. No extra constructor fields
are required. The caller supplies surface/view-factor physics; these links do
not implement an electromagnetic field or automatic geometry coupling.

The same checked integer input rules and native budgets as `SoftBody` apply.
A failed step preserves temperatures, pending powers and heat accounting.
Zero time retains pending powers; a successful positive step consumes them,
including loads on fixed-temperature nodes. Thermostats track the compensating
reservoir heat separately. See [thermal units, conservation and numerical
scope](thermal-networks.md). Delete each owned network once when finished.

## N-body gravity

`NBodyGravity` owns a standalone double-precision planar gravity simulation.
It has default and full-configuration constructors. The default `G=1` uses
reduced units; consult [native gravity](nbody-gravity.md) for physical scope,
Plummer softening, integration accuracy and encounter/work limits.

```javascript
const orbit = new physics.NBodyGravity();
const gravityConfig = orbit.getConfig();
gravityConfig.maxSubstep = 0.002;
orbit.setConfig(gravityConfig);
orbit.addParticle({x: -0.5, y: 0}, {x: 0, y: -Math.SQRT1_2}, 1);
orbit.addParticle({x:  0.5, y: 0}, {x: 0, y:  Math.SQRT1_2}, 1);
orbit.step(0.5);
const first = orbit.getParticle(0);
const metrics = orbit.getDiagnostics();
orbit.delete();
console.log(first.position, metrics.totalEnergy); // Copied values remain valid.
```

`addParticle(position)` defaults to zero velocity and unit mass. The other
form is `addParticle(position, velocity, mass)`. Use `getParticleCount()`,
`getParticle(index)`, `setState(index, position, velocity)`,
`applyImpulse(index, impulse)`, `getConfig()`, `setConfig(config)`,
`getDiagnostics()` and `step(dt)` for interaction. Index and count-budget
arguments must be finite nonnegative integers in the native index range;
zero budgets are rejected. Snapshots are copied plain values, including nested
position/velocity/diagnostic vectors. Failed steps preserve particles and the
previous successful work counters. No Engine registration or implicit World
coupling is involved. Delete the owned simulation handle when finished.

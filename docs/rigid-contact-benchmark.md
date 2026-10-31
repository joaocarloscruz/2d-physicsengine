# Rigid-contact physical diagnostics

> **Measured on `526570c` (2026-10-28), Windows x86_64, Release, Clang 23.1.4,
> with the default configuration:** speculative step order, `solverSubsteps = 4`,
> block two-point correction, stable polygon feature ids and joint warm starting.
> Each matrix row sets only the iteration count and warm factor from the table.
> The numbers below replace the earlier solver measurements, which are kept only
> where a section is explicitly a historical before/after record. The benchmark
> tool has no flag for the step order or the substep count, so the legacy-order
> and `solverSubsteps = 1` columns come from the same unmodified benchmark
> sources compiled with a locally force-included copy of `simulation_config.h`
> whose two defaults read an environment variable. That harness is not in the
> repository; the default-configuration numbers come from the repository's own
> `rigid_contact_benchmark` binary, and the harness reproduces its quick matrix
> row for row.

`rigid_contact_benchmark` is a bounded native diagnostic built alongside the test
tools with `BUILD_TESTING=ON`. It creates fixtures through `World`, immutable
shapes, body setters, automatic `Gravity`, configuration, statistics and public
collision listeners. It does not call the contact resolver or narrow-phase
functions, and it does not change production physics.

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel 2
build/rigid_contact_benchmark --quick --output rigid-quick.json
build/rigid_contact_benchmark --output rigid-full.json
build/rigid_contact_benchmark --dt 0.008333333333 --iterations 10 \
  --warm-start 0.8 --duration 2 --output rigid-selected.json
ctest --test-dir build -R rigid_contact --output-on-failure
```

Use the executable's `.exe` suffix on Windows when required. Output schema version
1 is JSON; without `--output` it goes to stdout. `--help` lists the controls.
The quick matrix has 120 rows and 7,280 World steps: dt 1/60 and 1/120,
4 and 10 iterations, and warm factors 0 and .8. The full matrix has 432 rows
and 45,630 steps: dt 1/60, 1/120 and 1/240, 4/10/20 iterations, warm factors
0/.8/1, and an additional 12-box stack. Both default to two simulated seconds
for support fixtures. Impact rows contain one instantaneous zero-duration solve,
so timestep refinement does not apply to those rows; they repeat for iteration
and matrix bookkeeping.

Overrides replace one axis of the matrix. dt must lie in [.0001,.1], iterations
must be an integer in [1,64], warm factor in [0,1], duration in [.1,8]. A preflight
cap rejects matrices exceeding 250,000 World steps. Fixtures have at most 13 bodies
and four vertices per polygon. Timing covers setup, stepping and independent
measurement; `elapsedMilliseconds` is nondeterministic and is excluded from
reproducibility comparisons. Body IDs do not appear in output. Identical builds
and platforms reproduce the other reported fields; cross-platform bit identity
is not promised.

## Fixtures and policies

All units are consistent mass/length/time units. Dynamic boxes have width=height=1
and mass=1; the static floor is 20×1 with its top through the origin. Rest and
stacks start at exact geometric contact with centers y=.5,1.5,... and no added
perturbation. The support load is the stored float gravity acceleration (0,-9.81),
reported exactly in each row. The inclined floor and supported box rotate by
the stored float angle pi/12 (15 degrees), with the box initially .5 along the
floor normal. High-friction material has static/dynamic coefficients .8/.6;
the sliding fixture has .05/.03. Both bodies use the same coefficients, so the
solver's geometric-mean material combination yields those coefficients.

Sleeping, both velocity limits and CCD are disabled. Restitution is zero for
support cases. Position correction factor=.8, penetration slop=.005 and the other
contact settings retain the default configuration, including the default
speculative step order and four solver substeps. With four substeps, a row's
iteration count is divided across them (`iterations / 4`, at least one, plus a
relax pass), so the 4/10/20 iteration axis does not mean the same work as in the
single pass. The restitution velocity
threshold is explicitly zero. Rows include actual dt, elapsed simulated duration,
iterations, warm factor, materials, masses, inertias and final body states.

Head-on impacts use two radius-.5 circles, mass A=1 and mass B=1, 10 or 1000, incident velocities
2 and 0, and restitution 0/.5/1. Their centers start at ±.4995, giving .001 overlap.
The off-center case uses a radius-.25 circle with mass=1 and velocity=(3,0), center
(-.749,.3), and a unit square with mass=2 at rest. It has the same small overlap,
zero friction and restitution=.5. Both impact fixtures have zero gravity and
position correction factor zero and call `World::step(0)` exactly once. Incident
velocities and lever geometry are therefore the known public initial state;
there is no integration or gravitational impulse between the reference and solve.
The tool separately reports position/orientation projection, which is zero in
these impulse fixtures. This isolates velocity/angular/energy accuracy from
coordinate changes caused by positional projection.

## Independent measurements and oracles

Post-step penetration comes from double transformed vertices and diagnostic SAT
for polygon pairs, disk distance for circle pairs, and finite-edge distance for
circle/polygon pairs. It does not reuse cached manifolds or their pre-correction
penetration. Peak penetration is the maximum **post-step** value, not the largest
temporary overlap before correction. Displacement and orientation change are
measured against the fixture's initial state. Terminal peak speed covers the last
quarter of the sampled steps, alongside final linear and angular speeds.

Momentum, angular momentum about the fixed world origin, kinetic energy and
gravity potential energy use double measurements of public body state. Angular
momentum includes orbital `m*(x*vy-y*vx)` and spin `I*omega`. Gravity and the static
floor exchange momentum/energy with support fixtures; their totals are observations,
not isolated conservation tests. The impulse fixtures have no external load or
position projection and test isolated conservation directly.

For a frictionless single impact, impulse is
`J=(1+e)*incidentNormalSpeed/(1/mA+1/mB+leverA^2/IA+leverB^2/IB)`.
The normal kinetic-energy loss is
`.5*(1-e^2)*incidentNormalSpeed^2/denominator`. Head-on tests independently use
the closed form from momentum plus Newton restitution. The off-center square
uses its uniform-area inertia `m*(1^2+1^2)/12` and the stored .3f lever. Regression
tolerances cover a few float response-rounding ulps at incident speed, without
substituting solver-produced effective masses into the oracle.

Static support and the high-friction incline have the ideal equilibrium oracle
of zero motion. The low-friction incline has acceleration
`g*(sin(theta)-muDynamic*cos(theta))`, ideal speed `a*t` and displacement `.5*a*t^2`
while its flat support remains on the finite floor. At longer custom durations,
the box can leave the finite floor, so that ideal-plane reference then ceases to
apply. Support errors remain observations rather than arbitrary pass/fail bands.

The report sums public integrated-body, candidate, resolved-contact,
solver-iteration and solved-constraint statistics. It records final/peak retained
contact counts, begin/persist/end callbacks, two-point frames and feature-ID changes.
Retained contact entries and feature persistence are **not cache-hit counters**,
and persistence does not prove a stack is at rest. The private impulse cache is
not inspected. Normal finite rows have classification `observation`, regardless
of physical error. Execution exceptions/nonfinite aggregate states produce
`execution_failure` rows and a nonzero exit; invalid controls and excessive planned
work also fail. Physical accuracy does not determine the tool's exit status.
Inapplicable impact oracle values are JSON null. CTest checks bounded execution,
invalid iteration/budget rejection and independent analytic regressions.

## Measured baseline on the current solver

The tables below were measured on `526570c` (2026-10-28) over two seconds, with
the full 432-row matrix (zero execution failures; 45,630 World steps), and use
the fixtures described above. Length=1 is a box width. "Default" is the
speculative step order with four substeps. "Single pass" is
`solverSubsteps = 1` (the speculative order solved as one rigid pass), and
"legacy" is `enableSpeculativeContacts = false`. These observations are evidence
of accuracy limits, not universal stability claims. The six rows are the ones the
previous revision of this section listed, from solver revision `a862d366`
(before issue #75's internal arithmetic changes); its values are kept in the
last column of the second table for comparison.

Default configuration:

| Fixture | dt | Iterations | Warm | Peak penetration | Maximum displacement | Final maximum speed |
|---|---:|---:|---:|---:|---:|---:|
| Rest | 1/60 | 4 | 0 | 0 | 0 | 0 |
| Rest | 1/120 | 10 | .8 | 0 | 0 | 0 |
| Stack 6 | 1/60 | 4 | 0 | .001807 | .007094 | .020547 |
| Stack 6 | 1/120 | 10 | .8 | .000867 | .003197 | .000886 |
| Stack 12 | 1/60 | 4 | 0 | .005459 | .027952 | .022516 |
| Stack 12 | 1/240 | 20 | .8 | .002202 | .012436 | .000223 |

Maximum displacement for the same rows under each solver. The a862d366 column is
the earlier measurement, not a rerun:

| Fixture | dt | Iterations | Warm | Default | Single pass | Legacy order | a862d366 (old) |
|---|---:|---:|---:|---:|---:|---:|---:|
| Rest | 1/60 | 4 | 0 | 0 | 0 | .005341 | .006311 |
| Rest | 1/120 | 10 | .8 | 0 | 0 | .005085 | .005151 |
| Stack 6 | 1/60 | 4 | 0 | .007094 | .088440 | .085168 | 1.245570 |
| Stack 6 | 1/120 | 10 | .8 | .003197 | .033935 | .041426 | .082295 |
| Stack 12 | 1/60 | 4 | 0 | .027952 | 1.148997 | 1.110446 | 5.763308 |
| Stack 12 | 1/240 | 20 | .8 | .012436 | .054785 | .076763 | .201263 |

Final maximum speed for the same rows: default .020547 / .000886 / .022516 /
.000223 for the four stack rows, single pass .716556 / .023334 / 3.044361 /
.027181, legacy order .553056 / .017697 / 2.840074 / .025090, and a862d366
2.641058 / .083027 / 7.514555 / .150270. The rest rows have zero final speed in
all three current columns (a862d366: .001550 and .0000793).

What the default solver changed:

* The isolated analytic impacts are unaffected: the maximum velocity error over
  the full matrix is 1.11e-7 for the default, single-pass and legacy runs alike
  (the quick matrix gives the same 1.11e-7), and the energy error maximum is
  2.53e-7. The earlier 1.47e-7 figure came from an older solver. The impact
  rows use a zero-duration step with no position correction, and all 270 of
  them report the same maximum errors under the three configurations.
* The rest box no longer sinks into its floor. With exact initial contact its
  peak penetration and maximum displacement are zero in every one of the 27 rest
  rows (legacy order: up to .005341, the configured slop).
* At dt=1/60, four iterations and no warming, the low-friction incline reaches
  downhill speed 4.509492 versus ideal 4.509486 and displacement 4.509495 versus
  ideal 4.509487 (legacy order: 4.514228, an error of .0047). Across the 27
  low-friction rows the largest distance error is 3.0e-5 for the default
  (2.5e-5 single pass, .0047 legacy).
* The high-friction incline under the same controls moves .000265 at most
  (downhill displacement -.000252, i.e. slightly uphill, with final speed
  .001745). The legacy order crept .054786 downhill and the single pass .006958.
  The largest default maximum displacement over all 27 incline rows is .000266.
* The tall stacks no longer collapse at low iteration count. Over all 27 rows
  per fixture the default's worst maximum displacements are .001476 (3 boxes),
  .007094 (6 boxes) and .027952 (12 boxes), against .0240, .0852 and 1.1104 in the
  legacy order and .0230, .0884 and 1.149 in the single pass. The worst 12-box row
  is the unwarmed four-iteration one at dt=1/60.
* Feature changes are zero in all 432 rows (and all 120 quick rows) for the
  default, single pass and legacy order. The earlier six-box row with 123
  feature changes no longer exists. Every stack row retains all its contact
  entries (final persistent contacts 6 for six boxes, 12 for twelve), as before.

What is still a limit:

* The default is not an exact equilibrium. Its worst 12-box row still has a peak
  penetration of .005459 (just above the .005 slop), displacement .027952 and a
  terminal peak speed of .035219; the 3-, 6- and 12-box fixtures end with
  terminal peak speeds up to .0135, .0206 and .0352 without warming, and about
  .002 to .005 with warm factor .8.
* The soft substep model rests on a small compression, so it trades the
  vanishing residuals that a well-converged rigid pass can reach for a
  constant-size sag. Against the single pass, the default has a larger maximum
  displacement in 13 of 27 three-box rows (warm 1 or 20 iterations), 7 of 27
  six-box rows and 6 of 27 twelve-box rows (all at warm 1; for example six boxes,
  dt=1/240, 20 iterations, warm 1: .000067 single pass against .003106). It also
  has the larger terminal peak speed in 12, 9 and 6 of those rows (for example
  three boxes, dt=1/60, 10 iterations, warm 1: 5.6e-17 against 8.9e-6). These
  are small absolute values; they show that one pass with converged warm
  starting can settle further than the soft model does.
* At warm factor 1 the default's peak penetration is nonzero even for converged
  stacks: .000347 (3 boxes), .000907 (6) and .002518 (12), because the soft
  contact acts on any overlap. The single pass reaches smaller residuals in the
  rows listed above, but only with warm starting and enough iterations.
* The fixtures are two seconds long, up to 12 boxes and one 20x1 floor. They do
  not establish behavior for taller towers, other mass ratios or longer runs.

Re-run the same matrix after production changes and compare physical metrics and
work rather than silently widening thresholds or removing the difficult fixtures.

## Polygon geometry comparison (historical, #83)

This section records a before/after comparison of two older solver revisions.
Its values were measured then and were not re-run: reproducing them needs the
two old revisions, not the current tree. The current default values for the
same four quick-matrix rows follow the historical table.

Issue #83's [polygon geometry change](polygon-manifold-numerics.md) was compared
against the same contact solver from `7c0df78` (locally cherry-picked as
`e5d9a31`). The after revision is `a17929b`. Both runs use the unmodified quick
matrix, Windows Release Clang, two-second support fixtures, 120 rows and 7,280
World steps. Neither run had execution failures. Commands:

```sh
rigid_contact_benchmark --quick --output rigid-contact-before83.json
rigid_contact_benchmark --quick --output rigid-contact-after83.json
```

The table reports before → after values. The rest fixture retained zero feature
changes, including the existing separate 600-step resting-patch regression.
Only one of the 120 matrix rows changed its feature-change count: the three-box
stack at dt=1/120, four iterations and no warming changed from 39 to 40. Its
slightly lower displacement accompanied a slightly higher final speed.

| Fixture | dt | Iterations | Warm | Feature changes | Maximum displacement | Peak penetration | Final maximum speed |
|---|---:|---:|---:|---:|---:|---:|---:|
| Rest | 1/60 | 4 | 0 | 0 → 0 | .006310654216 → .006310654323 | .005767737923 → .005767743234 | .001549835487 → .001549835639 |
| Stack 3 | 1/120 | 4 | 0 | 39 → 40 | .025632656460 → .025603808177 | .008495626156 → .008495222195 | .070715280541 → .070871225491 |
| Stack 6 | 1/60 | 4 | 0 | 123 → 123 | 1.245568252716 → 1.245568934502 | .063713628879 → .063713989136 | 2.641057693389 → 2.641055584762 |
| Stack 6 | 1/120 | 10 | .8 | 34 → 34 | .082295470490 → .082295742839 | .011733212670 → .011733220568 | .083026693861 → .083026695886 |

These are reproducible fixture observations, not acceptance bands. The scale
fix preserves the tested ordinary resting behavior but does not resolve the
coarse stack's large displacement or establish accuracy at arbitrary aspect
ratios. Elapsed wall time remains excluded from deterministic comparisons.

The same four rows on `526570c` with the default configuration (quick matrix,
measured 2026-10-28):

| Fixture | dt | Iterations | Warm | Feature changes | Maximum displacement | Peak penetration | Final maximum speed |
|---|---:|---:|---:|---:|---:|---:|---:|
| Rest | 1/60 | 4 | 0 | 0 | 0 | 0 | 0 |
| Stack 3 | 1/120 | 4 | 0 | 0 | .001059532166 | .000397920609 | .004384826403 |
| Stack 6 | 1/60 | 4 | 0 | 0 | .007093906403 | .001807212830 | .020547389984 |
| Stack 6 | 1/120 | 10 | .8 | 0 | .003197193146 | .000866502523 | .000886370137 |

Feature changes are zero in all 120 quick rows (stable polygon feature ids), and
the coarse stack's large displacement noted above no longer occurs.

## Coupled normal solve before/after (#85, historical)

The before/after columns compare two older solver revisions and were not re-run;
the extra column gives the current default on the same rows (`526570c`, measured
2026-10-28). The interpretation after the table is written about the historical
pair, and what holds today follows it.

[Committed compact matrices](data/two-point-contact-e78d60a-clang23.json) compare
the unchanged #78 tool and geometry on baseline `e78d60a` against block solver
`e61f918b`. These are Windows x86_64 / Release / Clang 23.1.1 / LLVM-MinGW runs.
No #83 polygon geometry changes are present. The quick matrix has 120 rows and
the full matrix 432; both before and after have zero execution failures. Each
row preserves the same fixture, dt, iterations, warming, two-second duration,
restitution, friction and position-correction settings. Metric arrays use the
file's `metricColumns` order; timing is intentionally excluded from comparison.

Reproduce on each solver revision with:

```
rigid_contact_benchmark --quick --output contact-quick.json
rigid_contact_benchmark --output contact-full.json
```

The following rows use dt=1/60, four iterations and warm-start factor zero.
Units are those of the original benchmark, with unit-width boxes and gravity
9.81. Terminal speed is the maximum in the last measurement window, not an
equilibrium proof.

| Fixture/metric | Before | After | Current default |
| --- | ---: | ---: | ---: |
| Rest maximum displacement | .006310654 | .005594164 | 0 |
| Rest terminal peak speed | .001549836 | <2e-17 | 0 |
| Stack 3 maximum displacement | .039466849 | .025882346 | .001476288 |
| Stack 3 terminal peak speed | .143089017 | .075699497 | .013459422 |
| Stack 6 maximum displacement | 1.245568253 | .132167443 | .007093906 |
| Stack 6 maximum angle change | .392756552 | .015142203 | 0 |
| Stack 6 terminal peak speed | 2.641057693 | .611564854 | .020547390 |
| Stack 12 maximum displacement | 5.763417436 | 4.364174749 | .027952194 |
| Stack 12 terminal peak speed | 7.514678190 | 5.800470342 | .035218537 |
| High-friction incline downhill displacement | .049150199 | .054801652 | -.000252485 |
| High-friction incline terminal peak speed | .003454870 | .006303004 | .001747899 |

At the time of #85, the large stack improvement was useful but left substantial
unsupported motion at low iteration count. The high-friction incline **regressed**: coupled normals
followed by sequential tangents do not guarantee exact static friction balance.
Other rows also worsen. Examples from the full matrix are:

* Stack 3, dt=1/60, four iterations, warm=.8: maximum displacement
  .025323955 → .026761522.
* Stack 6, dt=1/120, ten iterations, warm=1: maximum displacement
  .040788640 → .041782490.
* Stack 12, dt=1/120, twenty iterations, warm=1: maximum displacement
  .093813828 → .095684075.
* Stack 12, dt=1/120, four iterations, warm=0: peak penetration
  .100649223 → .106642150, despite the improvement in worst-case matrix maxima.

Low-friction sliding is nearly unchanged: full-matrix maximum displacement
4.514235288 → 4.514235749. Small per-row increases also exist, including
downhill displacement 4.510639973 → 4.510706119 at dt=1/240, ten iterations and
no warming. The isolated analytic impact maximum velocity error is 1.11e-7
afterward; independent impact momentum/angular/energy tests pass. These tool
observations retain their original classification and do not replace physical
acceptance criteria with a successful benchmark exit.

On the current default the picture differs from both historical columns. The
tall-stack motion at low iteration count is gone (12 boxes: .027952 against
4.364175), and the high-friction incline downhill displacement is -.000252, a
tiny uphill value, so the #85 regression no longer shows in these rows. The
examples above read, for the default: stack 3 (dt=1/60, 4 iterations, warm .8)
maximum displacement .000897; stack 6 (dt=1/120, 10 iterations, warm 1) .003105;
stack 12 (dt=1/120, 20 iterations, warm 1) .012426; stack 12 (dt=1/120, 4
iterations, warm 0) peak penetration .003491. Low-friction sliding over the
full matrix has a largest downhill displacement of 4.509508 (ideal 4.509487),
and at dt=1/240, ten iterations and no warming 4.509478. The isolated analytic
impact maximum velocity error is 1.11e-7. Which of these improvements comes
from the block solver and which from the speculative order and substeps was not
separated by this re-measurement.

No old thresholds were widened and no difficult fixtures were removed. The
remaining friction/position/manifold coupling and tall-stack limits require
separate work; the [normal block derivation](contact-solver.md#coupled-two-point-normal-impulses)
states its conditioning and approximate fallback explicitly.

## Integration-order control for incline creep

`contact_integration_diagnostic` isolates displacement that occurs before the
velocity constraints run. It uses a 15-degree 20x1 static floor and a unit box
of mass 1, both with friction 1 and restitution 0. Position correction, warming,
sleeping, CCD and velocity caps are disabled; velocity tolerance is zero and
64 solver iterations bring the post-solve velocities close to zero. Three
two-second runs use 120, 240 and 480 steps, for a bounded total of 840 World
steps. This artificial control permits penetration to accumulate; it is not
a production resting-contact setup or a replacement for the benchmark matrix.

For a body starting each step at rest, the current force integrator has already
advanced its position by `0.5*g*dt^2` before a contact impulse can stop its
velocity. Projected downhill, the accumulated term is
`0.5*g*sin(theta)*N*dt^2 = 0.5*g*sin(theta)*T*dt`. Thus a near-zero post-solve
velocity does not imply a stationary position. The tool also sums the measured
old-velocity contribution `sum(v_old_downhill*dt)` and reports the remaining
displacement separately. That remainder includes float position storage and
does not presume an unmeasured friction force.

Measured on `526570c` (2026-10-28), Windows Release Clang 23.1.4. The program
builds its worlds from a default `SimulationConfig`, so the step order and the
substep count are whatever the library default is; the other two columns come
from the local environment-variable harness described at the top of this page.

Legacy order (`enableSpeculativeContacts = false`). These values reproduce the
earlier measurement from solver revision `0666abe` digit for digit, so the
legacy order is unchanged:

| Steps | Measured downhill displacement | Force-integration term | Peak post-solve speed |
| ---: | ---: | ---: | ---: |
| 120 | .0423169950 | .0423169212 | 3.01e-16 |
| 240 | .0211575719 | .0211584606 | 1.55e-16 |
| 480 | .0105778603 | .0105792303 | 7.82e-17 |

Default (speculative order, `solverSubsteps = 4`) and `solverSubsteps = 1`:

| Steps | Default downhill displacement | Default peak post-solve speed | Single-pass downhill displacement | Single-pass peak post-solve speed |
| ---: | ---: | ---: | ---: | ---: |
| 120 | 2.01e-5 | 8.55e-4 | 0 | 2.73e-16 |
| 240 | 2.03e-5 | 1.42e-3 | 0 | 1.37e-16 |
| 480 | 2.04e-5 | 1.57e-3 | 0 | 6.83e-17 |

The ideal static-equilibrium displacement remains zero. In the legacy order
these observations localize an O(dt) integration-order contribution, equal to
the force-integration term above to within 1.4e-6. The speculative order
solves velocities before integrating positions, so that term is gone: the single
pass holds the box at exactly zero displacement, and the default's substepped
soft solve leaves a constant 2.0e-5 that does not shrink with dt, with a post-solve
speed of up to 1.6e-3, which the rigid single pass does not. The tool's `remainder` field
(measured displacement minus the force-integration term) is a legacy-order
decomposition and is large and negative (-.0423, -.0212, -.0106) in the new
orders, where the force-integration term no longer enters the position; it
should not be read as an error there. The #85 friction/stack regressions are
discussed above. Any integration change must also retain ballistic accuracy,
impact timing, CCD, loads, joints and existing event semantics. Follow-up work is
tracked in [issue #91](https://github.com/joaocarloscruz/2d-physicsengine/issues/91).

Build with `BUILD_TESTING=ON` and run `contact_integration_diagnostic` (or `.exe`
on Windows). It prints deterministic JSON with the stored float dt/gravity/angle,
all three displacement terms and peak velocity/rotation. A successful exit and
the CTest smoke entry confirm bounded finite execution, not physical acceptance;
the tool does not require future implementations to preserve today's drift.

The bounded [frozen-contact load experiment](contact-load-integration.md) compares
a separate diagnostic adapter against these unchanged fixtures. It was built on
the legacy order. It removes the isolated incline displacement term there, but
the default speculative order now removes that term in production and the
experiment's stack and terminal-speed regressions and within-step stopping error
remain. The current paired measurements are in that page.

## Opt-in friction displacement correction (#91)

This option belongs to the legacy step order (`enableSpeculativeContacts =
false`); the default speculative order below removes the same term without it.

`SimulationConfig::enableFrictionDisplacementCorrection` (default `false`)
addresses the integration-order term above in production `World` steps. After
each island's velocity iterations, every solved tangential contact impulse is
treated as a force acting over the whole step, like the gravity and loads that
the integrator already applied as `x += v*dt + a*dt^2/2`. Each body therefore
moves by half of that impulse's velocity change times `dt`. Rotation uses the
complete contact wrench, so normal and friction torques stay balanced. The
normal impulse's linear part is deliberately excluded: normal settling keeps
the existing penetration-slop behavior, which preserves resting manifolds for
sleeping, wake-up and contact events. Equal and opposite pair terms keep the
center of mass on its momentum trajectory. Bodies in a swept CCD impact and
experimental integration strategies are skipped; free flight is unchanged.

Run `rigid_contact_benchmark --friction-displacement` for the corrected
matrix. Measured on `526570c` (2026-10-28), Windows Release Clang 23.1.4, full
matrix, selected rows. The flag only acts in the legacy order, so "off" and "on"
below are the legacy order (`enableSpeculativeContacts = false`) from the local
harness described at the top of this page; the last column gives the default
speculative order, in which all 432 rows are identical with the flag on or off.
In the legacy order, only the 54 incline rows differ between off and on; the
rest, stack and impact rows (378 of 432) are exactly identical.

| Fixture | dt | Iterations | Warm | Max displacement off | on | Downhill off | on | Terminal speed off | on | Default order max displacement |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| rest | 1/60 | 10 | .8 | .00534064 | .00534064 | 0 | 0 | 0 | 0 | 0 |
| incline_stick | 1/60 | 4 | 0 | .0550450 | .0136084 | .0547865 | .0125216 | .006295 | .006295 | .000265899 |
| incline_stick | 1/60 | 10 | .8 | .0428234 | .00533181 | .0424905 | .000173482 | 8.616e-05 | 8.616e-05 | 8.85772e-05 |
| incline_stick | 1/240 | 20 | 1 | .0117154 | .00502054 | .0105851 | 5.38473e-06 | 9.848e-05 | 9.848e-05 | 8.49878e-05 |
| incline_slide | 1/60 | 10 | .8 | 4.51423 | 4.50950 | 4.51423 | 4.50949 | 4.509 | 4.509 | 4.50949 |
| stack3 | 1/60 | 10 | .8 | .0223205 | .0223205 | 0 | 0 | .001951 | .001951 | .000818491 |
| stack6 | 1/60 | 10 | .8 | .0494876 | .0494876 | 0 | 0 | .03539 | .03539 | .00336027 |
| stack12 | 1/120 | 20 | 1 | .0902805 | .0902805 | 0 | 0 | 1.064e-08 | 1.064e-08 | .0124264 |
| stack12 | 1/60 | 4 | 0 | 1.11045 | 1.11045 | 0 | 0 | 2.840 | 2.840 | .0279522 |
| stack12 | 1/60 | 4 | .8 | .300322 | .300322 | 0 | 0 | .5682 | .5682 | .0147638 |

Across the full matrix in the legacy order, sticking-incline downhill
displacement improves in all 27 configurations (ratios 6.9e-5 to .23). The
remaining maximum displacement is up to .0136 in the unwarmed four-iteration rows
and about .005, normal settling into the 0.005 penetration slop, in the converged
rows. The analytic sliding distance
`0.5*g*(sin(theta)-mu_k*cos(theta))*T^2 = 4.5095` is matched in every
`incline_slide` row with the flag on (largest error 2.9e-5), where the flag-off
legacy order carried an O(dt) offset (largest error .0047). The default order
needs no flag for this: its largest error over the 27 slide rows is 3.0e-5.
The earlier mixed stack result, in which the flag moved converged stacks a few
percent either way and collapsed twelve-box stacks further, does not reproduce
on the current solver: the flag changes none of the 81 stack rows.
The archive [docs/data/friction-displacement-f47a4fc-clang23.json](data/friction-displacement-f47a4fc-clang23.json) holds the paired rows from that earlier solver and is not regenerated.

Unit tests check the analytic properties directly: incline creep in the second
second below 2% of the default's `g*sin(theta)*dt/2` per second at three
timesteps; second-second sliding kinematics within 1e-5 m of
`v*t + a*t^2/2`, with the default's offset equal to `mu_k*g*cos(theta)*dt/2`;
sticking just below and sliding just above the friction angle; an internal
friction pair whose mass center follows its momentum exactly; and resting
contacts that still sleep and wake. The earlier reason for leaving the flag off,
unconverged tall stacks getting worse, no longer shows in these measurements.
The flag stays `false` by default and remains a legacy-order option; the original
thresholds above are unchanged.

## Speculative contacts and substeps (default step order)

`SimulationConfig::enableSpeculativeContacts` (default `true`) detects contacts
at the start-of-step poses with speculative margins, solves velocities, then
integrates positions; see [contact solver](contact-solver.md#step-order). With
`solverSubsteps = 4` (the default), groups of touching bodies without joints are
solved as soft contacts in substeps, as described in
[substepped soft contacts](contact-solver.md#substepped-soft-contacts); with
`solverSubsteps = 1` the speculative order is one rigid pass. Measured on
`526570c` (2026-10-28), Windows Release Clang 23.1.4, full matrix, warm start as
listed. "Legacy" is `enableSpeculativeContacts = false`; "single pass" is the
speculative order with `solverSubsteps = 1`; "default" is four substeps. The
legacy and single-pass columns come from the local environment-variable harness
described at the top of this page.

Several single-pass rows reproduce the earlier printed "speculative" values digit
for digit (the incline_stick, stack3, stack6 and converged stack12 rows at warm 1),
while the unconverged four-iteration stack12 rows and the no-warm rows differ from
the earlier print. The legacy column also differs from the earlier printed
"legacy" values (for example the three-box row's maximum displacement is .02215
now, .02576 then). The legacy step order itself reproduces the earlier
integration-order measurements exactly (see above); the matrix differences are
consistent with the other solver changes listed at the top of this page, but they
were not bisected.

| Fixture | dt | Iterations | Warm | Peak penetration legacy | single pass | default | Max displacement legacy | single pass | default |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| rest | 1/60 | 10 | 1 | 0.005341 | 0 | 0 | 0.005341 | 0 | 0 |
| incline_stick | 1/60 | 4 | 1 | 0.005329 | 0.0001052 | 0.0001059 | 0.04276 | 8.101e-05 | 8.477e-05 |
| incline_stick | 1/60 | 10 | 1 | 0.005329 | 4.423e-06 | 0.0001057 | 0.04265 | 1.532e-05 | 8.388e-05 |
| incline_stick | 1/240 | 20 | 1 | 0.005021 | 1.611e-07 | 0.0001058 | 0.01172 | 4.837e-06 | 8.499e-05 |
| incline_slide | 1/60 | 10 | 1 | 0.005329 | 1.477e-07 | 8.605e-05 | 4.514 | 4.509 | 4.509 |
| incline_slide | 1/240 | 20 | 1 | 0.005021 | 3.725e-07 | 8.589e-05 | 4.511 | 4.509 | 4.509 |
| stack3 | 1/60 | 10 | 1 | 0.008747 | 0.0001116 | 0.0003456 | 0.02215 | 0.0001657 | 0.0007765 |
| stack6 | 1/60 | 10 | 1 | 0.01024 | 0.00111 | 0.0008781 | 0.04874 | 0.003348 | 0.003106 |
| stack12 | 1/60 | 10 | 1 | 0.011 | 0.003712 | 0.00249 | 0.09947 | 0.01891 | 0.01242 |
| stack12 | 1/60 | 4 | 1 | 0.01622 | 0.008768 | 0.002513 | 0.09885 | 0.04909 | 0.01243 |
| stack12 | 1/120 | 20 | 1 | 0.009489 | 0.0003765 | 0.002401 | 0.09028 | 0.002168 | 0.01243 |
| stack12 | 1/60 | 4 | 0.8 | 0.03666 | 0.03207 | 0.002823 | 0.3003 | 0.2794 | 0.01476 |
| stack12 | 1/60 | 4 | 0 | 0.1419 | 0.1394 | 0.005459 | 1.11 | 1.149 | 0.02795 |
| stack6 | 1/60 | 10 | 0 | 0.01042 | 0.01084 | 0.001212 | 0.05032 | 0.05431 | 0.004612 |

| Fixture | dt | Iterations | Warm | Downhill legacy | single pass | default | Terminal speed legacy | single pass | default |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| rest | 1/60 | 10 | 1 | 0 | 0 | 0 | 0 | 0 | 0 |
| incline_stick | 1/60 | 4 | 1 | 0.04243 | 3.725e-05 | 1.554e-05 | 8.908e-05 | 9.936e-05 | 9.252e-05 |
| incline_stick | 1/60 | 10 | 1 | 0.04231 | 1.508e-05 | 9.362e-06 | 9.852e-05 | 9.687e-05 | 9.883e-05 |
| incline_stick | 1/240 | 20 | 1 | 0.01059 | 4.837e-06 | 1.676e-05 | 9.848e-05 | 9.848e-05 | 9.089e-05 |
| incline_slide | 1/60 | 10 | 1 | 4.514 | 4.509 | 4.509 | 4.509 | 4.509 | 4.509 |
| incline_slide | 1/240 | 20 | 1 | 4.511 | 4.509 | 4.509 | 4.509 | 4.509 | 4.509 |
| stack3 | 1/60 | 10 | 1 | 0 | 0 | 0 | 5.551e-17 | 5.551e-17 | 8.906e-06 |
| stack6 | 1/60 | 10 | 1 | 0 | 0 | 0 | 1.96e-09 | 2.043e-10 | 3.524e-05 |
| stack12 | 1/60 | 10 | 1 | 0 | 0 | 0 | 0.0005323 | 0.0001329 | 8.36e-05 |
| stack12 | 1/60 | 4 | 1 | 0 | 0 | 0 | 0.05292 | 0.03592 | 6.21e-05 |
| stack12 | 1/120 | 20 | 1 | 0 | 0 | 0 | 1.064e-08 | 1.144e-08 | 0.000116 |
| stack12 | 1/60 | 4 | 0.8 | 0 | 0 | 0 | 0.5682 | 0.609 | 0.005198 |
| stack12 | 1/60 | 4 | 0 | 0 | 0 | 0 | 2.84 | 3.044 | 0.03522 |
| stack6 | 1/60 | 10 | 0 | 0 | 0 | 0 | 0.177 | 0.2333 | 0.01666 |

What improved with the default:

* All impact rows (head-on and off-center, every dt, iteration and warm-start
  setting) have the same analytic impulse errors under all three configurations:
  maximum velocity error 1.11e-7.
* Rest: peak penetration and displacement are zero (legacy up to .005341).
* Sliding: every `incline_slide` row matches the analytic 4.5095 m, with a
  largest error of 3.0e-5 for the default and 2.5e-5 for the single pass (legacy
  .0047).
* Sticking incline: downhill creep is at most 2.2e-5 m at warm 1, 1.3e-4 at warm
  .8 and 2.5e-4 at warm 0 (single pass: 4.2e-5, 1.3e-3 and 7.0e-3; legacy .0106
  to .0424, .0107 to .0449 and .0107 to .0548).
* Stacks: maximum displacement and peak penetration are lower than in the legacy
  order in all 27 rows of each of the 3-, 6- and 12-box fixtures. The unwarmed
  four-iteration twelve-box stack, which the legacy order (1.110 m) and the
  single pass (1.149 m) let slide, moves .0280 m with the default.

What got worse, or is not better:

* Terminal peak speed of the stacks is higher than in the legacy order in 12 of
  27 three-box rows, 9 of 27 six-box rows and 6 of 27 twelve-box rows, mostly at
  warm 1, where the legacy values were between 1e-17 and 9e-7 (for example three boxes,
  dt=1/60, 10 iterations, warm 1: 5.6e-17 legacy, 8.9e-6 default). The largest
  of the affected default values is 1.0e-3 (three boxes, dt=1/60, 20 iterations,
  no warming); at warm 1 none exceeds 4.1e-4.
* Against the single pass, the default has a larger maximum displacement in rows
  at warm 1 with enough iterations to converge (stack12 dt=1/120, 20 iterations:
  .002168 single pass against .01243), and a nonzero resting penetration of
  1e-4 on the incline (the single pass reaches 1.6e-7 to 4.4e-6 in converged
  rows). The soft model rests with a small compression, so it does not reach the
  vanishing residuals a converged rigid pass can.
* The single pass is not better than the legacy order everywhere: the unwarmed
  four-iteration twelve-box stack moves 1.149 m in the single pass against 1.110
  m in the legacy order, and the unwarmed six-box row at 10 iterations is
  slightly worse (.05431 against .05032).

# Rigid-contact physical diagnostics

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
contact settings retain the default configuration. The restitution velocity
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

## Measured baseline before the contact arithmetic refactor

The table was measured on Windows, Release Clang 23.1.1, against solver revision
`a862d366` (before issue #75's internal arithmetic changes), over two seconds.
The benchmark schema/fixtures are those described above. Length=1 is a box width.
These observations are evidence of accuracy limits, not universal stability claims.

| Fixture | dt | Iterations | Warm | Peak penetration | Maximum displacement | Final maximum speed |
|---|---:|---:|---:|---:|---:|---:|
| Rest | 1/60 | 4 | 0 | .005768 | .006311 | .001550 |
| Rest | 1/120 | 10 | .8 | .005191 | .005151 | .0000793 |
| Stack 6 | 1/60 | 4 | 0 | .063713 | 1.245570 | 2.641058 |
| Stack 6 | 1/120 | 10 | .8 | .011733 | .082295 | .083027 |
| Stack 12 | 1/60 | 4 | 0 | .311358 | 5.763308 | 7.514555 |
| Stack 12 | 1/240 | 20 | .8 | .009245 | .201263 | .150270 |

The isolated analytic impacts' maximum velocity error was 1.47e-7 across the
quick matrix, and all energy/momentum/angular regression assertions passed.
The low-friction incline at dt=1/60, four iterations and no warming reached
downhill speed 4.509478 versus ideal 4.509486; its displacement was 4.514229
versus ideal 4.509487. The high-friction incline under the same controls crept
.049150 downhill despite its zero-motion equilibrium oracle. Its residual speed
was only .003455, so final speed alone would conceal that drift.

The single-body rest fixture remains near the configured penetration slop.
The six- and twelve-box stacks can move substantially or collapse at low iteration
count without warming. Refinement and warming improve the listed cases, but do
not make the taller stack an exact equilibrium: the finest listed 12-box case
still has .201 displacement and .150 speed. The poor six-box case retained all
six contact entries and had 123 feature changes; retained contacts alone did not
prevent drift. Re-run the same matrix after production changes and compare
physical metrics and work rather than silently widening thresholds or removing
the difficult fixtures.

## Polygon geometry comparison with the current contact arithmetic

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

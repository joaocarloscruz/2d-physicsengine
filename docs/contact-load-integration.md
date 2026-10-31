# Frozen-contact load integration experiment (#91)

> **Measured on `526570c` (2026-10-28), Windows x86_64, Release, Clang 23.1.4.**
> "Production" means the library as it is on that commit: speculative step order,
> `solverSubsteps = 4`, block two-point correction, stable polygon feature ids,
> warm start 1.0 where a matrix sets it. The experiment was designed on the
> legacy step order and its adapter still forces it
> (`enableSpeculativeContacts = false`), so a like-for-like baseline is the
> production solver in the legacy order. Both are given below. The legacy-order
> numbers come from the unmodified diagnostic sources compiled with a locally
> force-included copy of `simulation_config.h` whose speculative/substep defaults
> read environment variables (a measurement harness that is not in the
> repository; see the [rigid-contact benchmark](rigid-contact-benchmark.md)
> note). Its production-legacy values for the isolated observations match the
> earlier published ones digit for digit.

The standalone `contact_load_diagnostic` and `contact_load_benchmark` test a
bounded alternative to displacement applied before contact solving. **This is
not a production fix.** On the legacy order it removes the isolated equilibrium
integration term but worsens several stack and friction metrics. The default
speculative order has since removed that term in production, so the question the
experiment asked has largely changed; the measurements below say how. Production
`World`, integration, contact solving, public configuration and installed headers
are unchanged by this experiment.

Validation, measured on `526570c`: Release CTest passes 39/39 (in the full-suite
run, nine of the last tests failed to start with Windows exit code 0xc0000142, a
DLL initialization error, and passed when run again on their own); the native
suite has 813 cases and all pass (12,982,553 assertions), and
`run_tests "[!mayfail]"` matches no cases, so the nine expected #44 failures
recorded when this experiment was written are no longer expected failures. The
`[contact-load]` controls pass 758 assertions in ten cases. The earlier
ASan+UBSan run (13 cases, 1,056 assertions and the 120-row experimental quick
matrix) was not repeated.

## Derivation and scope

Write generalized velocity as `u=(v_x,v_y,omega)` and the constant generalized
load acceleration as `a=(F_x/m,F_y/m,tau/I)`. For a body initially at rest, the
current unconstrained integrator advances by `.5*a*dt^2` before contacts can
cancel `a*dt`. Thus a perfectly solved post-step velocity can coexist with an
O(dt) accumulated displacement error on a loaded static incline. The existing
[integration-order control](rigid-contact-benchmark.md#integration-order-control-for-incline-creep)
measures this without assuming a friction solver residual explains the drift.

At the current pose, let `P(u)` be the actual finite-iteration contact velocity
solve on eligible start-of-step contacts, with no loads, position correction,
warming, sleeping, drives, or velocity caps. The experiment computes

```
unloaded = P(u)
loaded   = P(u + a*dt)
deltaLoadReactionVelocity = loaded - unloaded - a*dt
deltaPose = .5*dt*deltaLoadReactionVelocity
```

It stages this pose offset **before** the ordinary production `World::step(dt)`.
That step still applies and consumes the real load once and produces the final
velocities. Scratch worlds hold copies, have no force registrations, and call
`step(0)`; positional correction is explicitly disabled. The scratch clones
evaluate only the exact, stateless built-in `Gravity` type when determining the
load kick. Opaque generators and Gravity subclasses bypass the experiment,
because evaluating a stateful generator again could change its semantics.
Manually accumulated force and torque are copied, never consumed on the original
body by a scratch solve. The new tests count actual generator calls in the
bypass path and verify one-shot force/torque consumption in free flight.

The difference between the loaded and unloaded projections excludes a common
initial-velocity collision impulse. It is not `.5*dt` times the entire collision
velocity change. No starting manifold means no correction; an initially closing
new impact is excluded. First-step closing roundoff is bounded by
`64*float_epsilon*dt*(|a_A|+|a_B|)`, with no physical unit floor. Subsequent eligible
pairs must have collided after the preceding real step, and still have a real
start-of-step manifold. There is no invented overlap, contact padding, or relaxed
narrow-phase threshold. Both materials must have zero restitution.

On a fixed geometry with a fixed linear active set, the subtraction isolates the
constrained acceleration response. If sticking cancels the whole load, it cancels
the unwanted `.5*a*dt^2`. If a kinetic sliding force remains constant through the
step, it integrates that deceleration in the position as well as velocity. These
statements do not extend automatically across active-set changes, finite
iteration errors, manifold changes or an impact occurring partway through a step.

The diagnostic adapter bypasses the entire world for joints, CCD, sleeping,
velocity caps, moving static supports, opaque generators, more than 32 bodies,
or more than 64 velocity iterations. This bounds scratch allocation and pair
work before cloning. Dynamic contact graphs are permitted only for measurement,
not as a conservation or stability guarantee. Scratch body IDs follow creation
and add order; the committed fixtures create and add bodies in the same order as
the original world. Arbitrary original-ID/add-order permutations can change
manifold canonicalization and iterative friction order. Bit-identical behavior
for such reordered worlds is not claimed.

Corrected poses are checked for finite float representability and staged together.
The ordinary real step retains its existing exception/partial-step behavior;
the adapter does not provide a whole-world rollback guarantee. Direct diagnostic
pose writes avoid treating the offset as a public teleport/wake request. This
does not establish production interpolation, previous-pose, wake, mutation,
event-order or force-generator contracts. Contact callback counts are measured,
not promised to match after corrected geometry changes.

## Reproduction and independent controls

The experiment was first measured on base `e0fd45d` with source operator
`6e576f6` (LLVM-MinGW Clang 23.1.1); every number in this page has been
re-measured on `526570c`. Both matrix executables use the same engine binary and
original #78 fixtures, materials, geometry, timestep, iterations, warming,
position correction, reporting and oracle calculations. Only the diagnostic
world type differs. The default benchmark output remains
unchanged. The experimental output labels its operator and reports per-row
eligibility counts, scratch solve counts and achieved normal/tangential speeds.

```
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel 2
build/contact_load_diagnostic > build/load-observations.json
build/rigid_contact_benchmark --quick --output build/contact-quick-baseline.json
build/contact_load_benchmark --quick --output build/contact-quick-experiment.json
build/rigid_contact_benchmark --output build/contact-full-baseline.json
build/contact_load_benchmark --output build/contact-full-experiment.json
python benchmarks/compare_contact_load.py --directory build \
  --output docs/data/contact-load-e0fd45d-clang23.json
ctest --test-dir build --output-on-failure
build/run_tests "[contact-load]"
```

Append `.exe` where required on Windows. The committed archive
`docs/data/contact-load-e0fd45d-clang23.json` holds the earlier solver's paired
rows and has not been regenerated. The archive script uses Python's standard
library, parses all five source artifacts and the resulting archive, verifies
paired controls/counts and removes only nondeterministic timing. It imposes a
16 MiB input report limit. CTest's diagnostic smoke additionally parses JSON on
CMake 3.19+; the project's older 3.16 minimum retains execution-only validation.
Execution success is not physical acceptance.

Ten native control cases check persistent support and a rotated incline,
constant kinetic-friction deceleration, outward-load detachment, ballistic
one-shot forces/torques, new closing impacts, isolated impact momentum/angular
momentum/energy, and exact production delegation for CCD, joint motors, sleeping,
moving supports, caps, opaque generators and budget exclusion. Existing analytic
impact margins remain `3e-6`; their mass-ratio-1000 float storage error is about
`3.65e-7` in momentum. Experimental instantaneous impact final states are also
required to equal the production final states exactly. The CCD control uses an
actual swept circle/polygon impact, not an unsupported polygon/polygon pair.

## Isolated load observations

The 24 support rows run 120/240/480 steps over the stored-float duration
`2.0000001043081284`, with 64 iterations, velocity tolerance zero, warming and
position correction zero, no sleeping/caps/CCD. Unit boxes and a 20x1 floor use
mass 1, restitution zero. Rest/stick friction is 1; sliding friction is .03 with
initial downhill speed 1. The incline is stored `pi/12`; its positions are exactly
the original nominal `normal*.5` and `normal*-.5`, without extra overlap.

| Steps | Legacy incline drift | Default incline drift | Experimental incline drift | Legacy flat sinking | Default flat sinking | Experimental flat sinking |
|---:|---:|---:|---:|---:|---:|---:|
| 120 | .0423169950 | .0000201378 | 0 | .1635003090 | 0 | .0013625026 |
| 240 | .0211575719 | .0000202807 | 0 | .0817465782 | 0 | .0003406107 |
| 480 | .0105778603 | .0000204128 | 0 | .0408697128 | 0 | .0000851452 |

The default's incline drift is about 2.0e-5 and does not shrink with the step;
its flat fixture does not sink at all, since the speculative order sees the
exactly touching boxes that the legacy order only detects after integrating into
them. The experiment's flat result is therefore now worse than production's.

The rotated float fixture has a starting manifold; flat exact touching does not.
The flat experiment therefore misses exactly one first-step correction and
preserves its resulting penetration. Its later zero motion does not mean exact
flat equilibrium from nominal touching. This residual scales as the one-step
`.5*g*dt^2`, rather than silently adding positive overlap to activate a solve.

Incline experimental loaded closing residual peaks at `2.59e-16`, `1.30e-16`,
and `6.48e-17`. Its two-second actual body contact impulse in y matches the real
load magnitude `19.6200018625` to accumulated double summation roundoff. The
scratch incremental y reaction agrees as well; small residual x impulses are
around `6e-15`. These are body-state-inferred reactions, not an implemented force
feedback on the static floor. Torque entries are spin angular impulses about the
body center; they are not complete wall torque about a world origin. Loaded
tangential speed is also reported, so a small closing residual alone is not
misidentified as sticking.

Kinetic-slide displacement is `6.50960555 / 6.50952533 / 6.50953990` versus the
constant-plane oracle `6.50948664`; the legacy order gives
`6.51422934 / 6.51185847 / 6.51070323` and the default
`6.50950020 / 6.50951255 / 6.50948543`. The experiment's remaining error is not
monotone at the finest spacing, and final float velocity errors remain; the
default is within 2.6e-5 of the oracle at every spacing. A separate dyadic flat
test verifies `mu=.25`, `g=8`, initial speed 2, time .5: displacement .75 and
speed 1 while sliding persists throughout each step.

Changing the load to outward `normal*9.81` after one second detaches the incline.
Experimental normal displacement after the following second is
`4.90500184 / 4.90499299 / 4.90499643`, versus the ballistic oracle 4.905.
Experimental event counts are `(begin,persist,end)=(1,59,1)/(1,119,1)/(1,239,1)`.
The legacy order retains the previously accumulated penetration and later
detachment, with normal displacements `4.82603748 / 4.86551266 / 4.88525742` and
event counts `(1,66,1)/(1,129,1)/(1,254,1)`. The default gives
`4.90491871 / 4.90490993 / 4.90491439` (event counts
`(1,59,1)/(1,120,1)/(1,242,1)`), within 9.1e-5 of the ballistic oracle and
slightly further from it than the experiment (within 1.0e-5). These changed
contact lifetimes are observations, not a general event compatibility claim.

## Within-step stopping remains wrong

Eight additional diagnostic rows use an already overlapping flat fixture
(center y=.499), mass 1, initial speed `.2f`, `g=8`, friction .25, duration .5,
and the same 64-iteration/tolerance-zero settings. The independent Coulomb
stopping time is `.10000000149`, and distance `.010000000298`.

| dt | Legacy distance | Default distance | Default error | Experimental distance | Experimental error |
|---:|---:|---:|---:|---:|---:|
| .125 | .0250000004 | .0099479165 | -.0000520838 | .0125000002 | .0024999999 |
| .0625 | .0171875004 | .0098925736 | -.0001074267 | .0109375007 | .0009375004 |
| .03125 | .0132812504 | .0098129930 | -.0001870073 | .0101562506 | .0001562503 |
| .015625 | .0116210934 | .0097648455 | -.0002351548 | .0100585930 | .0000585927 |

The experiment averages endpoint velocity changes through the whole step. A stop
inside the step instead needs its actual stopping time: displacement is
`v0*t_stop-.5*mu*g*t_stop^2`, with zero motion afterward. For dt=.125 the
experimental trapezoid gives `.5*v0*dt=.0125`; the exact result is .01. The
legacy order overshoots by up to .015. The default stops short instead, by 5.2e-5
at dt=.125 growing to 2.4e-4 (2.4% of the distance) at dt=.015625: it is closer
than the experiment at the coarsest spacing and farther at the finest, and its
error grows as the step shrinks. None of the three is exact. This active-set
transition remains a concrete blocker for any of them, despite near-zero final
velocities. The report retains the nonzero error; no acceptance threshold is
invented around it. Similar contact onset, stick-slip, release, and
rotating-manifold transitions need event timing or another justified integration
treatment.

## Unchanged quick/full matrices reject promotion

The quick matrix has 120 rows/7,280 ordinary World steps; full has 432/45,630.
Both methods have zero execution failures. The prototype adds 14,344 and 90,474
scratch solves respectively, beyond the original reported World work. The
[complete deterministic archive](data/contact-load-e0fd45d-clang23.json) keeps
all paired physical metrics, states, events and work from the earlier solver,
including worse rows; it was not regenerated for this refresh, whose raw rows
were kept outside the repository.

These coarse rows use dt=1/60, four iterations and warm factor zero over two
seconds, with original benchmark friction .8/.6 and positional correction .8.
The experiment runs on the legacy order, so the legacy-order production column is
its matched baseline, and the default column shows what production does today:

| Fixture / metric | Production, legacy order | Production, default | Experiment |
|---|---:|---:|---:|
| Rest displacement | .0053406358 | 0 | .0013625026 |
| Incline-stick downhill displacement | .0547864681 | -.0002524849 | .0101115199 |
| Incline-stick terminal peak speed | .0062947884 | .0017478991 | .0564005836 |
| Incline-stick feature changes | 0 | 0 | 7 |
| Incline-slide displacement | 4.5142315350 | 4.5094950110 | 4.5094901190 |
| Stack3 displacement | .0239973068 | .0014762878 | .0237431526 |
| Stack3 terminal peak speed | .0756771565 | .0134594217 | .0756771565 |
| Stack6 displacement | .0851683617 | .0070939064 | .0832591057 |
| Stack6 terminal peak speed | .5530557632 | .0205473900 | .5530557632 |
| Stack12 displacement | 1.1104459760 | .0279521942 | 1.1081752777 |
| Stack12 terminal peak speed | 2.8400735860 | .0352185369 | 2.8400721550 |

Against its matched legacy baseline the experiment now helps less than it did:
the stack rows are nearly unchanged, and the incline-stick row still trades a
smaller downhill displacement for a nine-fold larger terminal peak speed and 7
feature changes. Against the default, the experiment is worse on every row
except the incline-slide displacement, where the two agree to 5e-6.

At the coarse incline the scratch loaded closing residual is `.00633145`, not
near zero; tangential speed is `.00006272`. Stack6 peaks at `.174148` closing
residual and Stack12 at `.419318`, both with zero tangential speed. Finite
iterative `P` cannot be treated as an exact projection in these cases. All 120
incline steps have eligible starting contacts, so its regression is not simply
a lack of eligibility. Stack6 and Stack12 have one initial no-contact step.

In the full matrix, the experiment's displacement exceeds the legacy-order
production's by more than `1e-8` in 5/27 Stack6 rows and 13/27 Stack12 rows (0/27
Stack3); this reporting cutoff is solely for counting numeric deltas, not an
accuracy tolerance. Against the current default it is larger in 25/27 Stack3 rows
and in all 27 rows of each of Stack6 and Stack12. Examples:

* Stack6, dt=1/60, iterations 4, warm=1, legacy order: .0485506058 →
  .0499539375 (default: .0031065941).
* Stack12, dt=1/60, iterations 20, warm=1, legacy order: .0994920731 →
  .1045265198 (default: .0124235153).
* Stack12, dt=1/120, iterations 20, warm=.8, legacy order: .1015014648 →
  .1050214767 (default: .0124683380).
* Stack12, dt=1/60, iterations 4, warm=0: legacy order 1.1104459760, experiment
  1.1081752777, default .0279521942.

The existing isolated impact rows remain exactly unchanged against both the
legacy order and the default (all 270 full-matrix impact rows identical).
Improvements in position do not establish friction equilibrium, lower energy, or
stability. Position offsets can also change orbital angular momentum and
potential energy without a corresponding velocity impulse; general dynamic-contact
conservation has not been proved. No difficult row or threshold was removed.

## Next design decision

The speculative order (solve velocities, then integrate) has since removed the
isolated incline term and the flat-rest sinking in production, and its stack
metrics are better than both the legacy order and this experiment, so this
experiment is not a candidate for promotion. What it did not resolve is still
open: within-step stopping (the default's .00005 to .00024 undershoot above), onset,
stick-slip and release transitions, and the default's constant 2e-5 incline
residual. The recommendation recorded with the experiment was: keep the
production pipeline until a constrained load/integration design provides a
consistent position trajectory and velocity impulse budget, and then try a load/contact solve at the beginning of a step
with explicitly tracked persistent-contact active sets, separating sustained
reaction from impulsive impact response. It should integrate constant sliding
reaction only until independently computed onset/stop/release times, then advance
the remaining free/constrained interval. Reuse each real force evaluation once,
and derive how joint motors and CCD consume that same interval and load budget.

Acceptance requires all present rest/slide/impact/load controls, strict-touch
onset, within-step stopping distance, changing-load release, finite-iteration
residual reporting, dynamic-pair angular momentum/energy, moving supports and
joint/CCD/sleep/event contracts. Re-run both original matrices on identical
geometry, retain every regression, and keep the existing thresholds. This
prototype supplies evidence and reproducible controls, not an unvalidated
configuration switch or a general solution to #91.

# Frozen-contact load integration experiment (#91)

The standalone `contact_load_diagnostic` and `contact_load_benchmark` test a
bounded alternative to displacement applied before contact solving. **This is
not a production fix:** it removes the isolated equilibrium integration term,
but worsens several stack and friction metrics. Production `World`, integration,
contact solving, public configuration and installed headers are unchanged. All
nine expected #44 failures and their original thresholds remain.

Validation: Release CTest passes 21/21; the native suite has 572 cases, 563 passed
and nine failed as expected. The new controls pass 758 assertions in ten cases.
ASan+UBSan passes the 13 contact-load/oracle/diagnostic cases (1,056 assertions),
the JSON-validated observation tool, and the 120-row experimental quick matrix.

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

Measured on base `e0fd45d`, source operator `6e576f6`, Windows x86_64, LLVM-MinGW
Clang 23.1.1, Release, with maximum two build workers. Both matrix executables use
the same engine binary and original #78 fixtures, materials, geometry, timestep,
iterations, warming, position correction, reporting and oracle calculations.
Only the diagnostic world type differs. The default benchmark output remains
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

Append `.exe` where required on Windows. The archive script uses Python's standard
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

| Steps | Production incline drift | Experimental incline drift | Production flat sinking | Experimental flat sinking |
|---:|---:|---:|---:|---:|
| 120 | .0423169950 | 0 | .1635003090 | .0013625026 |
| 240 | .0211575719 | 0 | .0817465782 | .0003406107 |
| 480 | .0105778603 | 0 | .0408697128 | .0000851452 |

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
constant-plane oracle `6.50948664`; production gives
`6.51422934 / 6.51185847 / 6.51070323`. The remaining error is not monotone at the
finest spacing, and final float velocity errors remain. A separate dyadic flat
test verifies `mu=.25`, `g=8`, initial speed 2, time .5: displacement .75 and
speed 1 while sliding persists throughout each step.

Changing the load to outward `normal*9.81` after one second detaches the incline.
Experimental normal displacement after the following second is
`4.90500184 / 4.90499299 / 4.90499643`, versus the ballistic oracle 4.905.
Experimental event counts are `(begin,persist,end)=(1,59,1)/(1,119,1)/(1,239,1)`.
Production retains the previously accumulated penetration and later detachment,
with normal displacements `4.82603748 / 4.86551266 / 4.88525742`. These changed
contact lifetimes are observations, not a general event compatibility claim.

## Within-step stopping remains wrong

Eight additional diagnostic rows use an already overlapping flat fixture
(center y=.499), mass 1, initial speed `.2f`, `g=8`, friction .25, duration .5,
and the same 64-iteration/tolerance-zero settings. The independent Coulomb
stopping time is `.10000000149`, and distance `.010000000298`.

| dt | Production distance | Experimental distance | Experimental error |
|---:|---:|---:|---:|
| .125 | .0250000004 | .0125000002 | .0024999999 |
| .0625 | .0171875004 | .0109375007 | .0009375004 |
| .03125 | .0132812504 | .0101562506 | .0001562503 |
| .015625 | .0116210934 | .0100585930 | .0000585927 |

The experiment averages endpoint velocity changes through the whole step. A stop
inside the step instead needs its actual stopping time: displacement is
`v0*t_stop-.5*mu*g*t_stop^2`, with zero motion afterward. For dt=.125 the
experimental trapezoid gives `.5*v0*dt=.0125`; the exact result is .01. This
active-set transition is a concrete blocker, despite near-zero final velocities.
The report retains the nonzero error; no acceptance threshold is invented around
it. Similar contact onset, stick-slip, release, and rotating-manifold transitions
need event timing or another justified integration treatment.

## Unchanged quick/full matrices reject promotion

The quick matrix has 120 rows/7,280 ordinary World steps; full has 432/45,630.
Both methods have zero execution failures. The prototype adds 14,344 and 90,474
scratch solves respectively, beyond the original reported World work. The
[complete deterministic archive](data/contact-load-e0fd45d-clang23.json) keeps
all paired physical metrics, states, events and work, including worse rows.

These coarse rows use dt=1/60, four iterations and warm factor zero over two
seconds, with original benchmark friction .8/.6 and positional correction .8:

| Fixture / metric | Production | Experiment |
|---|---:|---:|
| Rest displacement | .0055941343 | .0013625026 |
| Incline-stick downhill displacement | .0548022371 | .0100595754 |
| Incline-stick terminal peak speed | .0063030018 | .0564011452 |
| Incline-stick feature changes | 0 | 7 |
| Incline-slide displacement | 4.5142249051 | 4.5094901191 |
| Stack3 displacement | .0258825851 | .0258109536 |
| Stack3 terminal peak speed | .0756994974 | .1012616485 |
| Stack6 displacement | .1321672816 | .1278428278 |
| Stack6 terminal peak speed | .6115648269 | .6120268693 |
| Stack12 displacement | 4.3645658605 | 4.7574037112 |
| Stack12 terminal peak speed | 5.8009528413 | 6.7807063514 |

At the coarse incline the scratch loaded closing residual is `.00633145`, not
near zero; tangential speed is `.00006298`. Stack6 peaks at `.184315` closing
residual, and Stack12 at `1.91014`, with tangential speed `8.73197`. Finite
iterative `P` cannot be treated as an exact projection in these cases. All 120
incline steps have eligible starting contacts, so its regression is not simply
a lack of eligibility. Stack6 and Stack12 have one initial no-contact step.

In the full matrix, displacement increases by more than `1e-8` in 8/27 Stack6
rows and 11/27 Stack12 rows; this reporting cutoff is solely for counting numeric
deltas, not an accuracy tolerance. Examples retained in the archive:

* Stack6, dt=1/120, iterations 4, warm=.8: .0502608711 → .0514378587.
* Stack12, dt=1/60, iterations 4, warm=.8: .4918984265 → .5402204306.
* Stack12, dt=1/60, iterations 20, warm=1: .0991093696 → .1025268195.
* Stack12, dt=1/120, iterations 20, warm=.8: .1009617366 → .1041750514.

The existing isolated impact rows remain exactly unchanged. Improvements in
position do not establish friction equilibrium, lower energy, or stability.
Position offsets can also change orbital angular momentum and potential energy
without a corresponding velocity impulse; general dynamic-contact conservation
has not been proved. No difficult row, threshold, or #44 failure was removed.

## Next design decision

Keep the current production pipeline until a constrained load/integration design
provides a consistent position trajectory and velocity impulse budget. The
recommended next experiment is a load/contact solve at the beginning of a step
with explicitly tracked persistent-contact active sets, separating sustained
reaction from impulsive impact response. It should integrate constant sliding
reaction only until independently computed onset/stop/release times, then advance
the remaining free/constrained interval. Reuse each real force evaluation once,
and derive how joint motors and CCD consume that same interval and load budget.

Acceptance requires all present rest/slide/impact/load controls, strict-touch
onset, within-step stopping distance, changing-load release, finite-iteration
residual reporting, dynamic-pair angular momentum/energy, moving supports and
joint/CCD/sleep/event contracts. Re-run both original matrices on identical
geometry, retain every regression, and preserve the nine #44 thresholds. This
prototype supplies evidence and reproducible controls, not an unvalidated
configuration switch or a general solution to #91.

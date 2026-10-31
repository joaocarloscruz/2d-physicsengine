# Constraint-based contact solver

Each `World::step` prepares a contact constraint for every manifold point.
Preparation converts contact positions to body-local anchors and precomputes
the normal and tangent effective masses. Stable feature IDs reconnect each
point to its cached normal and friction impulses from the previous step.

## Step order

By default (`SimulationConfig::enableSpeculativeContacts = true`) a step runs
in the Box2D order with speculative contacts:

1. Apply loads and integrate velocities. Positions stay at the start of the
   step.
2. Detect contacts at the start-of-step poses. Each body gets a margin of
   `2 * penetrationSlop` plus the distance it can cover this step
   (`dt * (|v| + |w| * reach) + dt^2 * |a|`); a pair is reported when its
   gap is below the sum of both margins. Speculative points carry negative
   penetration (minus the gap). Separated polygons whose closest features are
   two vertices get one point along the true gap.
3. Warm-start joints and contacts, then iterate their velocity constraints
   together (joints first in each pass). A speculative point with gap `s`
   enforces `v_n >= -s/dt`: the gap may close this step, no further.
   Overlapping points keep the restitution bias and Coulomb friction. An awake
   body predicted to close its gap to a sleeping body this step wakes that
   body's island first, so the sleeper takes part in this solve.
4. Integrate positions. A body whose solver velocity change is no larger than
   its applied load per step (support, friction) moves by half of that change,
   the trapezoid rule that matches the constant-load integrator, so resting
   and sliding bodies stay on their analytic paths without friction creep. A
   larger change is an impact and moves the body by all of it, which is what
   `v_n >= -s/dt` assumed; between one and two times the load the two blend.
5. Apply restitution to speculative points whose gap closed this step and
   that carried a normal impulse, so a bounce leaves from the surface (as in
   Box2D v3). The bounce targets `e` times the approach speed at the moment of
   contact, `sqrt(v0^2 - 2 a s)` from the start-of-step relative velocity and
   load, not the end-of-step prediction, which includes the load for the part
   of the step after contact and made `e = 1` gain energy.
6. Run CCD sweeps.
7. Detect contacts again at the integrated poses and iterate position
   constraints on those manifolds. Contact events, statistics and the
   persistent contact count also come from this end-of-step detection, with
   pairs within `0.25 * penetrationSlop` counted as touching.

Steps 3 and 4 run as described for groups with joints and for groups that
take the rigid solve. Other groups of contacts take substeps instead; see
[substepped soft contacts](#substepped-soft-contacts).

Islands for sleeping and waking only follow touching contacts. A speculative
pair joins awake islands for the solve. A speculative pair with a sleeping
body is skipped unless the awake body will close the gap this step, in which
case the sleeper wakes (step 3); a body merely passing close by never wakes it. Pairs that CCD sweeps (any pair with a CCD-enabled body)
get only the fixed margin and keep their exact time-of-impact handling.

With `enableSpeculativeContacts = false` the legacy order runs: integrate
positions, optional CCD, detect at the new poses, warm-start, velocity
iterations, position iterations. A fast body there sinks up to `v * dt` before
its contact is seen, and integrating the load before the friction impulse
creeps on inclines. Experimental integration strategies that publish an
end-of-step plan use this order for that step.

In either order the solver phases are:

1. Warm-start persistent contacts with their scaled cached impulses.
2. Iterate velocity constraints to enforce non-penetration, restitution, and
   Coulomb friction.
3. Iterate position constraints independently to repair remaining overlap.

`SimulationConfig::solverIterations` controls both iterative phases. Increasing
it improves convergence through stacks and chains of touching bodies, at the
cost of additional constraint work. Broad-phase and narrow-phase detection do
not repeat for each solver iteration.

Low-speed contacts use `restitutionVelocityThreshold` to suppress bounce.
`velocityTolerance` treats tiny tangential motion as settled,
`penetrationSlop` permits a small positional tolerance, and
`maxPositionCorrection` limits the repair performed by a single position
iteration. These values are validated by `SimulationConfig` and are available
through the WebAssembly configuration object.

## Substepped soft contacts

With `solverSubsteps > 1` (default 4) the speculative order solves each group
of contacts without joints in substeps of `h = dt / solverSubsteps`, after
Box2D v3's soft step. Each substep:

1. Adds the applied load over `h` to the velocities (with the velocity
   limits).
2. Warm-starts the contacts with their accumulated impulses, at the levers
   prepared at the start of the step.
3. Runs `solverIterations / solverSubsteps` (at least one) iterations. Each
   row measures its separation `s` at the current poses against the prepared
   normal. A gap (`s > 0`) is speculative: `v_n >= -s/h`. An overlap is a soft
   spring of `contactHertz` (capped at a quarter of the substep rate) and
   `contactDampingRatio`, solved implicitly: target speed `biasRate * s`
   (at most `contactPushSpeed`), with the impulse scaled by `massScale` and
   relaxed by `impulseScale` times the accumulated impulse. A two-point
   manifold solves both normal rows as one 2x2 LCP with each soft row's
   compliance on the diagonal. Coulomb friction follows.
4. Integrates positions with the same trapezoid rule as the single pass.
5. Relaxes: one more iteration with rigid rows and no push-out, so that the
   push-out moves positions without leaving velocity behind. The last relax
   leaves open gaps alone, so a body is not slowed before the contact (and its
   bounce) that the next step will see.

Restitution, CCD, the end-of-step detection and position correction then run
as in the single pass. Bounces of overlapping points are applied after the
substeps too, and a point counts as carried if any substep carried it.

### Why towers needed it

The single pass solves rigid rows with 10 Gauss-Seidel iterations and repairs
overlap in a separate position pass that only acts beyond the slop. That left
two gaps in a tall stack: the iterations do not converge through 20 contacts
in series (raising them to 100 still left a 20-box tower swaying by 9 cm), and
within the slop nothing restores the relative rotation of two boxes, so the
tower could bend inside that dead band while the position pass acted on the
deeper corner like an undamped spring. A misaligned 20-box tower fell after
about 12 s; with 40 iterations it swayed instead, with a 6 s period and 12 cm
at the top. Stepping at 120 Hz or 240 Hz still left about 2.6 cm of drift.

A soft row acts on any overlap, so it restores rotation, it damps, and its
compliance regularizes the iterations. Three choices mattered, measured on the
misaligned 20-box tower at 60 Hz: no slop dead band in the soft target (with
the slop as dead band the tower swayed by 50 cm), 60 Hz contacts (at Box2D's
30 Hz the tower buckled), and a damping ratio of 2 (Box2D's 10 left a 12 s
sway of 2.6 cm; at 2 the drift is 0.9 cm). Static contacts use the same
stiffness: Box2D's doubled static stiffness made a 50 kg box rock on a 1 kg
box. Solving the two points of a manifold one after the other, as Box2D does,
tilted an aligned tower by 1.7 cm.

### Which groups stay rigid

The soft model carries a load `lambda` per substep at an overlap of
`lambda * impulseScale / (normalMass * massScale * biasRate)`, which grows with
the load and not with the body size. A stack of 1 cm boxes sags 0.5 mm per
contact, 5% of a box, and buckles; with a scaled slop the position pass then
fights the sag. `CollisionResolver::UpdateRigidSolve` therefore estimates that
sag from each contact's cached impulse and sends the whole group to the rigid
single pass once it exceeds 1.25 times `min(penetrationSlop, 1% of the smaller
body's size)`, back below 0.75 times (the choice is cached per contact).
Groups with joints also take the single pass, so joint motors and limits keep
their one-solve-per-step semantics. Cached impulses record the substep count
they were solved with and are rescaled when a contact changes path.

`solverSubsteps = 1` takes the single pass everywhere and reproduces the
previous default step bit for bit.

## Coupled two-point normal impulses

A two-point manifold solves its two **normal** constraints together before
applying sequential tangent friction impulses. Single-point contacts and the
single-point CCD path retain the scalar normal/friction solve.

Position correction also treats a two-point manifold as one 2x2 block, the
LCP `lambda >= 0, w = K lambda + C >= 0, lambda . w = 0` with
`C_i = beta * (s_i + slop)` (positive, slack, while a point is within slop),
solved by enumerating the active sets. Correcting the two points one after the
other rotated the body toward the point corrected last; the position loop
usually stops after one pass, so every step left the same small tilt and a
20-box tower toppled after about 18 s. A degenerate block falls back to the
sequential correction.

For the common unit normal n, let c_ai=r_ai×n and c_bi=r_bi×n for the two
world-space contact levers. With m=invMassA+invMassB and ia/ib the inverse
inertias, the symmetric normal response matrix is

```
Kij = m + ia*c_ai*c_aj + ib*c_bi*c_bj.
b   = currentNormalVelocity - restitutionBias - K*accumulatedNormalImpulse.
```

The current velocities already include applied warm impulses. The solver finds
new accumulated impulses x satisfying x≥0, w=Kx+b≥0 and x_i w_i=0. It checks
both-active, first-only, second-only and neither-active sets. Restitution biases
are those prepared from incident point velocities and the existing bounce
threshold; no additional restitution or mass threshold is introduced.

The determinant is evaluated as positive terms, avoiding subtraction of nearly
equal K11*K22 and K12²:

```
det(K) = m*ia*(c_a1-c_a2)² + m*ib*(c_b1-c_b2)²
       + ia*ib*(c_a1*c_b2-c_a2*c_b1)².
```

With s=max(K11,K22,|K12|), inversion is allowed only when
det(K)/s² > 128 ε ((K11+K22)/s)², where ε is double machine precision.
This is a dimensionless conditioning check. Candidate impulse admissibility
uses 128 ε times the largest of |x1|, |x2|, |b1/K11| and |b2/K22|.
Residual admissibility for row i uses 128 ε (|Ki1*x1|+|Ki2*x2|+|bi|).
There is no absolute physical scale floor. Tiny admissible negative impulses
are clamped to zero and residuals are recomputed. Active rows must have residual
zero within this arithmetic tolerance; inactive rows must be nonnegative within
it. Stored body velocities are floats, so their subsequent contact residuals
also have float rounding error; double active-set admissibility is not a claim
of exact nonnegative post-storage velocity at every physical scale.

Singular or ill-conditioned patches first try admissible single-active or
neither-active solutions without matrix inversion. If no set is admissible, one
bounded projected scalar sweep supplies an **approximate fallback**. It does
not guarantee both complementarity rows are satisfied in one visit. Further
World iterations can improve the residual. Duplicate positions have a finite
single-active solution in the tested case. Matching duplicate feature IDs
consumes previous cache entries one-to-one, preserving distinct point impulses
instead of copying the first matching impulse into both points.

For accepted accumulated x, the correction is d=x-a. Its combined linear
impulse is n(d1+d2), while each body's torque uses its own complete lever sum.
All six proposed body velocity components are checked before either endpoint
or either accumulated normal cache is published. The same combined staging is
used by the approximate fallback and two-point warm starting. Cancellation in
the complete torque is retained; there is no observable intermediate normal
correction. Tangent friction uses the accepted normal impulses and also clamps
a stale cached tangent when the new normal bound shrinks, even if its slip
velocity is below the configured tolerance.

This is correction-level atomicity, not whole-step rollback. Earlier constraints
or a completed normal block may remain applied if a later tangent, position or
other World operation fails. Coupled normal solving does not couple friction,
position correction, different manifolds, or every body in a stack.

The [physical regressions](../tests/test_two_point_contact.cpp) include the
reported centered unit-box impact with one iteration: restitution 0/.5/1 gives
normal velocity 0/.5/1 with zero artificial angular velocity within a 2e-7
absolute float-state margin. Spinning/offcenter single-active impacts,
offcenter unequal dynamic bodies, momentum/angular momentum, elastic energy,
warm-started actual World impacts, extreme mass scales, duplicate features and
genuine derived angular overflow are checked independently. Absolute-margin
assertions disable Catch's default relative epsilon. Relative impulse/energy
checks retain explicit relative tolerances. See the [before/after benchmark](rigid-contact-benchmark.md#coupled-normal-solve-beforeafter-85)
for remaining stack drift and incline regressions.

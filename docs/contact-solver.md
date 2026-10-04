# Constraint-based contact solver

Each `World::step` detects collision manifolds once and prepares a contact
constraint for every active manifold point. Preparation converts contact
positions to body-local anchors and precomputes the normal and tangent
effective masses. Stable feature IDs reconnect each point to its cached normal
and friction impulses from the previous step.

The solver then runs three ordered phases:

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

## Coupled two-point normal impulses

A two-point manifold solves its two **normal** constraints together before
applying sequential tangent friction impulses. Single-point contacts and the
single-point CCD path retain the scalar normal/friction solve. Position
correction remains sequential and separate.

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

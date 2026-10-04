# Joints, islands and sleeping

Add both bodies to the World, construct a shared `DistanceJoint(a, b, length,
localAnchorA, localAnchorB)` or `RevoluteJoint(a, b, localAnchorA, localAnchorB)`, and
call `world.addJoint(joint)`. Local anchors default to body origins. A distance joint
holds a positive anchor separation; a revolute joint holds the anchors together
while permitting relative rotation. Getters report the current world-space anchors.
Removing a body removes its attached joints. Removing a joint wakes both endpoints.

Joint and contact velocity constraints share the iteration budget. Position
projection corrects drift using the configured correction bound and tolerance.
Distance constraints use scalar effective mass; revolute constraints solve a 2×2
point-mass system including angular inertia. Compliance is not yet implemented.
Connected bodies still collide unless their collision masks exclude
each other. The tests include analytical momentum transfer and 3,000-step pendulums.

Distance and revolute solvers compute rotated levers, point velocities, effective
masses, and impulses in double precision. No physical mass or nonzero distance is
clamped to an arbitrary minimum; only exactly coincident distance anchors use the
horizontal fallback direction. The point and coupled hinge solves expand their
determinants into positive mass/lever terms to avoid cancellation at long levers.
Coupled angular corrections use the angular row and expanded torque numerators,
so a small net torque is preserved between large opposing lever and motor/stop
impulses.
Physical impulses may exceed the float range when the resulting body state is
representable. Body positions, orientations and velocities remain float values,
so rounding of a published state still limits constraint accuracy at extreme scales.

Each correction stages both bodies' complete linear and angular results before
publication, including the additional angular impulse from a motor or stop. A
nonfinite intermediate or unrepresentable final component throws
`std::overflow_error` and leaves that correction's endpoint states unchanged.
Motor and stop accumulation advances only after successful publication. This is
correction-level atomicity: earlier accepted corrections and World integration
are not rolled back if a later correction fails.

Construction and solver preparation reject nonfinite consumed legacy body state
and invalid inverse properties with `std::invalid_argument`. Dynamic inverse mass
and inverse inertia must be positive; static values must be zero. Finite legacy
static velocities participate in relative point velocity, while static poses and
velocities remain unchanged by joint corrections. Corrections do not request an
external wake; existing World island and motor wake handling still applies.
Public `IJoint::getAnchorA/B()` return checked float world positions and throw
`std::overflow_error` when those coordinates cannot be represented. The solvers
use their internal double geometry directly, so they can process such anchors
when the resulting body state remains representable.

## Revolute speed motors

`joint->setMotor(true, speed, maxTorque)` drives angular velocity of body B relative
to A. Speed is in radians/second, with positive speed counterclockwise; the maximum
torque is non-negative (N m when using SI units). Motors default to disabled.
Use zero speed for a torque-limited brake and `setMotor(false, speed, maxTorque)`
to disable a drive. Set commands between steps or from post-solve collision events.
Changing a command wakes both endpoints and the next step propagates that wake to
their island. An enabled nonzero-speed motor with positive torque prevents sleep,
including when it is stalled or its speed is below the sleep-energy threshold.

Each step accumulates motor impulse across all solver passes, bounded by
`maxTorque * deltaTime`; increasing the iteration count does not increase available
torque. `getMotorTorque()` reports the signed average torque applied to B during the
last step, including zero for a zero-duration or sleeping step. Impulses on A and B
are equal and opposite. The drive updates velocity in the constraint phase after
pose integration, so driven motion converges with smaller fixed timesteps. As with
contact impulses, configured integration speed caps do not cap solver impulses.

## Revolute angular stops

`joint->setLimits(true, lower, upper)` restricts the angle of B relative to A,
measured from the joint's construction pose. `getAngle()` reports this principal
angle in radians, wrapped to [-pi, pi]. Both limits must lie strictly between -pi
and pi, with `lower <= upper`. Equal limits lock the relative angle; intervals
that exclude zero are allowed and position projection brings the joint into range.
Limits default to disabled; `setLimits(false, lower, upper)` restores free rotation.
Changing limits wakes the endpoints. Motors and limits can be enabled together.

Stops apply only inward angular impulses and allow immediate motion back into the
interval. A speculative velocity bound limits travel toward each stop over the
next step. Position projection uses a 0.005-radian convergence tolerance and a
0.2-radian correction bound per iteration, independent of the linear contact
tolerances. Active stops solve the anchor and angular constraints together in a
3-by-3 effective-mass system; they share the global iteration budget.

These are discrete principal-angle stops, not continuous multi-turn constraints.
Use sufficiently small steps to avoid crossing the angle wrap or both stops in
one step; extreme angular motion and initial errors can exceed the correction
budget. Smaller steps and more iterations improve coupled off-center constraints.

Dynamic contact/joint graphs form independent islands. Sharing the same static
floor does not merge separate islands. Each awake island is solved independently.
`islandCount`, `solvedIslandCount` and `solvedConstraintCount` report the work, while
`solverIterationCount` retains the configured global pass count for compatibility.

Set `SimulationConfig::enableSleeping=true` to skip integration and constraint
solving for settled islands. It is false by default, so numerical validation can
disable it explicitly. An island sleeps only after every member's kinetic energy
per unit mass remains below `sleepEnergyThreshold` for `sleepTimeThreshold` seconds.
Default thresholds are 0.0005 and 0.5 seconds. Sleeping zeros residual velocities.
Broad/narrow-phase contact detection continues, preserving lifecycle notifications.

External nonzero impulses, forces/torques, velocity/pose setters, joint edits,
configuration changes and collisions wake affected bodies. Wake propagation through
existing contacts/joints happens before the next step's force integration. Moving
or filtering a static support wakes previously connected bodies. Registered gravity
does not continuously wake resting bodies. If application code changes a force
generator in place or directly edits legacy state fields, call `Wake()` explicitly.

Native [prismatic joints](prismatic-joints.md) lock transverse motion and relative orientation while allowing translation along an axis fixed to body A.

## Reduced revolute velocity targets

Coupled revolute motors and stops eliminate the angular row **before** forming
the point-velocity target. Previously, constructing two full anchor velocities
and then subtracting the shared angular term could erase a small center velocity.
For example, a fixed radius-one support and a mass-one radius-one body at the
origin, both local anchors (1e20,0), incident center velocity (0,1), angular
velocity 1 and equal zero-angle stops left vy=1, omega=0 in one zero-duration
iteration. The simultaneous constraints require both zero. The reduced target
now preserves the center component; the same analytical test also covers anchors
through 1e30, rotated shared levers, common world translation, and mass scales
1e-30 through 1e30.

Let J rotate vectors by +90 degrees, ra/rb be the world-oriented local levers,
dv=vB-vA, d=ra-rb, m=invMassA+invMassB, ia/ib the inverse inertias, H=ia+ib,
and t the desired relative angular speed (motor target or stop bias). Define

```
shared = (ib*omegaA + ia*omegaB)/H
rbar   = rb + (ia/H)*d
weight = ia*ib/H.
```

The angular-eliminated point target and matrix are

```
reducedRhs = -dv + J*d*shared - J*rbar*t
S = m*I + weight*(J*d)*(J*d)^T
det(S) = m*(m + weight*|d|²).
```

The common anchor rotation has disappeared from the zero-speed target. Applying
the expanded adjugate keeps the shared term separate: d dot J*d is identically
zero and must not be recovered by cancellation of large floating-point products.
With c=ra cross rb and base=-dv, the linear impulse p is evaluated as

```
p = [m*base + weight*d*(d dot base) + m*J*d*shared
     - m*J*rbar*t + weight*d*c*t] / det(S).
```

Here d cross rbar equals c. The angular impulse increment is
q=(t-(omegaB-omegaA))/H - rbar cross p. Final angular states are recovered
directly from the eliminated row:

```
omegaA_final = shared - (ia/H)*t - weight*(d cross p)
omegaB_final = shared + (ib/H)*t - weight*(d cross p).
```

They are staged directly rather than adding a nearly -omega correction back to
omega. Static endpoint state remains unchanged. Neither endpoint nor the
accumulated motor/stop impulse is published before all proposed body components
are representable.

### Capped angular impulses

The candidate angular impulse is still accumulated and clamped to the existing
motor/one-sided-stop bounds. For an accepted fixed increment q, the point rows
must be recomputed for **that** q; substituting the unconstrained angular target
would violate the cap. The capped path uses center velocities directly, with

```
D = m*(m + ia*|ra|² + ib*|rb|²) + ia*ib*c²
Q = ib*omegaA + ia*omegaB
N_velocity = -m*dv - ia*ra*(ra dot dv) - ib*rb*(rb dot dv)
N_rotation = m*J*rb*(omegaA-omegaB-H*q) + m*J*d*(omegaA-ia*q)
             + c*[rb*Q + d*(ia*omegaB+ia*ib*q)]
p = (N_velocity+N_rotation)/D.
```

Expanded final angular numerators also keep the center contribution separate:

```
omegaA_final = [m²*omegaA
  + m*(|rb|²*Q + ia*(rb dot d)*omegaB + ia*(ra cross dv)
       - ia*q*(m-ib*(rb dot d))) + ia*ib*c*(rb dot dv)] / D
omegaB_final = [m²*omegaB
  + m*(|ra|²*Q - ib*(ra dot d)*omegaA - ib*(rb cross dv)
       + ib*q*(m+ia*(ra dot d))) + ia*ib*c*(ra dot dv)] / D.
```

These expressions follow by substituting the point solution into the angular
updates before evaluating them. They retain the small surviving spin in a
torque-capped drive or inward stop release. For fixed A, r=(R,0), mass-one B,
inverse inertia ib, incident vy=1/omega=1 and accepted q, the independent result
is omega_final=(1-ib*R+ib*q)/(1+ib*R²) and
vy_final=(ib*R²-R-ib*R*q)/(1+ib*R²). At R=1e20, a zero accepted q leaves
vy approximately 1 and omega approximately -1e-20, satisfying the point
constraint. An upper stop allows this inward release even though incident omega
was positive: the linear anchor coupling determines the accepted stop impulse.

An additional finite offcenter unequal-mass regression uses the independent
rational matrix

```
[9/8, -1/4, -3/4; -1/4, 9/4, 1/2; -3/4, 1/2, 3/2] * impulse
    = [9/2, -1, 15/4].
```

Its exact uncapped solution is (17/2,-27/25,711/100); fixing angular impulse
to 1/4 gives point impulse (657/158,-3/79). Tests compare both endpoint states,
motor accounting, momentum, angular momentum and energy with these external
solutions. A common-anchor unequal-mass lock also has the independent energy
target 16.5, down from 48.5. Every absolute-margin assertion disables Catch's
relative epsilon; relative motor-impulse checks specify their tolerance.

This arithmetic change is limited to coupled **velocity** solves. Free hinge
point constraints, distance joints and position solves retain their existing
paths. Their original full point-speed/update arithmetic can still lose a small
component when enormous rotations cancel; the inactive-hinge controls cover
the representable zero-incident-spin case, not arbitrary common-rotation
precision. No physical mass or lever floor is introduced. Float pose/velocity
storage, offsets already lost in the inputs, unresolved differences between
large lever products, rotational linearization and previous World operations
remain limitations. Failure preserves the attempted correction and its
unaccepted drive accounting, not the entire preceding World step.

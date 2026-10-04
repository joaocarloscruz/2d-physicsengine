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

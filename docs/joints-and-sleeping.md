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
point-mass system including angular inertia. No motors, limits or compliance are
implemented. Connected bodies still collide unless their collision masks exclude
each other. The tests include analytical momentum transfer and 3,000-step pendulums.

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

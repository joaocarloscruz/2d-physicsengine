# Collision lifecycle and CCD

Override `ICollisionListener::onCollisionBegin`, `onCollisionPersist`, and/or
`onCollisionEnd`, then register the listener with a World or Engine. Begin occurs
once for a new pair, persist once per subsequent step in contact, and end once
when it separates, fails filtering, or is removed. `clearBodies` also emits end.
End carries the last observed geometry. IDs are ordered, and normals point from
body A to B. Value events contain up to two world-space contact points.

Callbacks run after the solver. Removing bodies, clearing the world, or unregistering
listeners inside a callback is safe; resulting end events are queued until the
current notification completes. A listener added during notification starts with
subsequent notifications. Do not delete registered listeners. Reentrant stepping
is rejected. The old `onCollision(manifold)` still fires for begin/persist contacts.

## Sweeps

`body.SetCcdEnabled(true)` enables CCD for pairs involving that body. The default is
false. `SweepCircleCircle` and `SweepCirclePolygon` also expose standalone queries
with a normalized time-of-impact fraction, A-to-B normal, and world-space point.
Circle/polygon queries handle convex polygons of either winding, face impacts and
rounded corner impacts. Both shapes may translate. Initial overlap returns time zero.

World CCD processes the earliest approach, resolves its impulse, then advances the
remaining time using the changed velocities. Collision masks apply. Each pair
notifies at most once per World step even if multiple impacts occur. Transient
impacts begin a lifecycle even if the bodies have already separated by step end;
they end on the following step if contact does not recur.

`maximumCcdImpacts` bounds per-step work (default 32). Exhaustion sets
`ccdIterationLimitReached` and conservatively stops position advancement at the
last safe time. `ccdImpactCount` reports actual resolved impacts. Sweeps currently
scan all pairs, so reserve CCD for fast bodies. Pure polygon pairs and rotational
polygon motion are not swept; use small timesteps for those cases. Turning CCD off
retains the original discrete integration/collision path.
#### Mutation during stepping

Collision listeners run after solving and may add or remove bodies, joints,
particle systems, listeners, and force registrations. Force generators, custom
broad phases, and custom joint solvers run inside the active simulation step;
they must not change world structure or configuration. Such operations throw
`std::logic_error` before modifying the world. Queue these changes in application
code and apply them after `step` returns.

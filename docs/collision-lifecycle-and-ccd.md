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

The helper interval is `start + fraction * displacement`, rather than a pair of
stored endpoints. Each input must be finite and each radius positive and finite;
the implied endpoint may exceed float coordinate range. Circle/circle geometry
uses double relative motion and summed radii, a supporting-line clearance test
and a geometric disk chord instead of subtracting squared quadratic terms.
Regular contact normals/points are constructed from the target surface before
rounding the fraction. Initially overlapping circles retain the A-surface point
along the A-to-B center direction; coincident centers use normal (+1,0).
Unrepresentable returned float vectors throw `std::overflow_error`.

Circle/polygon geometry uses double finite offset faces and vertex disks, with
near-feature endpoint checks and a rounded expansion. Edges, winding, closest
points and relative displacements are calculated from the original inputs before
float arithmetic can overflow or underflow. Regular contact points lie on the
translated polygon feature. Initial polygon overlap retains its closest-point
convention: outside the polygon the normal points toward that point; inside it
points away from that point; zero distance uses (+1,0). Exact closest-point ties
use the first stored edge. Regular feature ties retain stored edge-then-vertex
traversal, comparing double line parameters before fraction conversion.
The vector-of-vertices overload still validates strict convexity in either
winding using the Polygon constructor; validation costs O(vertices²), followed
by O(vertices) sweep geometry. Neither helper changes World CCD policy.

`SweepHit::fraction` is still a float. On enormous displacement, it can round
away a small impact-time offset even when the helper correctly classifies the
hit and retains a local contact point. Interpolating with that fraction can yield
a different center position. World CCD still consumes float fractions and uses
float body trajectories; the helper correction does not establish accurate
extreme-scale World impact positions. Double line offsets can also lose small
details in nearly cancelling arbitrary directions. Choose meaningful coordinate
scales and timestep convergence checks. No rotating polygon policy changes are
implied by the helper arithmetic.

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

#### Circle-polygon manifold scale

The discrete circle-polygon narrow phase uses double relative geometry,
projections and `hypot` normalization. Every nonzero closest-corner direction
is tested; there is no absolute distance cutoff that can turn a small corner
gap into a collision. Local polygon edges are formed before adding body
translations. Both polygon windings and offset local vertices are supported.

Exact boundary touch remains excluded from discrete overlap (the query API
includes touching). Containment uses the minimum separating translation, with
stored edge order breaking exact ties. The normal is oriented for body A to B;
the contact point remains on the circle surface, including containment.
Reversing arguments negates the normal and preserves contact geometry.

Nonfinite transforms or mismatched shape arguments throw `invalid_argument`.
Manifold depth and vectors are checked before float conversion; a genuinely
out-of-range output throws `overflow_error`. This does not add precision to
stored float body positions or make arbitrarily ill-conditioned geometry exact.

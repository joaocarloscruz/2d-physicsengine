# Collision lifecycle and CCD

See the [rigid-contact benchmark](rigid-contact-benchmark.md) for reproducible
rest/stack/friction/impact measurements, analytical impulse regressions and the
current contact solver's measured accuracy limits.

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

With speculative contacts (the default step order, see
[contact solver](contact-solver.md#step-order)) the lifecycle follows the
end-of-step poses: a pair is in contact when it overlaps or is closer than
`0.25 * penetrationSlop`, so a body that lands during a step begins in that
step, and a resting body does not flicker. Pairs that were only speculative
(near but never touching) raise no events, are not counted in statistics or
`getPersistentContactCount()`, and never wake a sleeping body. Event
penetration is clamped to zero for touching pairs with a residual gap.

## Sweeps

`body.SetCcdEnabled(true)` enables CCD for pairs involving that body. The default is
false. World CCD covers every shape pair (circles, polygons and capsules, static
or moving) and sweeps rotation as well as translation; see
[World CCD](#world-ccd) below. `SweepCircleCircle`, `SweepCirclePolygon` and
`SweepCircleCapsule` also expose standalone translating queries
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
by O(vertices) sweep geometry.

`SweepHit::fraction` is still a float. On enormous displacement, it can round
away a small impact-time offset even when the helper correctly classifies the
hit and retains a local contact point. Interpolating with that fraction can yield
a different center position. World CCD still consumes float fractions and uses
float body trajectories; the helper correction does not establish accurate
extreme-scale World impact positions. Double line offsets can also lose small
details in nearly cancelling arbitrary directions. Choose meaningful coordinate
scales and timestep convergence checks.

### World CCD

World CCD processes the earliest approach, resolves its impulse, then advances the
remaining time using the changed velocities. Collision masks apply. Each pair
notifies at most once per World step even if multiple impacts occur. Transient
impacts begin a lifecycle even if the bodies have already separated by step end;
they end on the following step if contact does not recur.

Over the step each body moves from its start pose to its integrated pose at a
constant velocity and turns from its start orientation to its integrated
orientation at a constant rate (on the branch nearest `angularVelocity * dt`).
Two methods find the time of impact:

- A pair that includes a circle, where neither body turns, uses the exact
  translating sweeps above (circle/circle, circle/polygon on the polygon's
  world outline, circle/capsule). These sweeps are unchanged from before
  rotation was swept; a scene whose CCD pairs are circles against static or
  non-turning bodies gives the same results as before unless the freeze rule
  below applies or an impact sets a polygon or capsule turning.
- Every other pair (no circle, or a polygon or capsule that turns) uses
  conservative advancement on the signed distance between the two shapes. Each
  shape is a core (a point, a segment or a convex outline) plus a radius; the
  distance is exact while the shapes are apart and minus the overlap depth
  along the best separating axis when they overlap. Each iteration advances
  time by `(distance - target) / bound`, where `bound` is the closing speed
  along the current normal plus `|angularVelocity| * reach` for each body
  (`reach` is the core's farthest point from the body origin). The separation
  along a fixed normal cannot shrink faster than that, so no contact is
  skipped. The pair stops between a target of half the touching tolerance
  (`0.125 * penetrationSlop`) and three quarters of it, inside the touching
  band. A query takes at most 64 distance evaluations and otherwise stops at
  the last safe time it reached. The impulse uses the narrow-phase manifold at
  the impact pose, so a face that lands flat gets both of its points; when the
  narrow phase finds no contact there, a single point between the closest
  points is used.

Conservative advancement was chosen over a Box2D-style separating-axis root
finder because it needs only a distance routine (no per-feature separation
functions or cached simplex), treats rounded cores, segments and outlines the
same way, and is conservative by construction. Its weakness, slow convergence
when rotation dominates, did not show at the default limits: against
brute-force sub-stepping of 1000 random pairs moving at up to 200 m/s and
30 rad/s it took 2.5 distance evaluations on average (at most 8) with the
World's target, and its impact time was never after the target crossing.

A circle's orientation does not change its outline, so CCD leaves it as
integrated. A turning polygon or capsule that takes no impact ends the step at
its integrated orientation exactly; one that takes an impact continues from its
orientation at the impact with its new angular velocity.

The approach test uses the centres' motion for the exact sweeps and the
velocity of the closest points, rotation included, for the others.

A pair handled by the exact sweeps that is touching (within a quarter of
`penetrationSlop`) and still approaching gets one impulse per step. If it
touches and approaches again after that impulse, a neighbour is holding it
there: a body pressed between two others, or resting in a stack. Further
pairwise impulses would only pass the approach back and forth (each round
removes a share set by the mass ratio), so the pair is left to the next contact
solve, which sees all of those contacts together. For the rest of the step only
the circles' cores, a quarter of their radius, are swept for that pair, so it
can overlap by up to three quarters of the radius before CCD acts again.

A touching pair of the other kind is a contact that the step's contact solve
owns (as in Box2D, CCD only guards it against tunnelling). It may sink to a set
depth before CCD acts: two slops, or three quarters of the thinner shape's
smallest half-width if that is less, so that it never reaches the far side.
There it gets an impulse; pressed in again, one more.

A pair pressed in past its allowed depth (cores touching; reached again by the
sweep at or past the set depth, except for the first impulse of a pair that has
just sunk there; pressed in after its second impulse; or already that deep and
still closing) is frozen for the rest of the step, so it cannot creep deeper by
small steps: its moving bodies stop where they are and stay
stopped, even if another impact changes their velocity (the velocities are kept
for the next contact solve). Each freeze stops at least one moving body, so
freezes are bounded by the body count and do not use the impact budget below.
Earlier such a pair was simply no longer swept that step, so a body wedged and
driven inward could pass through: a CCD circle rammed into a 10 cm wall by a
heavy body ended 9 to 40 m beyond it in the legacy step order; it now stays in
front.

A touching pair that is not closing (resting or sliding) is still swept, since
rotation can close it later in the step (a spinning capsule whose far end comes
round, for example): it may sink to an overlap of the touching tolerance, or,
if it is already deeper, by half of the touching tolerance, before CCD acts
(after an impulse, no further than the set depth, or half the touching
tolerance past it when it is already there, where closing freezes it).
Corrections between two moving bodies (below) are not swept.

The world outline of a polygon is its validated local outline rotated and
translated in float, which can turn a strictly convex corner by an ulp or make
two close vertices coincide. World CCD sweeps that outline as it is (skipping
zero-length edges) instead of validating it again, so a valid polygon never
makes `step()` throw. The public `SweepCirclePolygon` still rejects such an
outline as input.

The sweep uses each body's actual displacement over the step, measured after the
velocity solve, so velocity added by joints, forces, impulses or contacts is
swept. The velocity limits (`maxLinearSpeed`, `maxAngularSpeed`) are applied
when velocities integrate and again after the joint and contact solve and after
CCD impacts and bounces, so impulses cannot carry a body past them; a velocity
inside the limits is never changed by the second application.
Position correction (joint and contact position solves, and the legacy
friction displacement) runs after the sweep, so it is swept separately: each
body's correction (its translation and, for a polygon or capsule, its rotation)
starts from its swept pose and stops at the first static body of a CCD pair it
would move into, with the same two methods. A correction that separates from or
slides along a surface is kept whole; one that would enter a static body is cut
at contact (dropped entirely when the body already touches it and closes on
it), so a joint pulling a CCD body toward a body beyond a wall leaves it against
the wall. This second sweep covers static targets only; corrections between two
moving bodies are not swept.

In the speculative step order a pair that CCD sweeps gets only the fixed
speculative allowance of `4 * penetrationSlop`; that now applies to every CCD
pair, not only to pairs with a circle.

`maximumCcdImpacts` bounds per-step work (default 32 impacts, counting also
advances to contacts that turn out not to close; freezes are free). A scene
with tens of CCD bodies hitting a pile at once can need more: the fuzz scenes
with about 40 CCD bullets of every shape ran out on several consecutive steps
after a volley with 32 and never with 64. Exhaustion sets
`ccdIterationLimitReached` and conservatively stops position advancement at the
last safe time. A CCD impact may wake a sleeping body; it joins its partner's
position correction that step. `ccdImpactCount` reports actual resolved impacts.
Sweeps currently scan all pairs with a CCD body, so reserve CCD for fast bodies.
A world without CCD bodies runs none of this. Turning CCD off retains the
original discrete integration/collision path.

CCD models each step as straight-line motion with a constant spin; it does not
integrate curved trajectories under changing forces within a step.
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

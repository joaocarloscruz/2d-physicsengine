# Capsules

`Capsule(halfLength, radius)` is a solid rounded rod: every point within
`radius` of a centered local spine from `(-halfLength, 0)` to `(halfLength, 0)`.
`halfLength` is half the **spine** length. The tip-to-tip extent is
`2 * (halfLength + radius)`. Body orientation rotates the spine, and the body
position is the spine midpoint, which is also the uniform-area centroid.

`halfLength` must be finite and nonnegative and `radius` finite and positive.
The constructor rejects an extent `halfLength + radius` or an area outside
finite float range. A zero half-length is the circle of the same radius:
area, inertia, bounds, manifolds and queries reduce to the circle results.

## Mass properties

With `h = halfLength` and `r = radius`, the capsule is a `2h x 2r` rectangle
plus two semicircular caps whose flat sides sit at `x = +/-h`:

- area `A = 4hr + pi r^2`;
- polar second moment about the centroid, per unit density,
  `J = 4h^3 r / 3 + 4h r^3 + pi r^2 h^2 + pi r^4 / 2`;
- `GetInertia(mass) = mass * J / A`.

Each cap contributes its own polar moment `pi r^4 / 4`, the parallel-axis term
`(pi r^2 / 2) h^2` and a cross term from its first moment `2r^3 / 3` about its
flat side. Tests compare area and inertia with an independent horizontal-slice
quadrature, not with another copy of this formula.

## Bounds

`RigidBody::GetAABB()` adds `h |cos(theta)| + r` and `h |sin(theta)| + r` to the
position with outward-directed rounding, the same conservative arithmetic as
circles and polygons.

## Contact manifolds

Collision dispatch supports circle-capsule, polygon-capsule and
capsule-capsule pairs in both argument orders. The normal points from body A
toward body B. Each contact position is the midpoint between the two surfaces,
so swapping the bodies negates the normal and leaves contacts unchanged.
Capsule-capsule tie-breaks use body IDs, like polygon pairs. Exact touching is
not overlap.

The penetration is the exact minimum separating translation of the rounded
shapes. It is the smallest projected overlap over the face normals of the
spines' Minkowski difference (each spine normal, and each polygon face normal)
and, when the spines are disjoint, the direction between their closest points.
A closest-spine distance alone would misreport crossing spines: two capsules
crossing at right angles escape by sliding one along its own spine, which can
be much shorter or longer than the spine distance suggests.

When a spine or polygon face is the separating axis, the incident spine or
polygon edge is clipped to that feature's extent. A parallel side contact then
produces two points, each kept only if it penetrates or lies within a
dimensionless allowance (`1e-5` of the narrower shape's width). The same
allowance prefers a face to a closest-point direction on near ties. No geometry
padding or absolute unit-dependent epsilon decides overlap.

Feature IDs use pair kinds `0x11`, `0x12` and `0x13` in the top byte, distinct
from circle, polygon and CCD IDs, so warm starting and contact events persist
across steps.

Tests check every manifold against an independent oracle: a double-precision
segment distance and a bisection search for the minimum separating translation
over 1440 directions. They cover deep crossing, containment in both directions,
coincident capsules, long perpendicular spines, near-parallel spines, endpoint
and side transitions, covariance under rotation and translation, and both
argument orders.

## Queries

`ContainsPoint`, `OverlapsCircle`, `RayCast` and `SweepCircle` (and their World
and Engine scans) accept capsule targets with the documented
[query semantics](spatial-queries.md): closed boundaries, contained starts at
fraction zero, tangency hits and limiting fractions on the closed segment. A
swept circle hits the capsule expanded by its radius. Exact ties prefer the
local `+y` side, then `-y`, then the `-x` cap, then the `+x` cap.

## Continuous collision

| Moving body | Target | Support |
| --- | --- | --- |
| Circle | Static capsule | Supported; translating sweep against the rounded spine |
| Circle | Dynamic capsule | Rejected |
| Capsule (any CCD flag) | Any body | Rejected |
| Polygon | Capsule | Rejected |

A CCD-relevant pair has at least one CCD-enabled body, at least one dynamic
body, and passes collision filtering. `World::step` throws
`std::invalid_argument` for an unsupported capsule pair **before** stepping
particles, applying loads or moving any body, so an unsupported cast is never
treated as a miss. Filter a CCD body away from dynamic capsules when both are
present. Zero-duration steps perform no sweep and are not rejected.

`SweepCircleCapsule` also accepts a prescribed translation of the capsule for
standalone relative-motion sweeps. No capsule rotates during a sweep.

## Other modules

- **Fluids.** `SampleRigidBodyBoundaries`, `CoupledFluidSimulation` and
  `FluidRigidCoupler::couple` reject capsule bodies with
  `std::invalid_argument` before changing particles or bodies. Capsules add no
  SPH boundary sampling or coupling accuracy.
- **Export.** World JSON writes
  `{"type":"capsule","halfLength":h,"radius":r}`. Export now rejects unknown
  shape tags instead of reinterpreting them as polygons.
- **Visualizer.** The SFML renderer draws capsules and picks them with
  `ContainsPoint`.
- **Browser.** WebAssembly bindings do not expose capsules yet.

Compound shapes, segment chains, concave shapes and rotating-shape CCD remain
separate formulations.

`examples/capsule_pile.cpp` is a bounded smoke scene: six capsules drop onto a
box and a static capsule ramp while a filtered CCD circle hits the ramp. It
checks resting heights and contact depth after four seconds.

# Multi-point contact manifolds

`CollisionManifold` represents a contact patch with up to two `ContactPoint`
values. Each point contains a world-space position, its local penetration depth,
and an opaque non-zero feature identifier.

Convex polygon pairs use reference/incident face clipping. Face-on contacts
normally produce two points; corner and edge contacts can produce one. Clipping
supports both clockwise and counter-clockwise convex vertex winding.

Feature identifiers describe the canonical body pair and selected polygon
features. They are independent of caller body order and remain stable under
small motion while the same features touch. Callers may compare identifiers for
equality across frames, but should not interpret their bit layout.

The solver stores accumulated normal and tangent impulses per feature. When a
patch persists, disappeared or new features do not inherit unrelated impulses.
Matched two-point warm impulses are combined before publication. Two-point
normal constraints are solved together, followed by sequential tangent friction;
position correction visits each point separately. See the [contact solver](contact-solver.md)
for active sets, conditioning and fallback behavior.

For compatibility, `CollisionManifold::contactPoint` is the first clipped point
and `CollisionManifold::penetration` is the maximum point penetration. New code
should use `contactCount` and `contacts`. Circle-circle and circle-polygon
collisions expose one contact through the same representation.

For polygon pairs, reversing the input bodies preserves contact positions and feature IDs while
swapping `A`/`B` and reversing the normal. This makes contact caches and event
consumers insensitive to broad-phase pair order.

Circle pairs keep the surface-of-A contact convention, or A's center with a +X
normal when centers coincide. Reversal therefore need not preserve their contact
position or reverse the coincident fallback normal. The public circle helper
rejects null bodies, non-circle shapes, invalid radii and nonfinite positions
with `std::invalid_argument`; circle orientation is irrelevant. Geometry uses
double intermediates and rejects unrepresentable float manifold output with
`std::overflow_error`.

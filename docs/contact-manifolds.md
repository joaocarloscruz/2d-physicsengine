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
patch persists, matching points are warm-started independently; disappeared or
new features do not inherit unrelated impulses. Positional correction is
applied once using the manifold's maximum penetration, while velocity and
friction impulses are solved at every point.

For compatibility, `CollisionManifold::contactPoint` is the first clipped point
and `CollisionManifold::penetration` is the maximum point penetration. New code
should use `contactCount` and `contacts`. Circle-circle and circle-polygon
collisions expose one contact through the same representation.

Reversing the input bodies preserves contact positions and feature IDs while
swapping `A`/`B` and reversing the normal. This makes contact caches and event
consumers insensitive to broad-phase pair order.

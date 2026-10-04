# Polygon manifold geometry

`CollisionPolygonPolygon` uses double edge directions, normals, separating-axis
projections and two-plane incident-edge clipping. Original local outlines are
preserved. Body translation, a rotated local anchor, and vertex offsets are kept
separate; neither edge normalization nor clipping planes use subtracted float
world vertices. Clockwise and counterclockwise strictly convex outlines work.
Geometry costs O(N² + M² + NM) for N and M vertices, including validation.

SAT rejects a pair if any separating face has minimum separation greater than
or equal to zero. Thus exact touching remains a miss, as before. No absolute
world-unit tolerance changes that classification.

For overlapping pairs, define `L` as the smaller of the polygons' minimum
face-normal thicknesses. Each thickness is the minimum, over the polygon's edge
normals, of its projection interval width; it is computed from original local
vertex differences in double. The intentional reference-face hysteresis and
accepted clipped-contact separation are both `1e-5 * L`. A long thin rectangle
therefore uses its thickness, rather than its longest edge, for this allowance.
Unit boxes against long floors retain the previous `1e-5` allowance. Uniformly
scaling a fixture scales the allowance with it.

Canonical body IDs determine pair order. The reference switches to canonical B
only if its separation exceeds canonical A's by the allowance. Exact face and
incident-normal ties retain the first stored edge index. Clipped contacts are
ordered along the reference edge; their existing encoded reference/incident
face IDs and contact ordinals are preserved. Each pair has at most two contacts.
Contacts stay on the incident edge, and penetration is the nonnegative distance
behind the reference plane. An accepted point just ahead of that plane has zero
penetration. Argument reversal preserves positions and feature IDs and reverses
the reported normal.

Null bodies, non-polygon shapes, invalid convex outlines and nonfinite transforms
throw `std::invalid_argument`. Final contact coordinates, depths and normals are
checked before conversion to finite float manifold fields; unrepresentable
results throw `std::overflow_error`. The helper changes no body state.

These intermediates fix edge-length overflow/underflow at ordinary float input
scales; they do not make arbitrary ill-conditioned geometry exact. Cancellation
in double rotated projections can still lose a tiny thickness compared with an
enormous oblique edge or offset. Public float contact coordinates can round away
local details at a huge common translation, or exceed the intentional plane
allowance after rounding on a high-aspect rotated shape. The classification and
depth are computed before that conversion. Input float outlines and transforms
also limit the geometry that can be represented. Use useful coordinate scales;
this helper alone does not establish extreme-scale broad-phase or World accuracy.

`test_polygon_manifold_numerics.cpp` checks analytical face patches at scales
`1e-35` through `1e35`, exact axis-aligned touch, shallow overlap, separated pairs,
both windings, reversed calls, containment, rotated corner clipping, original
offset local frames, huge common translations, and thin rectangles. A separate
interval SAT grid checks rotated classification outside a `0.002` normalized
boundary margin. Rotated thin contact-plane checks include an explicit bound for
the final float coordinate conversion, separate from the geometry allowance.

# Native point and ray queries

Include `physics/physics.h` (or `physics/core/spatial_queries.h`) and link the
installed `PhysicsEngine::Engine` target. Queries support circles and strictly
convex polygons without requiring narrow-phase headers.

```cpp
using namespace PhysicsEngine;
World world;
auto body = std::make_shared<RigidBody>(Circle(1), Material{}, Vector2{2, 0});
world.addBody(body);

bool inside = ContainsPoint(*body, {2, 0});
auto bodies = QueryPoint(world, {2, 0});
auto nearest = RayCastNearest(world, {0, 0}, {4, 0});
auto all = RayCastAll(world, {0, 0}, {4, 0});
if (nearest) {
    // fraction=0.25, point=(1,0), normal=(-1,0)
    RigidBodyPtr retained = nearest->body;
    RayHit geometry = nearest->hit;
}

// Query a shape directly without constructing a body:
Polygon box = Polygon::MakeBox(2, 2);
auto hit = RayCast(box, {0, 0}, {4, 0}, Vector2{2, 0}, 0.0f);
```

## Geometry and boundary semantics

`ContainsPoint(shape, point, position={}, orientation=0)` and
`RayCast(shape, start, end, position={}, orientation=0)` accept a shape's
world position and angle in radians. The corresponding body overloads use its
current position and orientation. Polygon vertices keep their original local
coordinates, including offsets from the origin; they are rotated and translated
without recentering. Both clockwise and counterclockwise polygons work.

Containment includes edges and vertices. A ray is the **closed finite segment**
from `start` to `end`; hits beyond either endpoint are excluded. Its first
contact returns `std::optional<RayHit>` with:

- `fraction`: double parameter in [0,1] along the segment.
- `point`: world contact point.
- `normal`: outward unit surface normal for a start outside the shape.

Tangency and endpoint contact count as hits. For polygon corner entry, the
normal belongs to the lowest stored edge index among equally timed entry faces;
it is a face normal, rather than an averaged corner normal.

A start **inside or on the boundary** immediately hits with fraction zero,
point equal to `start`, and normal (0,0), independent of the ray's direction.
This includes zero-length segments. A zero-length segment outside the shape
returns no hit. These are containment results and do not search for an exit face.

Coordinates, shape position and orientation must be finite. Invalid numeric
arguments or unsupported shape subclasses throw `std::invalid_argument`.
World helpers validate query coordinates even when the world is empty or the
filter excludes every body. They validate the transforms of bodies that pass
the filter; unrelated velocity/material state is not used.

Geometry and fractions use double intermediates to avoid overflow from float
coordinate subtraction, squared distances and cross products. Returned
`Vector2` points/normals still use floats. A reported fraction can round at
extreme scales even when the contact point remains distinguishable; ordering
uses the reported double fractions, with no epsilon-based merging.

Double precision does not provide exact arithmetic for arbitrary extreme input.
Nearly cancelling line-offset products can lose a small displacement relative
to enormous endpoints, so a poorly conditioned ray can miss a small target or
report an inaccurate contact. Prefer coordinates and segment lengths suited to
the target's scale. Derived points/normals that cannot be represented as finite
`Vector2` values throw `std::overflow_error` before conversion.

## World results, filtering and ownership

`QueryPoint` returns all containing `RigidBodyPtr` handles in ascending stable
body ID order. `RayCastAll` returns one `WorldRayHit` per intersected body,
ordered by hit fraction, then stable body ID for exact fraction ties.
`RayCastNearest` returns the first result under the same ordering, or
`std::nullopt`. Static and sleeping bodies are included. Results are not
truncated and do not depend on registration order.

Each point result and `WorldRayHit::body` retains shared ownership, so a body
remains accessible after removal, `clearBodies()`, or World destruction.
The hit geometry is a value snapshot; the retained body may later move.

All world helpers accept an optional `QueryFilter{categoryBits, maskBits}`.
Like collision filtering, both bit intersections must be nonzero:

```cpp
(body.GetCollisionCategoryBits() & filter.maskBits) != 0 &&
(filter.categoryBits & body.GetCollisionMaskBits()) != 0
```

The default category and mask are both `0xFFFFFFFF`, admitting every body
whose category and mask are nonzero. For example, a query with category 4 and
mask 2 includes a category-2 body only if that body's mask also accepts 4.
Zero category or mask excludes all bodies. Shape/body queries apply no filter.

These helpers scan current transforms linearly; they neither advance the
simulation nor rely on possibly stale broad-phase pairs. Simulation objects
remain unsynchronized; do not mutate a World concurrently with queries.
This API is native C++; WebAssembly bindings and accelerated shape casts are
separate future work.

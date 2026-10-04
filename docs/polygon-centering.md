# Polygon centroids and rigid-body setup

`Polygon::GetCentroid()` returns the centroid of a uniform-area polygon in its
existing local frame. It works with clockwise and counterclockwise strictly
convex outlines. `Polygon::Recentered()` returns a new polygon shifted by that
centroid; it preserves the source outline and its winding.

Use the recentered copy when constructing a dynamic rigid body. The current
rigid-body model treats the shape origin as its inertial center. Arbitrary
off-center dynamic shapes still require a future center-of-mass-aware solver;
these helpers make the documented centered setup explicit.

```cpp
#include <physics/physics.h>
#include <physics/math/matrix2x2.h>
using namespace PhysicsEngine;

Polygon outline = Polygon::MakeTriangle({-2, 0}, {2, 0}, {0, 5});
Vector2 center = outline.GetCentroid(); // (0, 5/3), within float rounding.
Polygon centered = outline.Recentered();

Vector2 originalOrigin{4, -2};
float angle = 0.7f;
Vector2 bodyPosition = originalOrigin + Matrix2x2::rotation(angle) * center;
auto body = std::make_shared<RigidBody>(centered, Material{}, bodyPosition);
body->SetOrientation(angle);
```

For an initial pose with rotation R, shifting the new body position to
`originalOrigin + R*center` preserves the original world outline within float
rounding. Local points and joint anchors in that outline become
`oldLocalPoint - center`. Impulses specified relative to the new body center use
that new frame. This recipe initializes geometry; it does not migrate an existing
off-center simulation's velocity, inertia or cached constraints.

`GetInertia(mass)` continues to mean inertia about the polygon's current local
origin. Calling it on the recentered copy gives centroidal inertia to float
precision. For the original outline, the parallel-axis relation is
`I_origin = I_center + mass*|center|^2`. Neither helper changes mass or density.

The centroid uses a triangle fan about a stored vertex, with double area/moment
accumulation, avoiding large absolute-coordinate moment subtraction. Returned
coordinates and recentered vertices remain floats. A new vertex outside their
finite range throws `std::overflow_error`; normal polygon validation rejects a
copy if rounding makes its outline degenerate. Failures leave the original intact.
The centered centroid may differ from exact zero by coordinate rounding.

Tests cover triangles, asymmetric trapezoids, translated boxes, both windings,
unit scaling, centered inertia, the parallel-axis relation, transform equivalence,
source preservation and unrepresentable shifted vertices.

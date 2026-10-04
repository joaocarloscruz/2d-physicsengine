# Grid and sampling numerical limits

Particle, fluid, rigid-body, and WCSPH boundary grids use signed `int` cell
coordinates. Coordinates are computed as `floor(double(position) / cellSize)`
and checked before conversion. Cell sizes and interaction radii must be positive
and finite. Nonfinite positions and invalid dimensions throw
`std::invalid_argument`; finite coordinates or rounded counts outside the `int`
range throw `std::overflow_error`. `UniformGrid` constructors and `setCellSize`
follow this same contract; invalid sizes no longer use an implicit fallback.

Neighbor windows use wider integer arithmetic and stop at the representable cell
domain. This permits queries at `INT_MIN` and `INT_MAX` without overflowing scan
endpoints or loop counters. Only positions within the representable cell domain
can be inserted. Exact distance comparisons in particle grids use double
arithmetic so squaring a large finite radius does not turn the cutoff into
infinity.

Each particle neighbor query, WCSPH boundary lookup, and rigid-body AABB
rasterization operation permits at most **16,777,216 cell visits** across the
whole input. Work is checked before traversing each window or rasterizing an
AABB. This prevents a finite but very small cell size from causing billions of
empty-cell lookups. Requests exceeding the budget throw `std::length_error`;
increase the cell size or reduce the interaction radius/scene extent.

`SphKernels2D::SquareLatticeMassScale` permits at most **1,048,576 lattice
points**, including points outside kernel support. Boundary sampling permits
at most **1,048,576 work units** per container append or complete rigid-body
sampling call. Circle samples and polygon sample attempts each cost one unit;
for samples inside a polygon, an attempt costs its vertex count to cover the
containment test. Filtered-out points still consume work. The circle layer count
is also bounded by the remaining work budget. Existing samples count toward a
container append's budget. Counts are checked using double division and ceiling
before conversion and before sample generation. Reduce support radius, enlarge
spacing, or split a scene into smaller sampling calls when a work budget is
exceeded. These are computational limits, independent of physical tolerances.

Boundary spacing squared must remain a positive finite float because it defines
sample volume. Sampling rejects invalid input or nonfinite generated positions
and velocities. Built-in container append methods keep the caller's existing
samples intact if sampling is rejected. Particle/fluid grid rebuilds construct a
replacement index and publish it only after all coordinates validate; a failed
rebuild preserves the previous cells and fluid statistics. Rejected fluid
neighbor queries also preserve the previous statistics.

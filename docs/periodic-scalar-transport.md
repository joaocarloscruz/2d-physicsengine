# Conservative periodic scalar transport

`PeriodicScalarTransport` is an independent native finite-volume building block
for cell-average scalar density `q` satisfying

\[
q_t+\nabla\cdot(\mathbf u q)=0.
\]

The caller supplies frozen periodic MAC face velocities. This first-order
donor-cell model does not advance velocities, project them, apply forces, diffuse
the scalar or connect to World, SPH, thermal networks or a shared clock. It is
not a complete Navier–Stokes step. Its [owned JavaScript binding](webassembly.md#owned-periodic-scalar-transport)
exposes the same independent state, clock and bounded operation. The
[Clawpack scalar conservation-law derivation](https://www.clawpack.org/riemann_book/html/Advection.html)
and [LeVeque chapter 20 examples](https://www.clawpack.org/gallery/gallery/gallery_fvmbook.html)
provide background; the equations and implementation below were derived
independently, without copying external solver code.

```cpp
#include <physics/physics.h>
using namespace PhysicsEngine;
PeriodicScalarTransport scalar({2, 2, .25, .5});
scalar.setState({1, 2, 1, 2});
scalar.setVelocities({{.25, .25, .25, .25}, {0, 0, 0, 0}});
ScalarTransportConfig options;
options.cflSafety = .9;
options.maxSubstep = .1;
options.maximumSubsteps = 10000;
options.maximumCellVisits = 100000000;
const auto diagnostics = scalar.step(.1, options);
const auto values = scalar.state(); // {1.1, 1.9, 1.1, 1.9}
```

Geometry is immutable and validates before allocation: each dimension is at
least two, total cells are at most 262144, spacings are finite/positive and
cell area/domain extents must be positive representable doubles. Unlike a
Laplacian solve, transport does not require a representable inverse spacing
squared. The scalar index is `i + columns*j`, with cell center
`((i+.5)*dx,(j+.5)*dy)`. `MacVelocityState::xFaces` stores `u` at
`(i*dx,(j+.5)*dy)`; `yFaces` stores `v` at `((i+.5)*dx,j*dy)`. Each periodic face
is stored once, without duplicate final rows or columns. Two-cell axes retain
both distinct interfaces between the two cells.

`q` may have any density units. `initialIntegratedScalar` and
`finalIntegratedScalar` measure `dx*dy*sum(q)` with units scalar×length²;
they are masses only if `q` has mass/area units. Velocities have length/time
units. The module's clock starts at zero and advances by successful requested
duration. All configuration, scalar, velocity and diagnostic snapshots own
their data. Setters validate both full shapes and finite values before copying;
they preserve the clock and last successful step diagnostics. A stored diagnostic
describes that earlier operation even after a setter changes the current state.

## Flux, positivity and the CFL bound

For the stored x-face at index `(i,j)`, let `L=(i-1,j)` and `R=(i,j)`, modulo
the grid. Its flux is

\[
F_{i,j}=u_{i,j}^+q_L+u_{i,j}^-q_R,
\qquad u^+=\max(u,0),\quad u^-=\min(u,0).
\]

A substep computes `h*F/dx` once from the old scalar, subtracts it from `L`
and adds the same value to `R`. Y-faces use the analogous bottom/top transfer
`h*G/dy`. Both directions use the same old state: this is an unsplit forward
Euler update, not directional splitting. Periodic pair transfers cancel in the
global sum in exact arithmetic.

For a cell, the sum of outward rates is

\[
a_{i,j}=\frac{u_{i+1,j}^{+}-u_{i,j}^{-}}{dx}
+\frac{v_{i,j+1}^{+}-v_{i,j}^{-}}{dy}.
\]

The donor matrix has a diagonal weight `1-h*a` and nonnegative neighbor weights.
Thus `h*max(a)<=1` preserves nonnegative data in exact arithmetic even for a
compressible velocity field. The configurable safety factor is **strictly
between zero and one**; the default is .9. The implementation rounds positive
outward-rate sums upward for its conservative `outflowRateBound`, rounds the
physical safety/rate limit downward, and uses equal substeps whose actual stored
`h` is no greater than `min(maxSubstep,safety/outflowRateBound)`. It checks actual
`maximumCfl` before allocation. No tolerant partition count can admit `h` above
that limit. The budget includes all chosen steps; there are no retries.

The sum of each donor-matrix row is `1-h*div(u)`. Therefore constant scalar
preservation and the global old-range bound require a **discrete divergence-free
advector**. A compressing flow increases density and can exceed the old maximum
while conserving the global integral. This is conservation-form density
transport, not the distinct equation `q_t + u·grad(q)=0` for a passive tracer
under compressible flow. Tests explicitly require nonuniform compressible flow
to change an initially uniform density.

`maximumAbsDivergence` uses differences of original stored face velocities before
spacing division when possible, avoiding loss of small differences atop common
large values. `discreteDivergenceFree` is true only when every **computed double
face-difference divergence is exactly zero**; this is an exact check on that
evaluation, not a proof about underlying real-valued velocities. Projected
velocities may retain residual divergence and need not qualify. In finite
arithmetic, constant/range behavior has the explicit diagnostic roundoff
allowance, rather than bitwise exactness. Successful nonnegative-input steps
publish no negative values; if rounding violates positivity, the operation fails
instead of clamping.

## Work, publication and float64 limits

`maximumSubsteps` has a hard ceiling of 1000000 and `maximumCellVisits` a ceiling
of 1000000000. Zero budgets are valid configurations but fail if required work
is positive. Positive-duration transport charges `(7+4*substeps)*cells` logical
cell visits: rate/divergence audit; two initial-summary scans; normalization;
staged copy; per-step zeroed increments, x-fluxes, y-fluxes and update; two final
summary scans. Each visit may access neighboring entries and do constant bounded
arithmetic. Allocator initialization/destruction is additional bounded linear
work, not a separate stencil visit. Complete substep/work bounds are checked
before scratch allocation; the initial three audit scans themselves are checked
before execution. There are no adaptive attempts outside those limits.

`step(0)` validates options, velocities, scalar diagnostics and their ranges;
charges `3*cells` visits; leaves state and clock unchanged; and publishes coherent
zero-duration diagnostics with zero substeps, CFL, drift and roundoff allowances.
Its outward-rate/divergence and initial/final integral/range measurements remain
meaningful. A positive duration with zero face velocities advances the clock
through the configured partition and retains scalar values exactly. Before any
successful step, `lastStep()` is the default zero-initialized diagnostic value.

Each face's signed normalized increment is applied equal/opposite with compensated
accumulation. Scaling by the initial maximum magnitude avoids needless overflow
of canceling large values. The stored scalar update adds its physical increment
to the original stored value, avoiding a normalize/rescale round trip when the
increment is zero. A normalized cancellation path handles an overflowing signed
increment when the complete updated scalar is finite. Scalar summaries use
compensated signed/absolute sums and exponent-staged scalar×area products.
No diagnostic changes the numerical state: there is no mass-fixing rescale,
silent clamping or absolute scalar floor.

Both the stored integral and divergence-free range are audited before publication.
For `N` cells and `S` accepted steps, let
`r=(32*N+64*S+64)*epsilon`. The conservation roundoff guard is
`r*max(initialAbsoluteIntegral,finalAbsoluteIntegral)`, and the range guard is
`r*max(initialMaxAbsScalar,finalMaxAbsScalar)`. These are scale-aware floating
arithmetic guards, without a unit-sized floor, not an interval-arithmetic proof
or a requested physical error tolerance. Independent tests impose tighter
analytical outcome checks. Signed quantities with severe cancellation can have
large relative uncertainty even when the absolute guard is small.

Finite inputs are not a promise that every derived quantity is representable.
Unrepresentable geometry, rates, divergence, products, stored updates, integrals,
roundoff bounds or clock increments fail explicitly. A nonzero scalar lost during
normalization, a nonzero normalized transfer/increment or diagnostic product that
underflows, and a duration too small to advance the stored clock also fail.
Some mathematically finite results requiring cancellation beyond this double
evaluation can be rejected. These limits are deliberate; a silent no-op would
falsely certify unresolved transport. Any step failure preserves the scalar,
clock, prescribed velocities and complete prior diagnostic snapshot. Setters
also publish transactionally. Compile without fast-math; the roundoff guards
assume ordinary IEEE double evaluation.

## Accuracy and reproducible checks

For constant velocities and a Fourier mode with grid phases `thetaX,thetaY`,
the amplification per substep is

\[
g=1-C_x-C_y+C_x e^{-i\,\mathrm{sign}(u)\theta_x}
+C_y e^{-i\,\mathrm{sign}(v)\theta_y},
\qquad C_x=h|u|/dx,\quad C_y=h|v|/dy.
\]

Modes usually lose amplitude: donor-cell transport has numerical diffusion.
Smooth cell-average errors converge at first order as space/time are refined;
pulse interfaces broaden and are not sharply tracked. Tests use the independent
cell donor matrix and this Fourier expression, both velocity signs, anisotropic
and two-cell axes, nonnegative pulses, conservative compression, deterministic
replay, and transactional budget/range failures. Temporal refinement is compared
to the exact exponential of the fixed-grid semidiscrete donor operator, isolating
time error from spatial diffusion. Spatial refinement compares actual cell
averages to continuous periodic translation, including the analytic sinc factors.

Run `run_tests "[scalar-transport]"` and `scalar_transport_demo`. The bounded
headless example reports a translating smooth mode on 24×12, 48×24 and 96×48
anisotropic grids at `T=.4`, `u=.7`, `v=-.2`, `maxSubstep=T/columns`.
Its JSON lines report the measured error, extrema, integrated-scalar drift,
roundoff allowance, CFL and deterministic work. Wall-clock time is not part of
the output. An installed-package consumer also exercises this public API.

One native Release run produced the following continuous cell-average errors;
all three integrated-scalar drifts were zero and maximum CFL was .32:

| Grid | Charged cell visits | RMS scalar error | Final min / max |
| --- | ---: | ---: | ---: |
| 24×12 | 29664 | .10502845768183652 | .858374 / 1.141626 |
| 48×24 | 229248 | .06488273779818525 | .794929 / 1.205071 |
| 96×48 | 1801728 | .03588313581039560 | .751577 / 1.248423 |

The coarser grids visibly damp this mode; their refinement ratios are not yet
exactly two. The asymptotic-order regression uses 32×16 through 256×128 and
checks the ratios approach two without relaxing the independent Fourier/matrix
accuracy checks.

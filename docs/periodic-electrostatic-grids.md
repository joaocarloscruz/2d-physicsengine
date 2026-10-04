# Neutral periodic electrostatic grid

`PeriodicElectrostaticGrid` solves homogeneous static electrostatics on a
rectangular periodic 2D domain:

\[
-\epsilon\nabla^2\phi=\rho,\qquad \mathbf E=-\nabla\phi.
\]

Charge is prescribed by the caller. In SI, `rho` is C/m³, uniform positive
permittivity is F/m, potential V and electric field V/m. Area-integrated charge
is C/m and energy J/m **per unit out-of-plane depth**. This is a smooth grid
charge-density model, not point-charge self-energy, particle-in-cell deposition,
charged-particle feedback, an extension of source-free TMz stepping, or automatic
World/SPH/material coupling. No clock advances. The zero harmonic/DC electric
field is assumed: a nonzero constant periodic E cannot be represented by the
gradient of a periodic potential and is not part of this solve.

The [electrostatic Poisson equations](https://farside.ph.utexas.edu/teaching/329/lectures/node61.html),
[field/source energy derivation](https://farside.ph.utexas.edu/teaching/jk1/lectures/node19.html)
and [explicit nullspace projection](https://petsc.org/main/manualpages/Mat/MatNullSpaceRemove/)
provide background. The repository discretization, solve and audits were
independently derived; there is no external solver dependency or copied solver code.

```cpp
#include <physics/physics.h>
using namespace PhysicsEngine;
PeriodicElectrostaticGrid grid({2, 2, 1, 1, 2});
ElectrostaticSolveConfig options;
options.absoluteGaussTolerance = 1e-10;
options.relativeGaussTolerance = 1e-10;
options.maximumIterations = 1000;
options.maximumCellVisits = 100000000;
const auto diagnostics = grid.solve({1, -1, 1, -1}, options);
auto result = grid.getSnapshot();
// phi={.125,-.125,.125,-.125}, Ex={-.25,.25,-.25,.25}, Ey=0.
// Field/source energy = .25 J/m in these units.
```

Default construction uses 16×16 cells, unit spacing and permittivity 1.
Geometry/medium are immutable. `getConfig()` and `getSnapshot()` return owning
copies; modifying or retaining them after grid destruction is safe. Every
`solve(charge[,options])` starts from zero, stages all data and publishes original
charge, effective charge, potential, fields, residual, curl and diagnostics
together after all checks pass. A failed input, work, range, convergence or audit
leaves the complete previous snapshot unchanged. There is no separate charge
setter. Before the first successful solve all arrays are zero and diagnostics
have `hasSolution=false`; `permittivity` already describes the constructed grid.

## Discrete incidence, gauge and compatibility

Arrays use row-major index `i+columns*j`. Charge and potential are at
`((i+.5)*dx,(j+.5)*dy)`. `xFaces` is at `(i*dx,(j+.5)*dy)` and `yFaces` at
`((i+.5)*dx,j*dy)`. Periods are `columns*dx`, `rows*dy`. Each face is stored once,
without duplicate final rows/columns. Indices below wrap periodically:

```
Gx phi(i,j) = [phi(i,j)-phi(i-1,j)]/dx; Ex = -Gx phi
Gy phi(i,j) = [phi(i,j)-phi(i,j-1)]/dy; Ey = -Gy phi
D E(i,j) = [Ex(i+1,j)-Ex(i,j)]/dx + [Ey(i,j+1)-Ey(i,j)]/dy
A = -D G = -Lap
curl E(i,j) = [Ey(i,j)-Ey(i-1,j)]/dx - [Ex(i,j)-Ex(i,j-1)]/dy
```

Curl is at corner `(i*dx,j*dy)`. Both directed interfaces remain distinct on a
two-cell axis; their repeated neighboring cell contributes twice to A.
Uniform area-weighted summation gives `G=-D*`, A symmetric positive semidefinite,
constant potential its nullspace, and commuting differences make curl G zero
in exact arithmetic. A solution has zero-mean potential gauge; the measured
stored mean/curl/field means may contain roundoff and are reported.

Periodic compatibility requires zero integrated charge. Let `m` be the
compensated mean of the **original** caller array and `a` its mean absolute value.
The sole eligibility allowance is fixed `64*double_epsilon*a`, with no unit
floor and no configurable physical background. Material nonneutrality
`abs(m)>allowance` is rejected. This narrow floating-point policy is a numerical
roundoff band, not a proof about uncertainty in the caller's physical source.

Two uniform subtraction passes remove the measured mean, with the second mean
also required to lie in that allowance. The original/effective stored arrays
are retained separately. Diagnostics report both charge means and integrated
charges, total `removedChargeMean`, actual `maximumSourceCorrection`, and the
per-cell bound `2*neutralityMeanAllowance+4*double_epsilon*max(abs(original))`.
Actual effective mean must still lie in the neutrality band. Floating subtraction
can leave a small remaining mean; it is **not discarded from the stored Gauss
audit**. If that mean alone exceeds the requested RMS target, the solve rejects.
Requests below representable accuracy can also fail later. Mean removal never
permits a materially nonneutral source to enter the solver.

## Bounded solve and actual physical residual

Projected conjugate gradients solve in the mean-zero subspace, using the
Laplacian diagonal to normalize coefficients and the effective source maximum
to normalize magnitudes. Residuals and directions are projected explicitly.
Every success must satisfy the residual reconstructed from the published field:

```
r_effective = epsilon*D(stored E)-stored effectiveCharge
target = absoluteGaussTolerance + relativeGaussTolerance*RMS(effectiveCharge)
RMS(r_effective) <= target
```

The absolute tolerance has charge-density units, and the relative tolerance is
dimensionless. Both are finite/nonnegative; neither has a hidden lower floor.
The recursive CG residual is only a candidate trigger. On candidate failure,
the algorithm recomputes a residual and restarts within the same work/iteration
budgets. `originalGaussRms` and `maximumAbsOriginalGauss` independently recompute
`epsilon*D(stored E)-originalCharge`; they expose the full original physical
equation defect, including the reported source correction. `gaussResidual` in
the snapshot refers to the effective source. No accuracy statement follows
solely from finite residuals or energy agreement; independent spatial tests
measure continuum error.

Zero iteration budget is valid. It succeeds only if the actual zero-potential,
zero-field candidate already meets the requested tolerance. Thus a deliberately
loose tolerance can accept a nonzero source with zero field; `zeroSource` remains
false. Zero charge succeeds with zero iterations/fields/energies and coherent
`hasSolution=true`, `zeroSource=true` diagnostics, but still needs **52*N** logical
cell visits for validation, projections, candidate and physical audits.

Each dimension is at least two and total cells at most 262144, validated before
allocation. `maximumIterations` is at most 1000000; `maximumCellVisits` at most
1000000000. Zero work budget cannot certify even zero charge. Each explicit
full-grid arithmetic/validation scan charges N visits **before** executing;
each visit can read neighboring entries and perform bounded constant work.
An ordinary CG iteration charges 13*N, in addition to initialization, candidate
checks, any restarts and final audits. Allocation initialization/destruction and
plain vector copies add bounded linear work and are not stencil visits.
No uncharged adaptive attempts or unbounded convergence loop are permitted.

## Energy and float64 limits

Using the actual stored values and area `V=dx*dy`, diagnostics evaluate

\[
W_E=\frac{\epsilon V}{2}\sum( E_x^2+E_y^2),\quad
W_\rho=\frac{V}{2}\sum\rho_{effective}\phi,\quad
C_r=\frac{V}{2}\sum\phi r_{effective}.
\]

Discrete summation by parts gives `W_E-W_rho=C_r`. The residual correction has
either sign; `residualEnergyBound=V*N/2*RMS(phi)*RMS(r_effective)` is its
Cauchy–Schwarz bound in exact arithmetic. `energyIdentityError` measures
`W_E-W_rho-C_r`. The scale-aware arithmetic guard is
`(32*N+128)*double_epsilon*max(W_E,abs(W_rho),abs(C_r))`, zero at zero energy;
the solve rejects an identity error larger than this guard. This is a floating
arithmetic heuristic, not an interval bound or a requested physical tolerance.
Zero-start CG often makes `phi` orthogonal to the remaining residual: field/source
energies can agree while the potential is still inaccurate. Tests compare a
loose solution to an independent dense solution to expose that distinction.

Positive finite input alone does not guarantee representable derived quantities.
Geometry requires positive finite inverse spacing squared, diagonal, normalized
directional coefficients, area and domain extents. Extremely unequal spacings
whose directional coefficient vanishes are rejected rather than silently becoming
a 1D model. Products/dots use exponent staging, compensated sums and scaled norms;
field energy accumulates an aggregate norm before squaring. Lost nonzero scaled
products/normalization, unrepresentable differences, norms, energy, tolerances or
source-correction bounds reject transactionally. Cancellation beyond these
double evaluations can still reject a mathematically finite result. These are
explicit scale limitations, not arbitrary physical floors or a promise of
arbitrary-range accuracy. Compile without fast-math.

The physical Gauss audit stages permittivity into each difference/spacing term;
unweighted `div(E)` need not be representable when `epsilon*div(E)` is finite.
It also scales opposite-sign faces first if their raw difference overflows.
The unweighted potential-gradient/curl evaluations can still reject individually
unrepresentable terms even when their final mathematical cancellation is finite.

## Independent checks

Run `run_tests "[electrostatic]"` and `electrostatic_grid_demo`.
Tests independently use a reduced dense Gaussian solve with gauge fixing,
Fourier eigenvalues `4*sin²(thetaX/2)/dx²+4*sin²(thetaY/2)/dy²` and analytic
staggered fields. They check anisotropic/two-cell grids, smooth nonuniform neutral
sources, superposition/sign/permittivity/charge scaling, stored Gauss/curl/means,
energy correction, narrow neutrality treatment, deterministic replay, copied
snapshots, and complete rollback on late range/work/convergence failures.
The installed-package consumer exercises this public header through the umbrella.

The bounded headless example uses a unit-square analytic sinusoidal potential,
16×8 through 128×64 anisotropic grids, permittivity 2, and its **continuum** source.
JSON lines report second-order potential/face-field errors, actual stored Gauss
and original-source defects, correction/neutrality band, energies, curl and work.
Charge sampling is at cell centers; this example does not infer accuracy from the
Poisson residual alone. [Owned JavaScript bindings](webassembly.md#owned-periodic-electrostatic-grids)
expose the same static solve, copied snapshots, diagnostics and native budgets.

One Windows Clang 23.1.1 optimized Release run produced:

| Grid | Potential RMS error | Face-field RMS error | Cell visits | Effective Gauss RMS |
| --- | ---: | ---: | ---: | ---: |
| 16×8 | .0161368 | .0587688 | 8320 | 2.31e-13 |
| 32×16 | .00399017 | .0145970 | 33280 | 1.21e-12 |
| 64×32 | .000994808 | .00364332 | 133120 | 4.50e-12 |
| 128×64 | .000248531 | .000910458 | 532480 | 1.36e-11 |

Errors decrease by about four, while the arithmetic residual rises with finer
spacing but remains below the requested 7.92e-9 charge-density target. Each run
takes one CG iteration; the charge mean correction is at most 7.11e-15, well
within the roughly 1e-12 neutrality band. Wall-clock timing is not an accuracy
or work-budget metric.

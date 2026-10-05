# Native periodic incompressible flow

`physics/core/fluids/periodic_incompressible_grid.h`, included by
`physics/physics.h`, supplies an owning `PeriodicIncompressibleGrid`. Its private
`PeriodicMacGrid` retains the existing projection and diffusion algorithms.
Geometry, homogeneous density rho > 0 (kg/m³) and kinematic viscosity nu >= 0
(m²/s) have no setters. Velocities (m/s), time (s), step diagnostics and the last
MAC projection/diffusion snapshots are owning copies. Setting finite velocity
arrays retains time and all history. There is no World, particle or SPH coupling.

The model is `du/dt + div(u tensor u) = -grad(p)/rho + nu Laplacian(u)` and
`div(u)=0`. The domain is periodic in both axes. Each face occurs once at
`k=i+columns*j`: u at `(i*dx,(j+.5)*dy)`, v at `((i+.5)*dx,j*dy)`.
Values are **face-center point samples**, not exact dual-volume averages.
Momentum per unit depth is `rho*dx*dy*sum(q)`; kinetic energy per unit depth
(J/m) is `rho*dx*dy/2*sum(u²+v²)`. These are midpoint quadratures for continuum
fields, and exact definitions of the discrete diagnostics. Pressure samples
(Pa) are at `((i+.5)*dx,(j+.5)*dy)` with the native zero-mean gauge.

## Conservative staggered update

Every positive step first projects its input, including initially compressible
data. That work and its energy ledger are explicitly reported as
`initialProjection`. Each adaptive substep freezes the currently stored,
projected velocity as its advector, performs donor-cell forward Euler momentum
advection, backward Euler constant-viscosity diffusion, and pressure projection.
This is first-order Lie splitting and first-order donor spatial transport.
The component Laplacian and pressure gradient/divergence have their native
centered discretizations. Their combination is not a second-order time method.

For the u dual volume the right x speed is `(u[i,j]+u[i+1,j])/2` and the upper
y speed is `(v[i-1,j+1]+v[i,j+1])/2`. For the v dual volume they are
`(u[i+1,j-1]+u[i+1,j])/2` and `(v[i,j]+v[i,j+1])/2`. Indices wrap periodically.
Each interface stores one flux `w*q_upwind`; both adjacent cells consume that
same flux with opposite signs. Forward and backward interfaces remain distinct
on two-cell axes. Telescoping gives momentum conservation in exact arithmetic.
The implementation measures stored means and rejects drift beyond its reported
scale-dependent roundoff allowance, without correcting a mean.

For both components, h is chosen so that
`h*(max(wx_right,0)/dx + max(-wx_left,0)/dx +
    max(wy_top,0)/dy + max(-wy_bottom,0)/dy) <= cflSafety`,
where `0 < cflSafety < 1` (default .8). This makes donor weights nonnegative.
The represented dual divergence D is measured; it need not vanish when the
native projection has a finite residual. The update's row sum is `1-h*D` in
exact arithmetic. Diagnostics instead compute the actual represented outgoing
and incoming weights and maximum row sum R. Conservative column sums equal one
in exact arithmetic, so weighted Jensen gives `K_adv <= R*K_before`.
The accepted energy change is checked against `(R-1)*K_before` plus an explicit
roundoff allowance. This does not promise unconditional kinetic-energy decay.

The exact donor FE ledger for each component is

```
K_adv - K_before =
  -rho*dx*dy*h/2 * sum(D*q²)
  -rho*dx*dy*h/2 * sum_faces(abs(w)*jump(q)²/spacing)
  +rho*dx*dy/2   * sum((q_adv-q)²).
```

`dualDivergenceWork` is signed; `donorDissipation` is nonnegative numerical
diffusion, and `forwardEulerIncrementEnergy` is nonnegative. Stored increments
are formed before squaring. Energy changes use `rho*area*(q*delta+delta²/2)`
instead of subtracting two large total energies. Signed sums are compensated.
Per-substep diagnostics expose advection storage error and roundoff allowance.

Diffusion contributes `-gradientDissipation - incrementKineticEnergy +
residualWork`. Each projection (including the initial one) contributes
`-correctionKineticEnergy + divergencePotentialInnerProduct`. The latter signed
term is residual work, bounded by the native `residualEnergyBound`. The final
stored energy increment is checked against this complete ledger. Native
residual bounds and roundoff allowances are reported separately from donor
loss. No velocity clamp, positivity floor or posthoc repair is applied.

`lastProjection().pressure` is the **latest substep** pressure, converted as
`rho*potential/h`, not a full-step average; `diagnostics.timeStep` gives h.
The initial projection has a separate diagnostic record and uses the requested
dt for conversion. Pressure error can increase as h shrinks at fixed absolute
divergence tolerance: convergence studies must verify residual convergence,
rather than assume pressure accuracy from velocity accuracy.

## Bounds, transactions and supported range

`step(dt, options)` accepts finite dt >= 0. The shared iteration ceiling spans
initial projection, every diffusion component and every later projection.
Every inner operator receives only the remaining iteration/cell-visit budget.
Wrapper cell scans, interpolation, outgoing-rate scans, fluxes, stored audits,
setter validation/copies and copied velocity/MAC snapshots are charged too.
A cell visit is one loop over the grid index; a fused u/v loop counts once,
separate component loops count twice. Snapshot/copy costs are conservatively
charged per array. Native operators retain their own scan accounting. Vector
allocation is bounded by grid size; metadata is bounded by the substep ceiling.
All successful per-substep records have at most 10,000 entries, approximately
7 MB of records at that ceiling. Vector capacity is less than twice that bound;
publication also stages a returned diagnostics copy. No unbounded retry loop
exists. Defaults are 1,000
substeps, 10,000 shared iterations and 100,000,000 cell visits; hard maxima are
10,000 substeps, 1,000,000 iterations and 1,000,000,000 visits. Limits may be
zero to request immediate budget failure.

All positive-step fields, clock and snapshots are staged until final stored
divergence, energy and momentum audits pass. A late budget, range, allocation or
residual failure preserves every published snapshot. A zero step charges a
fresh three-grid-scan diagnostic, preserves fields/time/operator snapshots and
publishes zero substeps. It does not project compressible input.

Geometry inherits MAC limits (at least two cells per axis, at most 262,144
cells, representable reciprocal spacings, area and Laplacian diagonal).
Nonfinite arrays/configurations reject. Product coefficients and diagnostic
terms reject finite-range overflow or nonzero underflow; stored differences
are measured directly. The complete step also inherits the projection's raw
squared-norm range restrictions, even when its physical energy could otherwise
be represented using extreme density scaling. Very small increments or clock
increments that cannot be represented reject explicitly. Tolerances are user
controls with their actual native units, never enlarged to force acceptance.

## Reproducible controls and comparisons

Build/run `incompressible_flow_demo` for 20 bounded steps on a 24² Taylor–Green
grid. The native `[incompressible]` tests independently assemble dense Poisson
and BE diffusion matrices and oriented staggered fluxes on anisotropic grids
and both two-cell axes, starting with compressible input. They check pressure,
momentum, energy identities, exact budget accounting, replay and late rollback.

The shear control `u=U`, `v=sin(k*x)` has exact discrete substep multiplier
`[1-|U|*h/dx*(1-exp(-i*sign(U)*k*dx))] /
 [1+4*nu*h*sin²(k*dx/2)/dx²]`. Positive, negative and zero U verify uniform
transport; numerical damping depends on U, so exact Galilean invariance is
not claimed. Fixed-grid time refinement compares to the **semidiscrete**
exponential of its donor/diffusion generator. Spatial refinement uses a small
fixed timestep and the continuum `exp(-nu*k²*t)*sin(k*(x-U*t))` reference.

Independent Taylor–Green point references on the square 2*pi domain are
`u=A*sin(x)*cos(y)`, `v=-A*cos(x)*sin(y)`,
`p=rho*A²/4*(cos(2*x)+cos(2*y))` with
`A(t)=A(0)*exp(-2*nu*t)` and energy
`rho*pi²*A(0)²*exp(-4*nu*t)`. Velocity, pressure and energy are compared at
their matching locations, with spatial refinement separated from temporal
tests. A fixed 12² Taylor–Green time study additionally compares velocity and
pressure with a 256-step numerical time reference, using a verified 1e-13
absolute projection tolerance. That reference isolates time error on the same
grid and is explicitly a numerical reference, separate from the independent
analytic spatial comparisons. The coarse inviscid case deliberately reports donor energy loss;
under-resolved results are not claimed to reproduce inviscid continuum flow.
Walls, free surfaces, obstacles, variable density and turbulence models need
different formulations. Browser bindings are a subsequent task.

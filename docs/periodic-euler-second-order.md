# Second-order periodic ideal-gas Euler option

`PeriodicEulerGasGrid::stepSecondOrder(duration, EulerGasSecondOrderConfig)` is
an explicit native option for the same periodic homogeneous ideal-gas model,
geometry, units, strict admissible stored state and owning snapshots described
in [the first-order documentation](periodic-euler-gas.md). `step()` remains the
first-order default, with unchanged update arithmetic and regression inputs.
There are no additional material models, walls, sources, viscosity or World
coupling. This second-order option is native-only; the browser Euler binding
retains first-order `step()` and exposes neither `stepSecondOrder()` nor its
historical observer. Higher order describes smooth resolved solutions;
it does not imply second-order accuracy at shocks or clipped extrema.

## Reconstruction and invariant domain

Write `U=(rho,mx,my,E)` and `G={rho>0, E-|m|^2/(2rho)>0}`. Each directional
conserved slope uses the monotonized-central limiter

```
MC(dl,dr) = minmod(2*dl, (dl+dr)/2, 2*dr).
U(x,y) = Ubar + theta*((x-xc)/dx*sx + (y-yc)/dy*sy).
```

Slopes are computed with shared component scales `max(rho)`, `sqrt(max(rho)*max(E))`
for both momenta, and `max(E)`. One common scalar theta per cell scales all eight
conserved slopes. Check the four face midpoints and four corners for strict
stored admissibility and representable derived primitives. Try theta=1,1/2,...,
2^-32, then theta=0, at most 34 trials. Theta=0 copies the original stored center
exactly; it never normalizes/rescales that fallback state. The opposite face
pairs each average to Ubar in exact arithmetic, and the cell mean is unchanged.
Because G is convex, admissible corners also imply the affine polynomial is
admissible throughout the rectangular cell in exact arithmetic. Finite stored
face checks remain mandatory in floating arithmetic.

Only reconstructed face states set the global directional speeds: alphaX is
the maximum |u|+c over x face states, alphaY the analogous y maximum. Corners
are an additional limiter audit, not flux quadrature points. The usual shared
Rusanov face flux is computed once and contributes opposite increments to its
two cells. This is conservative reconstruction, not independent clipping of
cell updates, density/pressure floors, or repair of energy after a step.

The stricter unsplit sufficient positivity bound is `C=h*(alphaX/dx+alphaY/dy)
<1/2`. To derive it, set rx=alphaX/dx, ry=alphaY/dy, r=rx+ry and own-face
weights wx=rx/(2*r), wy=ry/(2*r). The old mean equals the sum of its four face
states with these weights. After substituting the face flux, an own x face
contributes `b_x*Uface +/- h/(2*dx)*F(Uface)`, where
`b_x=wx-h*rx/2=wx*(1-C)`. Each neighboring x face contributes
`h*rx/2 * (Uneighbor +/- F(Uneighbor)/alphaX)`; y is analogous. All coefficients
sum to one. The own-state flux ratio is admissible whenever
`(h/(2*dx))/b_x <= 1/alphaX`, equivalent to C<=1/2. Every term is therefore an
admissible LF split or a convex combination of a split and its unsplit state.
The ideal-gas LF split calculation in the first-order documentation holds for
every gamma>1, rather than assuming gamma<=2. This proves a sufficient
exact-arithmetic invariant domain for the particular four-face linear scheme.

The general convex positivity and SSP stage arguments are discussed in the
primary [Zhang/Shu rectangular Euler paper](https://www.math.purdue.edu/~zhan1966/research/paper/euler.pdf)
and [positivity review](https://www.math.purdue.edu/~zhan1966/research/paper/review.pdf).
The linear decomposition above and implementation are independently derived;
this is not an implementation of those papers' general high-order quadrature.

## SSPRK2, changing speeds and transaction

For a candidate h, prepare U0, apply U1=FE(U0,h), prepare U1 again, and apply
U2=FE(U1,h). Accept `Unew=.5*U0+.5*U2`. Thus each FE update is positive under
its own reconstructed stage's bound, and the final SSP convex blend is positive.
The implementation requires `2*h*r <= cflSafety <1` at both stages, default
cflSafety=.9. It rounds computed face speeds/rates upward and the computed CFL
duration downward, then takes the minimum with exact maxSubstep and remaining
duration. These local roundings are not a global interval proof. Every stored
FE output and final blend is separately audited for finite rho>0, internal
energy>0 and representable derived primitives.

If U1's actual reconstructed bound rejects the candidate h, discard the whole
scratch attempt and retry U0 with `min(h/2, roundedStage1Limit)`. Recompute both
stage reconstructions and speeds on each attempt. Only stage-CFL violations
retry; an arithmetic, admissibility, conservation or allocation failure rejects
the entire call. No state repair or positivity retries are applied to FE
outputs. Bounded slope trials operate on a polynomial about a fixed cell mean.
Extreme-range slope trials may also require scaling; `rangeLimitedCells`
reports those cells rather than concealing the reason for the scale reduction.

All accepted scratch substeps are unpublished until the entire duration passes
its final stored conservation audit. A failure preserves fields, clock,
lastStep and lastSecondOrderStep, including failures after earlier accepted
scratch substeps. `setState()` preserves the clock and both historical records.
`lastStep()` returns the most recent base conservation snapshot from either
method. `lastSecondOrderStep()` returns the most recent second-order record;
subsequent first-order calls leave that historical record intact. Initially it
is an all-zero no-step record with minimumSlopeScale=1. Returned values own
their data. Adding the record changes the native class layout; rebuild binaries
using this header/library together.

Zero duration preserves fields/clock while publishing fresh records in both
observers. It requires 3N visits, even with zero allowed substeps/attempts, and
can fail on work/range errors. It sets zeroDurationNoOp=true and no stage/trial
counters; it is not a no-op on diagnostic observers.

## Resource and numerical-range contract

`EulerGasSecondOrderConfig` inherits the first-order options and ceilings.
maximumAttempts defaults to 20000 (hard 1000000); maximumRetriesPerSubstep
defaults to 16 (hard 64). An initial candidate counts as an attempt; the retry
limit counts discarded stage-CFL attempts before an accepted substep.
maximumSubsteps counts only accepted substeps. All attempts and reconstruction
trials, including rejected work, consume maximumCellVisits. Before allocation,
a positive call requires at least 19N visits; a zero call requires 3N.

A successful positive call with A attempted substeps, S accepted substeps and T
individual cell reconstruction trials charges exactly

```
(4 + 8*A + 5*S)*N + T.
```

This includes two initial/final summary passes each, two preparation passes per
reconstruction, one cell visit per reconstruction trial (the first includes MC
slope calculation), four visits per completed FE stage (clear/x faces/y faces/
stored update audit), and one final SSP blend visit per accepted substep.
There are 2A preparations, A+S completed FE stages and S blend passes. With
R=A-S rejected attempts, the minimum is `(4+15*S+10*R)*N`; extra slope trials
are added explicitly. A trial checks eight points and bounded scalar work.
Final summary work is reserved before every operator/trial scan. Scratch has
48 double arrays per cell plus the object's 4 state arrays, approximately
96 MiB scratch at the 262144-cell ceiling. Allocation initialization, copies,
and destruction are bounded linear work outside the operator-visit count.

The original explicit arithmetic/range limitations still apply: complete
nonzero products and normalized components that underflow are rejected;
individually tiny kinetic products may be rejected even if total E survives;
lost internal energy in stored E-K cannot be recovered; gamma near one or very
large gamma can exhaust representability. Reconstructed coefficients and
transfers introduce additional range requirements. Common-theta scaling can
reduce an unrepresentable reconstruction about an admissible mean; transfers,
stored updates and RK blends are never silently dropped or clipped.

Diagnostic counts include rejected attempts. limitedSlopeCells counts cell
preparations where MC changes any centered slope; positivityLimitedCells counts
theta<1 for either admissibility or range; rangeLimitedCells is the range
subset; zeroSlopeFallbackCells counts theta=0. minimumSlopeScale includes all
preparations. maximumSignalSpeedX/Y includes rejected prepared stages;
maximumCfl is the actual largest accepted physical C, maximumRejectedCfl the
largest rejected stage's C. Conserved summaries use stored rho,mx,my,E and
independent derived I/K. The roundoff guard is
`(64*N+288*S+128)*machine_epsilon` times the larger positive/absolute integral,
accounting for two FE passes and a blend per accepted step. This scale-aware
heuristic has no unit-size floor and is not a certified error interval.

## Independent validation and reproducibility

`run_tests "[euler2]"` compares raw physical MC/Rusanov/SSPRK2 face arithmetic
without production scaling to the stored result on anisotropic and two-cell
grids, then checks axis exchange, strict cold-flow positivity across scaled
states and gamma values, common-theta fallback, exact work/retry limits, late
rollback, snapshots, replay, historical observers, zero steps and range errors.
Separate unchanged `[euler]` regressions retain first-order targets. Smooth
contact/simple-wave cell-average comparisons use identical initial physical
inputs for both methods; limiter extrema and shocks are not claimed second
order locally. The exact Sod oracle splits conserved quadrature at waves;
same-dx doubled-domain comparison bounds numerical periodic-image effects.

Temporal tests separate spatial defects: a padded cubic contact has analytic
FV density offset `.5*b*u*dx^2*t` and SSPRK2 offset `b*u^3*sum(h^3)`. The test
reports both, guards a numerical stencil radius of four cells per accepted RK2
step inside the polynomial patch, and measures the expected factor-four
temporal refinement. A nonlinear simple wave compares against independently
implemented, further-refined semidiscrete RK4 while separately reporting its
continuum cell-average error. Fixed-grid temporal convergence is not treated
as continuum accuracy. The headless `euler_gas_second_order_demo` emits both
methods' contact JSON at identical resolutions and inputs. Tests, examples and
installed static/shared consumers are local validation, not hosted CI evidence.
Local optimized measurements with gamma=1.4 are:

| Independent comparison | Resolution or maximum h | Second-order error |
| --- | --- | --- |
| Identical 2D moving contact at t=.15, density RMS | 32, 64, 128, 256 columns | .0256336, .00439350, .000924034, .000235855 |
| Nonlinear simple wave at t=.2, conserved L1 | 32, 64, 128, 256 columns | .00158106, .000385865, .0000950629, .0000230875 |
| Exact Sod conserved L1, t=.12, length 2 | 128, 256, 512 columns | .0397738, .0229265, .0111596 |
| Exact two-rarefaction conserved L1, t=.1, length 4 | 128, 256, 512 columns | .111099, .0740745, .0408383 |
| Nonlinear temporal conserved L1, fixed 64 columns, t=.01 | .0025, .00125, .000625 | 1.45244e-6, 3.47818e-7, 8.87556e-8 |
| Padded cubic contact temporal density defect, t=.4 | .05, .025, .0125 | 1.00000e-10, 2.50006e-11, 6.25030e-12 |

The finest spatial ratios are 3.92 (contact) and 4.12 (simple wave). The fixed-grid
nonlinear temporal ratios are 4.18 and 3.92; its independently refined continuum
spatial error is 1.77241e-5. The cubic contact also reports a separate 5e-9 FV
spatial offset, rather than claiming the temporal defect is its continuum error.
MC limiting clips slopes at extrema and drops local order; the independent
extremum-face oracle checks that behavior explicitly. Riemann discontinuities
and fan endpoints do not have smooth second-order convergence. Both Riemann
same-dx doubled-domain controls match within 2e-13 per conserved component in
the comparison windows. The finest 2D contact requests 500000000 visits because
the default 100000000 budget is insufficient; smaller examples use the default.
These fixture-specific numbers are not universal error bounds. The 64x2 Sod
retry fixture with safety=.99 records 53 attempts, 29 accepted substeps, 24
rejections and exactly 86912 visits, including every rejected preparation.

```cpp
PeriodicEulerGasGrid gas({64, 32, 1.0/64, 1.0/32, 1.4});
EulerGasSecondOrderConfig options;
options.cflSafety = .8; // Physical directional CFL <= .4 at both stages.
options.maximumAttempts = 10000;
const auto record = gas.stepSecondOrder(.05, options);
const auto copied = gas.lastSecondOrderStep();
```

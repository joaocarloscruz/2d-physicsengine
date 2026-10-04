# WCSPH consistency diagnostic and next formulation step

This investigation does **not** resolve issue #44's nine expected-failing tests.
Their thresholds and `[!mayfail]` tags remain unchanged. It isolates the first
two failures from translation, temporal integration, initialization and wall
effects, using actual public kernels and the existing WCSPH solver.

## Reproduce

Build a Release configuration with `BUILD_TESTING=ON`, then run:

```sh
cmake --build build --target fluid_consistency_diagnostic --config Release --parallel 2
build/fluid_consistency_diagnostic --output build/fluid-consistency-full.json
build/fluid_consistency_diagnostic --quick --output build/fluid-consistency-quick.json
build/run_tests "[!mayfail]"
```

Use `.exe` on Windows and `build/Release/` with multi-configuration generators.
The quick mode retains the kernel/refinement/translation measurements and three
rest controls; full mode additionally integrates both spatial-refinement paths
and three outer timesteps. CI runs the quick diagnostic as a smoke check.

[Recorded full output](data/fluid-consistency-2d79c4e-clang23.json) was measured on
Windows with Clang 23.1.1, from source revision `2d79c4e` plus this diagnostic.
The six new operator/control tests contain 464 passing assertions. They
characterize operators and the already-supported initialized continuity mode;
they do not replace any of the nine failing physics targets.

## Discrete sums, rather than an incorrect continuum normalization

Both scalar kernels integrate to one in 2D, as the existing radial-integration
tests verify. Here `h` is the entire compact-support radius, so `h/dx=2` has
only nine nonzero lattice samples, **including self**.

For nominal particle mass `m=rho0*dx²`, the bulk density ratio is
`Cρ=dx² Σ Wρ(k*dx,h)`. With the poly6 weight at ratio two, only self, four
axis neighbors and four diagonal neighbors contribute:

`Cρ = [1 + 4*(3/4)^3 + 4*(1/2)^3] / pi = 3.1875/pi = 1.014612762`.

This is a quadrature defect. Translating the solver lattice by (0.037,-0.061)
changes measured bulk density by only 2.49e-8 at the coarse spacing. The
translation-invariance part is already satisfied; the one-percent bulk-density
target fails independently.

The scalar spiky weight whose derivative supplies the pressure gradient is a
different function from the poly6 density weight. Its discrete zeroth moment
is not the density sum used by the solver. For a linear scalar field, the
pressure-gradient first moment `Mx=-dx² Σ r_x*G_x(r,h)` should be one.

| h/dx | Nonzero samples | Poly6 density Cρ | Spiky scalar sum | Spiky gradient Mx | Poly6 derivative Mx |
|---:|---:|---:|---:|---:|---:|
| 1.5 | 9 | 0.957113 | 1.625356 | 0.654936 | 1.006016 |
| 2 | 9 | 1.014613 | 1.273641 | 0.886462 | 1.014613 |
| 2.5 | 21 | 0.993976 | 1.137314 | 0.924808 | 0.996936 |
| 4 | 45 | 1.000469 | 1.034168 | 0.985020 | 1.004821 |
| 8 | 193 | 1.000025 | 1.004272 | 0.997996 | 1.000326 |

The pressure response to a linear field is about 11.35% low at ratio two.
Applying the existing density mass calibration `1/Cρ=0.98559767` fixes its
density sum but changes this pressure-gradient moment to **0.87369490**.
Density calibration therefore does not calibrate the pressure operator.
The poly6 derivative column is an analytic comparison, not an implemented
alternative solver.

## Spatial and temporal controls

After substituting `r=dx*k`, every factor of physical length cancels from these
dimensionless moments at fixed `h/dx`. The actual-kernel test verifies this
for dx=0.1, 0.05, 0.025 and 0.0125. Refining both h and dx at ratio two
therefore preserves the 1.461% density bias and 0.886462 pressure moment.
There is no route to the one-percent target through that refinement alone.

The full diagnostic also holds h=0.2 while refining dx. This increases the
neighbor quadrature resolution and reduces both defects. This is a kernel
quadrature experiment, not proof of a complete continuum convergence limit:
a continuum study must eventually decrease h while increasing h/dx.
The density error also oscillates for some intermediate ratios; arbitrary
ratio changes should not be assumed to improve it monotonically.

For the canonical 21x21 zero-gravity block, rho0=1000, c=15, gamma=7,
viscosity=0.05 and nominal mass=10 per particle:

| Outer dt | Initial bulk rho/rho0 | Initial peak acceleration | Final peak speed at 0.1 s | Maximum displacement |
|---:|---:|---:|---:|---:|
| 1/240 | 1.01461272 | 17.5027 | 0.227214 | 0.0209863 |
| 1/480 | 1.01461272 | 17.5027 | 0.227830 | 0.0209486 |
| 1/960 | 1.01461272 | 17.5027 | 0.227833 | 0.0208269 |

The initial Tait pressure reaches 3435.75 despite zero gravity and zero
velocity. Interior pressure forces cancel on a complete lattice, while
truncated free-surface support and the pressure clamp produce outward forces
near the edge. Smaller timesteps do not remove this pressure source.
Pairwise forces still preserve total linear momentum to floating-point
accuracy; conservation of the block's momentum does not imply equilibrium.

The spatial integration matrix represents a fixed 2x2 cell footprint, with
400/1600/6400 particles and nominal total mass 4000 at every spacing.
Each run lasts 0.1 s with outer dt=1/960 and the same material/EOS settings.

| dx | h | Refinement path | Initial bulk rho/rho0 | Final peak speed | Maximum displacement |
|---:|---:|---|---:|---:|---:|
| 0.1 | 0.2 | Both paths' coarse case | 1.01461274 | 0.227819 | 0.0208285 |
| 0.05 | 0.1 | Fixed h/dx=2 | 1.01461270 | 0.227827 | 0.0218669 |
| 0.025 | 0.05 | Fixed h/dx=2 | 1.01461270 | 0.227218 | 0.0222937 |
| 0.05 | 0.2 | Fixed h, h/dx=4 | 1.00046894 | 0.0126831 | 0.00112903 |
| 0.025 | 0.2 | Fixed h, h/dx=8 | 1.00002515 | 0.000819255 | 0.0000701249 |

At fixed ratio, initial maximum acceleration doubles as h halves
(17.50/35.01/70.01), while peak speed remains about 0.227.
Increasing neighbor resolution removes most of this particular rest-state
error without altering continuum kernel normalization, masses or rho0.
It costs more neighbors and does not establish hydrostatic wall consistency.

## Existing opt-in controls and rejected shortcuts

Explicit `SquareLatticeMassScale(dx,h)` initialization lowers the canonical
block's speed to 4.71e-5 and displacement to 3.58e-6, but its total mass changes
from 4410 to 4346.49. This is a caller-selected different initial mass, not a
solver correction that preserves the nominal physical input. The diagnostic
checks that the solver subsequently preserves every mass and rest density.

The existing `WcsphDensityMode::Continuity`, initialized at rho0 with nominal
mass, produces exactly zero speed and displacement in this zero-gravity rest
control. This is supported initialization for a state variable, not evidence
that summation now reproduces that density or that disorder and walls are fixed.
The supported hydrostatic continuity initialization remains documented in the
[validation plan](fluid-validation-plan.md).

Do not make the default density kernel's normalization depend on lattice
spacing: that would change its physical unit integral and still miss
nonuniform or disordered quadrature. Do not silently alter rho0 to match the
biased sum. Likewise, a Shepard ratio with fixed volumes `m/rho0` would
return rho0 even under uniform compression because numerator and denominator
are proportional; this would suppress the EOS response rather than fix it.
Changing only the pressure gradient cannot eliminate a bias introduced
earlier by density summation.

## Bounded next implementation: an explicit matched kernel family

A normalized 2D cubic B-spline is a useful first **opt-in** family to implement
and evaluate, rather than replacing the current public kernels globally.
With full support radius h, conventional smoothing scale h/2, u=2r/h and
`C=40/(7*pi*h²)`, its weight is:

```text
W = C*(1 - 1.5*u² + 0.75*u³)       for 0 <= u < 1
W = C*0.25*(2-u)^3                for 1 <= u < 2
W = 0                            otherwise
```

The candidate in the diagnostic uses the analytic derivative of this same
weight. Unit-integral, finite-difference derivative and exact ratio-two
lattice-sum tests verify the comparison independently of any solver change.
It retains the same physical support radius and requires no mass calibration.

| h/dx | Cubic density zeroth moment | Matched cubic gradient first moment |
|---:|---:|---:|
| 1.5 | 1.049144 | 0.748498 |
| 2 | 1.000862 | 1.013099 |
| 2.5 | 0.999448 | 1.001046 |
| 3 | 1.003440 | 1.006725 |
| 4 | 0.999958 | 0.999100 |
| 8 | 1.000001 | 0.999849 |

At ratio two it reduces the nominal zeroth-moment defect from 1.461% to
0.0862%, and the first-moment defect from 11.35% to 1.310%. This is a
quantified design candidate; the diagnostic **does not integrate a cubic
solver**, so it supplies no claim that self-expansion or walls are solved.
Ratio 1.5 demonstrates that the improvement is not universal.

Add an explicit kernel-family setting with the current family as the default.
Route density, pressure gradient, continuity rate, boundary quadrature and
DFSPH coefficients through the selected weight/derivative pair; audit surface
operators separately rather than silently changing their meaning. Preserve
caller mass/rho0 and the unit integral. Retaining a radial gradient and a
symmetric scalar pair coefficient preserves central, equal/opposite pressure
forces, hence linear and angular momentum in exact arithmetic. That symmetry
does not guarantee exact local moments, hydrostatic support or energy balance.

Evaluate rest and hydrostatic acceptance experiments below before endorsing
the new mode. Include free-surface tension and negative-pressure/tensile
instability tests with the pressure clamp both enabled and disabled. A clamp
can remove attraction without supplying a mechanism to heal disorder.
Also vary neighbor count and measure pairing/clumping: increasing h/dx is not
an unlimited cure for cubic B-spline quadrature. The convergence/pairing
analysis of [Dehnen and Aly](https://arxiv.org/abs/1204.2471) motivates that
test and a future Wendland comparison; its three-dimensional analysis does
not establish a safe neighbor threshold for this 2D solver.

## Further formulation step: consistent divergence and its pressure adjoint

If matched-family residuals still miss the intended regime, a larger
formulation experiment should be **opt-in**, retain caller
masses/rho0 and continuum-normalized kernels, and build on initialized
continuity density. Add the analytic derivative of the same poly6 density
weight, then cache local first-moment matrices per prepared neighborhood.
Correct those gradients to reproduce linear fields, and derive pressure
forces from the discrete adjoint of that corrected continuity/divergence
operator. Include wall samples and their reaction forces in the same operator.

This is a design proposal, not code or a claim of validated hydrostatics.
It follows the consistency/conservation questions studied by
[Bonet and Lok](https://doi.org/10.1016/S0045-7825(99)00051-1).
The required conservation reasoning is explicit: a divergence operator that
annihilates uniform translation has an adjoint pressure force with zero net
internal force; if it also annihilates rigid rotation, its pressure work on
that rotation vanishes. Derive these identities with the actual mass/volume
weights, rather than inserting a correction into only one existing loop.

Applying each particle's inverse moment matrix independently to the current
pair force can lose equal/opposite force symmetry. Averaging the matrices
recovers that symmetry but generally loses exact local linear reproduction;
matrix-corrected forces need not remain central, so angular momentum is an
additional obligation. A full reproducing-kernel formulation also entails
such tradeoffs: [CRKSPH](https://arxiv.org/abs/1605.00725) reports exact mass,
linear momentum and energy conservation with approximate angular momentum
conservation. Neither paper is a drop-in implementation for this boundary
model.

Report ill-conditioned or rank-deficient moment matrices. A bounded fallback
must be visible in diagnostics and counted as unsupported by strict
consistency experiments, rather than silently treated as corrected.
Keep particle shifting and density diffusion as separate policies; a
zero-force block cannot heal positional disorder by itself.

Acceptance experiments for that future mode:

- Reuse unit-integral kernel tests and test zeroth/first moments on square,
  rotated, anisotropic, perturbed and wall-truncated neighborhoods. Check
  constant and linear fields, and report conditioning/fallback counts.
- Test uniform compression's density rate against the analytic continuity
  equation, plus translation and rigid rotation with zero density rate.
  Matching rho0 at rest must not hide compressibility.
- Check total linear/angular momentum and pressure-work balance with no
  external forces; perform dt, dt/2, dt/4 energy tests and verify viscous
  dissipation separately. Use an adjoint implementation, not assertions
  based only on equal forces in a two-particle case.
- Meet the unchanged rest thresholds (speed <0.05, displacement <0.005,
  momentum <1e-3) with nominal masses at three spacings, alongside kernel
  neighbor refinement. Verify user mass/rho0 are unchanged.
- Initialize hydrostatics through the EOS, include wall reaction, and measure
  initial pressure RMSE <0.1 and acceleration RMS <1.0. Then require the
  existing speed/density targets under both dt and spatial refinement,
  including flat/corner walls and free surfaces.
- Retain disorder, ramp, coupled momentum and DFSPH controls separately.
  Remove a failure tag only after its own intended regime is supported.

## Nine failures retained in the measured revision

Running `run_tests "[!mayfail]"` still reports **9 cases failed as expected**.
These are measurements at this revision, not new supported tolerances.

| Unresolved target | Measured result |
|---|---:|
| Nominal lattice interior density | 1.46127% bias |
| Nominal rest block | 0.227214 speed; 0.0209863 displacement |
| Disorder healing | RMS 0.00282841 unchanged; speed zero |
| Hydrostatic initialization | pressure RMSE 0.548368; acceleration RMS 9.80979 |
| Dummy-wall weight balance | vertical residual 0.632665 |
| Dummy-wall spatial force convergence | residuals 0.0175504 / 0.165006 / 0.235835 |
| Summation sampled-wall hydrostatics | bulk density error 6.60735%; speed 1.70497 |
| Calibrated sampled-wall hydrostatics | density error 27.8468%; speed 2.168 |
| Gravity ramp | peak speed 45.9111 vs immediate 16.0798 |

The hydrostatic errors are not reduced to a single kernel prefactor by this
diagnostic. Initial pressure support, boundary operators and density evolution
remain separate work. DFSPH free-surface performance does not validate WCSPH
container hydrostatics.

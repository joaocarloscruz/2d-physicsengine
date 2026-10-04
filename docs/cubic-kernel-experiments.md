# Matched cubic kernel: tested regime and remaining limits

Issue #62 adds an **opt-in** matched cubic family to native WCSPH and DFSPH.
Legacy poly6/spiky dynamics remain the default. Nominal-mass square lattices
at h/dx=2 now meet the original density and rest acceptance thresholds in the
cubic mode. This does **not** resolve issue #44 as a whole: all nine legacy
expected-failing tests and thresholds remain unchanged, and sampled-wall
startup, disorder and unclamped tensile/clumping controls remain unsupported.

```cpp
WcsphConfig config;
config.kernelFamily = SphKernelFamily::CubicSpline;
WcsphSolver solver(h, config);
// DfsphConfig exposes the same opt-in selector.
```

The family routes density weights, pressure gradients, continuity/divergence,
DFSPH coefficients and diagnostics through one normalized weight/derivative
pair. WCSPH wall density, pressure extrapolation and mirror density rates use
that same pair. The full support radius remains h, with standard cubic
smoothing scale h/2. Masses and rest densities remain caller inputs. The
separate Muller viscosity operator and #57 diffusion bounds remain active.
Changing the computed density can change viscosity coefficients and timestep
limits even though the viscosity kernel is unchanged.

Existing two-argument kernel calls and the two-argument free diagnostic call
retain legacy behavior. The family-aware diagnostic overload can receive
prepared wall rates; WCSPH now includes those rates in raw divergence metrics
for **both** families. Density diffusion is excluded from that measurement.
This diagnostic correction does not change legacy particle dynamics. The
public config structs gained a field, so native binary consumers must rebuild.
WCSPH uses each particle's support for density summation. Its matched cubic
summation pressure now weights each particle's own density gradient, as derived
in [fixed-support pressure](sph-fixed-support-pressure.md). Continuity pressure,
the legacy family and the comparison diagnostics retain mean-support gradients;
DFSPH uses mean support for both pair operators. Supplied smoothing lengths
are fixed; density-adaptive supports and grad-h terms are not implemented.
DFSPH still has no boundary solver. The separate fluid-rigid surface coupler
retains its existing model; selection does not redesign surface traction.

## Reproduce

Configure Release with BUILD_TESTING=ON, then run:

```sh
cmake --build build --parallel 2
build/run_tests "[sph][kernel]"
build/run_tests "[family]"
build/run_tests "[dfsph]"
build/run_tests "[!mayfail]"
build/fluid_consistency_diagnostic --family cubic --output cubic.json
build/fluid_consistency_diagnostic --family legacy --output legacy.json
ctest --test-dir build --output-on-failure
```

Use `.exe` on Windows and the configuration directory with multi-configuration
generators. `--quick` omits full spatial/timestep integration and the slower
perturbation/tensile/wall controls. Both family quick modes run in CTest/CI.
[Recorded cubic output](data/fluid-cubic-8d2663a-clang23.json) and
[legacy control](data/fluid-legacy-8d2663a-clang23.json) were measured with
Windows Clang 23.1.1 Release from base 8d2663a plus the kernel, solver and
experiment changes identified in each report. Every reported number is finite.
All 11 rest controls preserve each caller mass and rest density exactly.

## Density and rest results with nominal mass

The production cubic lattice sum agrees with the independently implemented
candidate formula. At h/dx=2, the density zeroth moment is 1.00086183 and the
matched gradient first moment is 1.01309945. Both retain small quadrature
errors; the gradient is not exactly linearly reproducing. The legacy moments
are 1.01461276 and 0.88646202. Physical unit-integral normalization is retained;
there is no lattice-dependent renormalization in the solver.

For the original 21x21 nominal block, dx=.1, h=.2, rho0=1000, mass=10,
c=15, gamma=7, viscosity=.05, zero gravity, clamped negative pressure and .1 s:

| Outer dt | Cubic final maximum speed | Cubic maximum displacement | Cubic total momentum magnitude |
|---:|---:|---:|---:|
| 1/240 | 0.0141077 | 0.00131239 | 2.01e-6 |
| 1/480 | 0.0141412 | 0.00131345 | 2.54e-7 |
| 1/960 | 0.0141577 | 0.00130534 | 4.42e-6 |

These meet unchanged speed <.05, displacement <.005 and momentum <1e-3.
The legacy 1/240 result remains .227214 speed and .0209863 displacement.
Cubic initial bulk density is 1.00086172 rho0, with peak initial pressure
194.59 and acceleration 1.16424, compared with legacy 3435.75 and 17.5027.
Lower pressure bias explains the improvement; the result is not exact rest.
Tests repeat the unchanged thresholds at dx=.1/.05/.025 and outer dt
1/240,1/480,1/960, also checking angular momentum <1e-3 and unchanged inputs.

The full diagnostic uses a fixed 2x2 represented footprint and total mass
4000, .1 s duration and outer dt=1/960 for spatial refinement:

| dx | h | Path | Cubic final maximum speed | Maximum displacement |
|---:|---:|---|---:|---:|
| .1 | .2 | Both coarse paths | .0141392 | .00130169 |
| .05 | .1 | Fixed h/dx=2 | .0141535 | .00136290 |
| .025 | .05 | Fixed h/dx=2 | .0141360 | .00139415 |
| .05 | .2 | Neighbor refinement, h/dx=4 | 0 | 0 |
| .025 | .2 | Neighbor refinement, h/dx=8 | .0000708935 | .00000894070 |

Fixed h/dx retains the small scale-invariant defect; meeting a tolerance is
not proof that this refinement converges to exact consistency. At ratio four,
initial density is slightly below rho0 (.99995756), so the pressure clamp
makes this rest state force-free. That zero is not evidence of an exact
quadrature operator. Density error is nonmonotonic at intermediate ratios;
ratio 1.5 remains a poor cubic choice (about 4.91% density error).

Explicit family-aware lattice mass calibration lowers the canonical cubic
speed to 4.33e-5 but changes total mass from 4410 to about 4406.20. It remains
a caller-selected different initial mass. Initialized continuity density at
rho0 retains exact zero-gravity rest with nominal masses in both families.
Neither control establishes summation consistency on irregular particles.

## Compression and conservation controls

A two-percent isotropic position compression increases cubic interior density
by more than two percent and raises Tait pressure with mass/rho0 unchanged.
Continuity rates and diagnostics match the independent analytic cubic gradient;
uniform translation has zero density rate. Analytic one-wall tests exercise
wall density, extrapolated pressure force and mirror compression rate.

Unequal-mass pressure controls check total linear and angular momentum for
both solvers. Because the pressure gradient remains radial and the scalar
pair coefficient symmetric, equal/opposite central pressure forces retain
those conservation properties in exact arithmetic. Viscous dissipation and
strong-viscosity substepping controls pass with the independent Muller bound;
these checks do not claim angular conservation for viscous tangential forces.
The existing four DFSPH projection controls run with both families, including
compression/divergence reduction, tolerance response, repeated finite state
and momentum, and iteration-budget reporting. Calibrated initial masses in
those controls use the selected family explicitly. Coupled tank buoyancy
ordering also passes for both families. Energy conservation and a consistent
surface-traction discretization remain separate formulation obligations.

## Perturbation, neighbor count and tensile limits

The full benchmark starts 21x21 nominal lattices at rest and applies an
alternating +/-10% dx offset in each coordinate when perturbation is enabled.
It records .05 s at outer dt=1/960, zero gravity, c=15, viscosity=.05. Minimum
pair separation and positional RMS relative to the original lattice are
characterizations, not newly supported tolerances.

| h/dx | Perturbed | Negative pressure clamped | Initial -> final minimum separation / dx | Peak speed |
|---:|---|---|---:|---:|
| 2 | No | Yes | 1 -> .999835 | .0141577 |
| 2 | Yes | Yes | .8 -> .628179 | 1.59451 |
| 2 | No | No | 1 -> .620440 | 3.82191 |
| 2 | Yes | No | .8 -> .109343 | 5.48921 |
| 4 | Yes | Yes | .8 -> .8 | 0 |
| 4 | Yes | No | .8 -> .0757481 | 9.58867 |
| 8 | Yes | Yes | .8 -> .8 | 0 |
| 8 | Yes | No | .8 -> .0247988 | 12.1615 |

At ratio two with the clamp, perturbed positional RMS grows from .0141421 to
.0235764. Increasing neighbor count with the clamp can instead leave the
perturbation motionless; neither behavior heals disorder. Allowing negative
pressure causes severe close-pair/clumping behavior, including on regular
free-surface blocks. This finite-patch test combines surface tensile effects
and perturbed neighborhoods; it does not identify a bulk Fourier pairing
threshold or establish that increasing h/dx is safe. These limitations are
consistent with the need for explicit pairing studies motivated by
[Dehnen and Aly](https://arxiv.org/abs/1204.2471), whose three-dimensional
analysis does not give a safe neighbor threshold for this 2D solver.

## Sampled-wall startup remains unsupported

The diagnostic also runs the original-style 19x15 column in a 2x2 sampled
polygon tank for .1 s, dt=.001, h=.2, dx=.1, c=40, viscosity=.05 and gravity
-9.81, with nominal mass and summation density. It is **not initialized through
the hydrostatic EOS**. Cubic peak speed is .767190, versus 1.48479 for this
legacy nominal control; cubic final bulk density error is 1.04090%.
The speed still exceeds the unchanged quiet-column target .2. Final bulk
error is not the existing test's maximum error across the whole trajectory.
Wall force balance, initialized hydrostatic pressure/acceleration, calibrated
walls, ramping and spatial wall convergence remain separate acceptance work.
Do not remove any #44 failure tag based on this shortened startup control.

A matched family reduces a specific lattice defect while preserving central
pair pressure forces. Exact local field consistency would require a larger
corrected divergence/pressure-adjoint design, with boundary reactions,
conditioning diagnostics, compression response, momentum and work-balance
validation as described in the [original formulation plan](fluid-consistency-diagnostic.md).

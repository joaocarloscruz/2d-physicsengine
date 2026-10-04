# Sampled-wall force and pressure-work audit

Issue #69 adds diagnostics and analytical tests only. **No production solver,
wall sampler, kernel, mass or rest-density policy is changed.** All nine #44
expected-failing tests retain their tags and thresholds. An initialized
snapshot with accurate pressure or small total force is not a quiet-column
integration result.

## Reproduce and scope

```sh
cmake --build build --parallel 2
build/run_tests "[wall-audit]"
build/fluid_wall_diagnostic --quick --output wall-quick.json
build/fluid_wall_diagnostic --output wall-full.json
build/run_tests "[!mayfail]"
ctest --test-dir build --output-on-failure
```

Use `.exe` on Windows and the configuration directory with multi-configuration
generators. Both modes measure flat-bottom, bottom/left-corner and closed-tank
stencils, both kernel families and two distinct initializations. Full mode
contains 84 prepared states; quick mode contains 12. Both include 36 bounded
constant-pressure phase probes. There is no timestep integration. The
[recorded full report](data/fluid-wall-0839bb5-clang23.json) uses base 0839bb5,
Windows Clang 23.1.1 Release plus this diagnostic. The JSON stores one scene
per line to keep the full matrix reviewable. Every numeric value is finite.

Fluid particles use nominal mass rho0*dx², rho0=1000, c=40, gamma=7, gravity
(0,-9.81), zero viscosity/density diffusion and pressureScale=1. Coordinates
are x=-1+dx,...,1-dx and y=dx,...,1.5. Tank walls are at x=+-1 and y=0,2;
flat/corner controls omit the other walls to isolate stencils. Their omitted
lateral supports create open-edge acceleration, so all-particle RMS in those
controls is **not** a hydrostatic acceptance criterion. Bottom-interior and
bottom-left-region RMS localize the supported wall and corner regions.

The benchmark reconstructs prepared pressure forces and raw continuity rates
in double precision, using the actual selected kernels but separate loops.
The maximum relative force disagreement with `prepare()` is 2.77e-6 across
the full matrix. A mismatch above 2e-5 causes the diagnostic to fail; this is
an operator-reconstruction tolerance, not a relaxed physical target.
The audit bounds inputs to 5000 fluid/10000 wall samples. Scene dimensions and
probe ratios are fixed/bounded; the executable accepts no arbitrary scene size.

## Initialization is a separate control

Uninitialized summation starts at rho0 but `prepare()` replaces density with
its fluid/wall kernel sum. It does not create the intended hydrostatic pressure.
Its bulk acceleration remains about gravity even as neighbor quadrature improves.

The initialized control uses continuity density and the exact static Tait
profile, with z=max(surfaceHeight-y,0), surfaceHeight=1.5:

```text
R = [1 + (gamma-1)*g*z/c²]^(1/(gamma-1))
rho = rho0*R
p = rho0*c²/gamma * (R^gamma - 1)
```

This satisfies dp/dz=rho*g, unlike an exactly linear incompressible pressure
profile at finite compressibility. Density is an explicitly initialized state
variable. Masses and positions remain nominal/uniform; the represented volume
m/rho therefore varies with depth. There is no silent lattice mass calibration
or material-row compression. This distinction must remain visible in later
hydrostatic comparisons.

Prepared initialized pressure RMSE, normalized by rho0*g*1.5, is about 5e-6
for both families. Summation pressure RMSE at coarse h/dx=2.5 is .558 legacy
and .557 cubic. Initializing pressure correctly does not establish force balance.

## What the radial mirror actually implements

For fluid i and sampled boundary b, let r=x_i-x_b, G=grad W(r), V_i=m_i/rho_i,
V_b be sample volume, u=v_i-v_b, s=pressureScale and p_b the weighted
extrapolated pressure. There is **no normal or surface identifier** in
`FluidBoundaryParticle`. Its displacement direction e=r/|r| is not a plane normal.
The current terms for s>0 are:

```text
F_i,b       = -V_i*V_b*s*(p_i+p_b)*G
rhoDot_i,b  = 2*rho_i*V_b*(u dot e)*e dot G
             = 2*rho_i*V_b*(u dot G)     because G is radial
```

The factor two mirrors relative velocity along each individual sample ray.
It does not store or implement a geometric plane-normal reflection. An oblique
sample can therefore give nonzero density rate for motion tangent to y=0.
The independent oracle at r=(.3,.4), u=(.1,0), h=1, V_b=.2 and rho=1000
matches the measured nonzero rate for both kernels, while a plane reflection
would give zero. **Symmetric oblique samples can cancel that response**;
a separate test verifies exact aggregate tangent-rate and tangent-force
cancellation. An individual ray is not proof that a complete flat stencil fails.

Joint translation of fluid and wall gives exact zero rate throughout the
matrix. Joint rigid rotation gives maximum rates about 3e-8/s, because u is
perpendicular to r. A future plane-normal correction must retain this property;
inserting normals alone is not enough if velocities/reactions are evaluated
at inconsistent surface points.

## Pressure work and inferred reaction

For a fluid pair, raw continuity and central pressure forces obey:

```text
F_ij dot (v_i-v_j) + m_i*p_i/rho_i²*rhoDot_i
                         + m_j*p_j/rho_j²*rhoDot_j = 0
```

The largest pair-work residual in the measured matrix is 6.2e-10. These are
instantaneous operator identities, not finite-timestep total-energy guarantees.

For a wall, the report labels R_b=-F_i,b as an **inferred** reaction. No callback
applies this reaction to the tank or rigid coupler. Including its mechanical
work, the thermodynamic pressure-work identity is:

```text
wall mechanical work = F_i,b dot v_i + R_b dot v_b = F_i,b dot u
fluid internal rate  = m_i*p_i/rho_i²*rhoDot_i,b = 2*V_i*V_b*p_i*(u dot G)
residual             = V_i*V_b*[2*p_i-s*(p_i+p_b)]*(u dot G)
```

At s=1,p_b=p_i it closes exactly; moving-wall tests require the inferred
reaction work to obtain that closure. Gravity extrapolation generally makes
p_b differ from p_i. A virtual boundary reservoir could account for the
remaining power; its energy is not tracked by this model. The residual is a
model accounting obligation, **not by itself proof of instability**. Gravity
work is reported separately as external work. The existing pressureScale also
scales force without scaling the mirror rate, giving an additional residual
for s!=1; the analytical test exposes it rather than assuming full adjointness.

At coarse initialized h/dx=2.5 under isotropic compression v=-.1*x:

| Family | Pair mechanical / internal power | Wall mechanical / fluid internal power | Wall accounting residual | Bulk rhoDot/rho |
|---|---:|---:|---:|---:|
| Legacy | -3254.95 / +3254.95 | +1380.86 / -1400.90 | -20.0314 | .184126 |
| Cubic | -3516.02 / +3516.02 | +1530.24 / -1556.77 | -26.5271 | .199305 |

The continuum compression rate is .2/s. Different quadrature/mass-to-density
volumes explain why the measured rate is not exactly that value. The report
also includes rest, tangent motion against fixed walls, joint translation,
normal motion against fixed walls and joint rigid rotation.

## Good total force can hide local wall errors

For the initialized tank at fixed h/dx=2.5:

| Family | dx | Vertical force residual / weight | Bulk acceleration RMS | Bottom-interior RMS | Corner-region RMS | All-particle RMS |
|---|---:|---:|---:|---:|---:|---:|
| Legacy | .1 | .106972 | .859936 | .755895 | 4.99954 | 1.81971 |
| Legacy | .05 | .095677 | .860423 | .627731 | 10.3885 | 1.55398 |
| Legacy | .025 | .090094 | .860723 | .671464 | 21.2711 | 1.40805 |
| Cubic | .1 | .006489 | .130386 | 2.90344 | 9.36199 | 2.41757 |
| Cubic | .05 | .006083 | .135446 | 6.05512 | 19.0995 | 2.63822 |
| Cubic | .025 | .012309 | .138279 | 12.3726 | 38.5947 | 3.22334 |

Coarse cubic summed wall force is (-.0149,27777.1), gravity is
(0,-27958.5), and total prepared fluid force is (-.0149,-181.424).
Fluid-pair forces cancel globally; inferred wall reaction has the opposite
sign to summed wall force. That small global residual hides large local
bottom/corner acceleration. Neither family converges uniformly under fixed
neighbor-ratio refinement.

Holding h=.25 while reducing dx to .05/.025 improves bulk gradients: cubic
bulk RMS is .1235/.1401 and legacy .2125/.1490. It does not remove every local
wall/corner defect; cubic all-particle RMS is 2.1057/3.2393. Fixed-dx h=.2/.4
controls vary the ratio separately; see the full matrix. No old hydrostatic
threshold is removed based on these snapshots.

## Constant-pressure wall quadrature isolates a cause

Polygon sampling places points at tangential edge midpoints, spaced dx, with
normal layers at 0,-dx,... . Fluid lattice points have a different tangential
phase. The constant-pressure probe sets rho=1010 and **explicit caller mass**
rho*dx², with c=20, gamma=7 and zero gravity/viscosity/diffusion, so fluid and
wall volumes both equal dx². It uses a sufficiently wide
half-space patch to exclude free edges from the measured bottom center.
This probe is separate from the nominal-mass hydrostatic matrix.

Aligned wall samples reproduce the missing lattice rows and cancel the
constant-pressure stencil to float precision. With a half-spacing tangential
shift, h/dx=2.5 has a scale-invariant defect F_y/(p*dx):

| Family | Phase/dx | Normalized constant-pressure force | Acceleration at dx=.1/.05/.025 |
|---|---:|---:|---:|
| Legacy | 0 | about 2e-7 | roundoff only |
| Legacy | .5 | -.00498201 | -.203326 / -.406652 / -.813305 |
| Cubic | 0 | about 4e-8 | roundoff only |
| Cubic | .5 | +.0223313 | +.911387 / +1.82277 / +3.64555 |

The pressure is 4122.02 in this probe. Acceleration scales as p/(rho*dx)
times the dimensionless defect, so fixed h/dx refinement doubles it as dx
halves. This explains a local force-error mechanism that pressure initialization
alone cannot remove. Aligning a wall to one square lattice is an oracle,
**not a portable correction for arbitrary fluid phase, disorder or curved walls**.

## Two counterfactuals and a bounded next implementation

The report evaluates alternatives without applying either to the solver:

- Exact local-fluid adjoint of the current radial mirror uses
  F=-2*V_i*V_b*p_i*G, omitting p_b. It closes that local pressure-work identity,
  but coarse initialized cubic weight residual worsens from .006489 to .069386
  (legacy .106972 to .163061). Dropping p_b is not an established improvement.
- Signed gravity extrapolation uses weighted p_i-rho_i*(g-a_b) dot r, then
  applies the configured negative-pressure clamp to the resulting wall pressure.
  The current max(0,-(g-a_b) dot r) clips the negative increment required by an
  affine hydrostatic field for some side-wall neighbors. An independent oracle
  demonstrates this loss. The signed counterfactual changes coarse cubic all
  RMS from 2.41757 to 2.27455, with weight residual .006354. It does not remove
  the phase defect and can worsen acceleration at h/dx=2.

The recommended next correction is an **opt-in planar boundary quadrature
prototype**, with geometric surface identity and a shared force/rate operator,
not a global pressure prefactor. First test fluid centers at dx/2 from a plane
with reflected volume quadrature centered at -(layer+.5)*dx and matching
source-fluid tangential phase. Assign wall reaction to the actual surface
projection, and evaluate moving-wall velocity at that same point. Specify
corner ownership/overlap before extending to multiple surfaces. This changes
wall quadrature and effective boundary placement; it must not silently alter
caller particle masses, rho0 or continuum kernel normalization.

For irregular fluids, reflected samples must retain their source ownership;
a velocity ghost depends on that owner's velocity and the wall motion. Assemble
that linear continuity operator and derive its pressure adjoint, including
source-fluid and wall reaction terms. Copying a new ghost density into the old
ray-mirror loop is insufficient. If extrapolated ghost pressure is retained,
state and report the virtual-reservoir pressure-work exchange explicitly.
Central legacy ray forces conserve angular momentum with inferred opposite
reaction at the sample point; a plane-normal force can lose that identity if
reaction point and velocity mapping are inconsistent. These are design
obligations, not solved features in this patch.

A separate small opt-in signed-extrapolation experiment is justified by the
affine-field oracle, but must retain reconstruction tests and report its work
exchange; it must not be presented as the wall-balance solution.

Acceptance for a later solver implementation must include constant-pressure
phase/disorder invariance, flat-wall zero tangent flux, joint translation and
rotation, fixed/moving-wall pressure-work balance with explicit reactions,
correct compressive response, EOS-initialized local acceleration/pressure and
global weight balance, flat/corner/curved sampling and spatial/neighbor/dt
refinement. Retain every #44 target until its own intended regime is supported.

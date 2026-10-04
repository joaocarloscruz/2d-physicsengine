# Periodic plane-strain elastic P and S waves

`PhysicsEngine::ElasticWaveGrid` is a separately owned, homogeneous, isotropic,
small-strain linear elastic continuum on a rectangular periodic domain. It stores
velocity and stress in `double`, independent of `World`, rigid contacts, the scalar
membrane and mass-spring bodies. It does not track displacement, deformation or
material interfaces, and has no forcing, damping, free boundary, fracture or
automatic coupling. Its state is at one common physical time after each step.

## Physical model and units

With velocity `v`, in-plane symmetric stress `sigma`, density `rho`, Lamé modulus
`lambda` and shear modulus `mu`, the continuum equations are

```text
rho * v_dot = div(sigma)
sigmaXX_dot = (lambda + 2*mu)*vx_x + lambda*vy_y
sigmaYY_dot = lambda*vx_x + (lambda + 2*mu)*vy_y
sigmaXY_dot = mu*(vx_y + vy_x).
```

Plane strain means `epsilonZZ = epsilonXZ = epsilonYZ = 0`, not `sigmaZZ = 0`.
The derived out-of-plane stress is

```text
sigmaZZ = lambda*(epsilonXX + epsilonYY)
        = lambda/(2*(lambda + mu))*(sigmaXX + sigmaYY).
```

In SI, spacings are metres, time seconds, velocity m/s, stress and Lamé moduli Pa,
and density kg/m^3. Integrals and energies are **per unit out-of-plane depth**
(J/m). Density and `mu` must be positive; `lambda` may be negative for a stable
auxetic material, subject to the underlying three-dimensional isotropic condition
`lambda + 2*mu/3 > 0`. The compressional and shear speeds are

```text
cp = sqrt((lambda + 2*mu)/rho)
cs = sqrt(mu/rho).
```

The implementation computes `b = lambda + mu` and then `b + mu` to avoid the
otherwise unnecessary overflow of `2*mu` in supported large auxetic materials.
It does not rescale or calibrate physical density, moduli, fields or energy.
Other required derived coefficients must still be representable; finite input
alone is not a promise that all operations can be performed in float64.

## Staggered spatial operators

Each of the five arrays has `columns*rows` entries, indexed by `i + columns*j`.
Periodic samples are stored once, without a duplicate end face or corner.

| Field | Position |
| --- | --- |
| `vx` | `(i*dx, (j+.5)*dy)` |
| `vy` | `((i+.5)*dx, j*dy)` |
| `sigmaXX`, `sigmaYY` | `((i+.5)*dx, (j+.5)*dy)` |
| `sigmaXY` | `(i*dx, j*dy)` |

Define `Dx+ f = (f[i+1,j]-f[i,j])/dx` and
`Dx- f = (f[i,j]-f[i-1,j])/dx`, and similarly in y. Every neighbor index wraps
periodically. Two cells on an axis are supported: its two physical staggered
faces remain distinct; the forward and backward periodic differences are not
replaced by a centered derivative that would vanish on that axis.

Let `E` be the engineering strain-rate operator:

```text
E v = (Dx+ vx, Dy+ vy, Dy- vx + Dx- vy)
    = (epsilonXX_dot, epsilonYY_dot, gamma_dot), gamma = 2*epsilonXY.
```

Let `D` be the stress-divergence operator:

```text
D sigma = (Dx- sigmaXX + Dy+ sigmaXY,
           Dx+ sigmaXY + Dy- sigmaYY).
```

Periodic summation by parts gives `(Dx+)* = -Dx-`, and hence `D = -E*` under
the common cell-volume inner product `dx*dy*sum`. The engineering shear pairing
is `sigmaXY*gamma_dot`, which is the physical double contribution of off-diagonal
stress. Constitutive matrix `C` is

```text
[lambda+2*mu, lambda,      0]
[lambda,      lambda+2*mu, 0]
[0,           0,          mu].
```

Thus `sigma_dot = C E v` and `v_dot = -E* sigma/rho`. The test suite assembles
an independent dense incidence matrix, checks both operators entry by entry,
and checks their power cancellation without advancing the time integrator.

## Time stepping, spectrum and energy

One substep of length `h` is the symmetric map

```text
sigmaHalf = sigma + (h/2)*C E v
vNext     = v + h*D sigmaHalf/rho
sigmaNext = sigmaHalf + (h/2)*C E vNext.
```

For a physical-phase staggered Fourier mode with integer mode numbers `(mx,my)`,

```text
kappaX = 2*sin(pi*mx/columns)/dx
kappaY = 2*sin(pi*my/rows)/dy.
```

Longitudinal and transverse velocity polarizations are along `kappa` and
perpendicular to it, with semidiscrete frequencies `omegaP = cp*|kappa|` and
`omegaS = cs*|kappa|`. The map frequency is
`theta/h = 2*asin(h*omega/2)/h`. Its strict stable range is `h*omega < 2`.
A grid-independent-of-mode sufficient condition is

```text
h * cp * hypot(1/dx, 1/dy) < 1.
```

`getStableTimeStep()` returns the smaller of `maxSubstep` and a value rounded
inward from `cflSafety/(cp*hypot(1/dx,1/dy))`, with `0 < cflSafety < 1`.
`step(dt)` uses a bounded number of equal substeps at or below this limit.
No wave speed is silently clipped to satisfy CFL.

Writing `V = dx*dy` and `b = lambda+mu`, physical energy is

```text
H = V*sum[rho*(vx^2+vy^2)/2
          +(sigmaXX+sigmaYY)^2/(8*b)
          +(sigmaXX-sigmaYY)^2/(8*mu)
          +sigmaXY^2/(2*mu)].
```

For a **fixed substep h**, the symmetric map conserves the quadratic form

```text
Hh = H - V*h^2/8 * sum[b*(epsilonXX_dot+epsilonYY_dot)^2
                       +mu*(epsilonXX_dot-epsilonYY_dot)^2
                       +mu*gamma_dot^2].
```

This follows by diagonalizing the negative-adjoint constitutive operator into
oscillators; the stress-kick/velocity-drift/stress-kick map conserves the oscillator
form with its velocity weight reduced by `1-h^2*omega^2/4`. No assumption that
initial stress is displacement-compatible is required: uncoupled stress modes
remain static. With `eta = h*cp*hypot(1/dx,1/dy) < 1`,

```text
Hh <= H <= Hh/(1-eta^2).
```

`getModifiedEnergy(h)` evaluates the reference invariant before stepping.
`modifiedEnergyStep` identifies the `h` used by the last successful step;
`physicalEnergyUpperBound` evaluates the envelope using that same reference.
This is a **conserved-reference bound for fixed h**, with floating-point roundoff,
not a guarantee of one conserved invariant or bounded drift when h changes
between calls. Physical energy generally oscillates even for fixed h. Zero
reference step evaluates physical energy. Positive reference steps must satisfy
the strict physical CFL condition, though they need not satisfy the configured
safety margin or `maxSubstep`.

Norms are accumulated after applying physical square-root weights with `hypot`,
so a representable aggregate energy can be supported even when each particle-like
sample's squared energy would underflow. Nonzero total kinetic/strain/correction
norms whose energies round to zero, and overflowing energies, are rejected.
No arbitrary energy unit floor or tolerance clamp is used.

## Compatibility and represented means

The in-plane strain inferred from stress is

```text
a = (sigmaXX+sigmaYY)/(4*(lambda+mu))
d = (sigmaXX-sigmaYY)/(4*mu)
epsilonXX = a+d, epsilonYY = a-d, gamma = sigmaXY/mu.
```

The measured local Saint-Venant defect is

```text
Ly epsilonXX + Lx epsilonYY - Dx+ Dy+ gamma,
Lx = Dx+ Dx-, Ly = Dy+ Dy-.
```

Both mixed derivatives map corner shear to normal-stress cells. These commuting
periodic differences annihilate strain increments `E v`, so the defect is an
observed invariant (up to roundoff). It has units 1/length^2. A zero defect does
not certify that arbitrary stresses arise from periodic displacements: strain
means are additional global conditions. The API accepts nonzero means and
arbitrary supported initial prestress, reports the defect, and does not project
or assert compatibility. Uniform translation and uniform stress are exact DC
modes of the represented update.

Periodic differences also preserve the two velocity means and three stress
means, up to update roundoff. Diagnostics use scaled Neumaier compensation
within the existing measurement sweep rather than summing `field/N`; constant
accepted fields report their stored value exactly, including subnormals. The
same mean treatment applies to derived `sigmaZZ`. Mixed dynamic ranges whose
nonzero contributions disappear during normalization/rescaling, or whose final
nonzero mean rounds to zero, are rejected. This is an explicit float64 range
restriction, not an arbitrary-precision summation promise. Derived local strains,
`sigmaZZ` and compatibility still have normal floating-point rounding.

## Ownership, budgets and transactional errors

Geometry, material and budgets are immutable. `getConfig()`, `getState()`,
`getDiagnostics()`, `getSpatialRates()`, `getCompatibility()` and
`getOutOfPlaneStress()` return owning copies. Editing them never changes the grid.
Spatial rates expose acceleration and engineering strain rate without updating
state and are useful for independent operator checks.

Both axes require at least two cells; the hard cell cap is 262144. Configuration
checks dimensions/count products before allocating arrays. The configurable
substep cap defaults to 10000 (hard cap 1000000); the cell-visit cap defaults to
100000000 (hard cap 1000000000). For `N` cells and `s` substeps the reported
update/measurement work is `N*(3*s+1)` center visits: three sweeps per substep and
one final diagnostic sweep. The fixed-cost stencils within a visit, initial
copies/input validation, constructor/state-replacement diagnostics and explicit
read-only queries are outside this step counter. Counts and budgets are checked
before scratch allocation or iteration, including an inward-limit rounding case
that can require an extra substep.

`setState()` requires five finite, exactly sized arrays. It stages and measures
the candidate before publishing, retains the clock, and resets last-step work and
energy-reference diagnostics. A successful zero `step(0)` is a complete no-op,
including prior diagnostics. Negative/nonfinite steps are invalid. A positive
step must have representable coefficients, a representable clock increment and
bounded work. Failure preserves **all five fields, clock and prior diagnostics**,
including failures after partial scratch updates. There are no external callbacks
or partial published substeps.

The implementation checks required reciprocal, volume, domain extent,
material/derivative and energy coefficients, wave rates and reference envelopes.
It stages the Lamé and step factors where this retains valid results. Remaining
float64 range limitations are deliberate: for example a raw difference of
opposite huge stresses can overflow even if multiplication by a tiny derivative
factor could have made the exact result finite. Such calls fail transactionally.
These checks do not claim universal exact arithmetic for all finite inputs.

## Native use and reproduction

Include `physics/core/elastic_wave_grid.h` or the public `physics/physics.h`
umbrella and link `PhysicsEngine::Engine`. The owned synchronous
[WASM adapter](webassembly.md#owned-plane-strain-elastic-waves) exposes the same
physical state and copied observations to JavaScript.

```cpp
PhysicsEngine::ElasticWaveGridConfig config;
config.columns = 32;
config.rows = 24;
config.spacingX = .0625;
config.spacingY = .125;
config.density = 2;
config.lambda = 3;
config.shearModulus = 2;
config.maxSubstep = .01;
PhysicsEngine::ElasticWaveGrid grid(config);
auto state = grid.getState();
// Fill staggered samples, all at t=0, then validate/publish:
grid.setState(state);
const auto reference = grid.getModifiedEnergy(.01);
grid.step(.01);
const auto result = grid.getDiagnostics();
```

The headless `examples/elastic_wave_demo.cpp` initializes a longitudinal `(1,1)`
mode with zero stress, advances 100 steps of .01 and prints JSON with speeds,
physical/reference energies, compatibility and work. It also checks the fixed-h
invariant and compatible stress evolution. Reproduce with a native C++17 toolchain:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel 2
ctest --test-dir build --output-on-failure
build/elastic_wave_demo
build/run_tests '[elastic]'
cmake --install build --prefix /absolute/path/physics-install
cmake -S tests/package_consumer -B build-consumer -DCMAKE_PREFIX_PATH=/absolute/path/physics-install
cmake --build build-consumer --parallel 2
build-consumer/consumer
```

On Windows, add `.exe` and the installed `bin` directory to the current process's
runtime PATH when running the external shared-library consumer. Multiconfiguration
generators also require `--config Release` and the corresponding executable
subdirectory. The existing Linux/Windows native CI jobs build and run this new
CTest/example plus the installed consumer; the existing Linux ASan/UBSan job
covers the new sources and tests. Workflow coverage is not a claim that an
unpublished commit has already passed hosted CI.

Independent tests cover dense negative-adjoint incidence, complex five-field
Fourier maps (including two-cell axes and auxetic material), closed P/S phase
solutions in both axes and oblique directions, and physical continuum convergence
on 16/32/64 grids. With `Lx=2`, `Ly=3`, `rho=2`, `lambda=3`, `mu=2`, duration .19,
and steps no larger than .4 of the configured CFL limit, each axis/oblique P/S
maximum all-field error decreases by a factor in [.18,.31] upon doubling resolution.
The latter controls independently include polarization error between continuum
`k` and represented `kappa`, not only the time integrator's own map.

Further controls exercise 400 fixed-h steps of nonzero, initially incompatible
stress and verify the energy envelope, means and compatibility; exact DC prestress;
pure compression/engineering shear; malformed and nonfinite inputs; aggregate
subnormal energy and constant means; signed cancellation and extreme-range
rejection; genuinely finite-energy stencil overflow; clock failure; work budgets;
zero steps; owning snapshots; and deterministic variable-step replay. None change
or retag the nine unresolved legacy fluid physical targets.


## Recorded local validation

On 2026-10-04, Windows x86-64 LLVM-MinGW Clang 23.1.1, CMake 4.4.3 and Ninja,
with at most two build workers:

- Release native CTest: 22/22 entries passed. Catch2: 580 cases, 571 passed,
  nine existing physical targets failed as expected; 214374 assertions.
- Elastic-only Release and fresh static Debug ASan/UBSan: 13 cases and 2393
  assertions passed. The elastic example also passed ASan/UBSan.
- Installed Release shared-library package: external `tests/package_consumer`
  configured, linked and exited successfully using only the installed prefix.
- Example JSON parsed successfully. At time 1, `cp=1.8708286933869707`, `cs=1`,
  physical energy was 2.998201322037799, reference invariant was
  2.9962726494495158 (initial 2.9962726494494918), compatibility RMS was
  1.1485200399561727e-13, and the last step used 3072 center visits.

The independent continuum experiment described above gave the following maximum
all-field errors (velocity and stress use the fixed test's physical units):

| Mode | 16 x 16 | 32 x 32 | 64 x 64 |
| --- | ---: | ---: | ---: |
| P (1,0) | .0159497 | .00413053 | .00103241 |
| P (0,1) | .0143334 | .00364450 | .000910987 |
| P (1,2) | .0291135 | .00727364 | .00181762 |
| S (1,0) | .00666498 | .00167772 | .000420951 |
| S (0,1) | .00479431 | .00120299 | .000301845 |
| S (1,2) | .0390744 | .00976174 | .00243259 |

These are bounded validation experiments, not claims of arbitrary wavelength,
material contrast or long-time accuracy. Hosted CI results require the normal
parent integration/push workflow and were not fabricated for this local clone.

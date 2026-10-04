# Checkerboard disorder audit (#44)

The later [Wendland comparison](wendland-kernel-experiments.md) adds an explicit
third-family run while preserving every legacy/cubic measurement below.

This diagnostic localizes the existing **Calibrated lattice heals small
positional disorder** failure. It changes no production solver, pressure policy,
kernel, mass, position, target or failure tag. All nine legacy expected failures
remain. Returning particles to their original labeled lattice is a particle
regularization assumption; continuum-fluid equilibrium alone does not require
that return. A stationary discrete state also does not prove correct density
quadrature or supported hydrostatics.

## Reproduce

```sh
cmake --build build --parallel 2
build/run_tests "[disorder]~[!mayfail]"
build/run_tests "Calibrated lattice heals small positional disorder"
build/fluid_disorder_diagnostic --quick --output build/disorder-quick.json
build/fluid_disorder_diagnostic --output build/disorder-full.json
ctest --test-dir build --output-on-failure
```

Append `.exe` on Windows. The [recorded full report](data/fluid-disorder-355fe2b-clang23.json)
uses base 355fe2b plus this diagnostic, Windows Clang 23.1.1 Release. Quick mode
has eight original-input trajectories, full mode sixteen plus ten separately
labeled controls. Both include 24 infinite-lattice rows and two pressure-work
experiments. Dimensions and timesteps are fixed, not command-line inputs.
The diagnostic has at most 1936 particles (operator hard limit 2048), 384 outer
steps per trajectory, h/dx at most eight, and the existing solver's 1024 adaptive
substeps per call. Separation/independent energy scans are O(N²), measured
initially and at eight equally spaced outer-step endpoints. These are sampled
extrema, not bounds on every intervening substep. Allocation/copy and adaptive
solver work are additional. No periodic integration solver is introduced.

## Exact original inputs, rather than retuning density

The main comparison retains 21x21 particles, dx=.1, full support h=.2,
rho0=1000, mass=`rho0*dx*dx*(1/1.014612675f)`, viscosity=.05, c=15, gamma=7,
zero initial velocity and external acceleration. Both original parity
expressions coincide, since `(3*row+column)%2 == (row+column)%2`:

```
delta x_i = a*dx*(1,1)*(-1)^(row+column),  a=.02.
```

This is a single diagonal Nyquist checkerboard, not generic random disorder.
The legacy test measures 2.828407 mm RMS both initially and at .2 s, with exactly
zero speed. The independent double report measures 2.828405 mm for the same
stored positions; its accumulator does not replace the original assertion.
The unchanged target requires RMS below 90% of its initial value.

All main trajectories preserve the same caller mass/rho0 and displacement when
switching between legacy poly6/spiky and matched cubic. Each runs to the same
requested .2 s, with 48/96/192/384 outer steps (dt≈1/240,...,1/1920). Actual
substep counts and the float sum of requested outer durations are reported.

| Family, clamp | Initial bulk rho/rho0 | Bulk max acceleration | Surface max acceleration | Final RMS at dt≈1/480 | Final min separation/dx |
| --- | ---: | ---: | ---: | ---: | ---: |
| Legacy, on | .999436 | 0 | 0 | 2.828405 mm | .960832 |
| Cubic, on, identical mass | .987463 | 0 | 0 | 2.828405 mm | .960832 |
| Legacy, off | .999436 | .08438 | 596.53 | 193.364 mm | .052266 |
| Cubic, off, identical mass | .987463 | 1.94259 | 530.96 | 71.658 mm | .044552 |

With the clamp on, **every** finite-patch particle is below rho0. Consequently
pressure, force and velocity are exactly zero, at all four outer timesteps.
Viscosity dissipates velocity differences; it cannot move these resting
particles. This failure is not an integration error or an unknown wall reaction:
there are no walls or gravity in this fixture.

Removing the clamp creates substantial tensile free-surface contraction.
Across the four same-time refinements, legacy final separation/dx is .011-.052
and RMS is .182-.198 m; cubic separation/dx is .044-.060 and RMS is .068-.072 m.
Refinement does not turn collapse into healing. The finite-patch surface
acceleration dominates, but the independently reconstructed infinite bulk also
accelerates **along** its imposed displacement: .08353 m/s² legacy, 1.94153
m/s² cubic. Thus surface deficiency and an adverse bulk response are distinct
contributions. These observations do not establish a universal bulk pairing
threshold, continuum convergence or long-run stability.

## Independent density symbol

Let r span a centrosymmetric square lattice and G_rho=grad W_rho. Linearizing
summation density for a displacement Fourier mode gives the row symbol

```
delta rho_i = m * sum_r G_rho(r) dot delta x_i * (1-exp(-i k dot r)).
```

For k=(pi/dx,pi/dx), the parenthesized factor is two on odd-parity offsets and
zero otherwise. The odd subset contains both r and -r, so its gradient sum is
zero. This conclusion holds for both independently evaluated radial density
gradients, for the legacy pressure gradient, and at every tested h/dx. It is a
linear density null mode of the periodic/infinite complete-support stencil;
finite surfaces are audited separately. It is not a claim that every finite
perturbation has zero density or energy change.

Only odd neighbors move relative to a chosen checkerboard particle, by
`2*a*dx*(1,1)`. The finite row is therefore

```
C_rho(a) = dx² sum_r W_rho(r + 2*a*dx*(1,1)*odd(r)).
```

At legacy h/dx=2, only four axis neighbors contribute to its change. Independent
Hessian expansion gives `C_rho(a)-C_rho(0)=-4.5*a²/pi+O(a⁴)`, while
`C_rho(0)=3.1875/pi`. At a=.02 the ratio is .9994358958. The fixture's float
mass scale slightly changes this number, not its sign or pressure branch.
Cubic has a positive second-order change at this ratio, but the **same legacy
caller mass** yields .987463 density ratio. Retuning its mass is a different
input, never evidence that a solver correction fixed the original case.

Fixed h/dx spatial row refinement dx=.1/.05/.025 leaves these dimensionless
responses unchanged. Neighbor rows at ratios 2/2.5/4/8 still have the Nyquist
linear null and nonmonotone second-order response. Full integration separately
compares 42x42 particles at dx=.05 and h=.1, preserving the main 2.1x2.1
particle-cell footprint and total mass. Clamped inputs remain stationary;
unclamped inputs still form close pairs. This control measures resolution
dependence and is not a convergence claim for the collapsed trajectories.
The additional family-calibrated controls are explicitly named
`input-family-calibration-control-not-a-solver-fix`. Even the clamped cubic
control's final RMS is 3.292 mm, above the original 2.828 mm initial RMS.

## Actual EOS energy, versus a formal pressure-map adjoint

For Tait pressure p=rho0*c²/gamma*(R^gamma-1), R=rho/rho0, independently integrate
`du/drho=p/rho²`, with u(rho0)=0:

```
u(R) = c²/gamma * [(R^(gamma-1)-1)/(gamma-1) + 1/R - 1].
U = sum_i m_i*u(R_i).
```

With clamped negative pressure, u=0 for R<=1. The original clamped checkerboard
has zero U and kinetic energy: there is no positive EOS energy to dissipate
toward its reference lattice. Without the clamp the bulk perturbation energy
near calibrated R=1 begins at fourth order in a, because delta rho=O(a²).

The independent diagnostic uses double analytic weights/derivatives, separate
all-pair loops and the actual caller positions/masses. It evaluates

```
rhoDot_true_i = sum_j m_j*(v_i-v_j) dot grad W_density(r_ij)
Udot_true = sum_i m_i*p_i/rho_i²*rhoDot_true_i
F_ij = -m_i*m_j*(p_i/rho_i²+p_j/rho_j²)*grad W_pressure(r_ij).
```

For an independently specified compressed 9x9 positive-pressure work control,
legacy mechanical work is 1166.2200 and Udot_true is -1270.8686, giving a
-104.6486 residual (8.97% of mechanical work). Replacing the true density
derivative by grad W_pressure makes a formal pair-work residual cancel to
1.14e-12, but that surrogate is **not** the time derivative of its actual
summation-density EOS energy. No normalization removes this defect.
The matched cubic control's true residual is 6.82e-13. Central force balance
and angular momentum cancel independently; reconstructed native pressure
forces agree within 2e-5 relative maximum force.

Centered finite differences of U along the specified velocities independently
verify the chain rule. Legacy derivative errors at delta t=.001/.0005/.00025
are .0008451/.0002113/.00005282; cubic errors are
.0011546/.00028865/.00007216. Both decrease by four. This is an energy-derivative
oracle, not a change to the solver integrator.
The trajectory report includes recomputed U plus actual kinetic energy,
momentum and angular momentum. Cubic unclamped sampled total energy remains
near its initial 2672.19 while particles collapse; bounded energy alone does
not establish acceptable particle order. Legacy at dt≈1/480 ends near 23149.19
versus initial 3995.97. Existing semi-implicit integration, viscosity and
float publication affect trajectory energy; these measurements do not claim
exact integration conservation. Muller tangential viscosity also need not
conserve angular momentum, so that reported value is not an asserted invariant.

## Rejected shortcut and bounded next step

Neither disabling negative-pressure clamping nor selecting a matched kernel
with these unchanged inputs supports the return-to-lattice criterion. The
first creates tensile collapse, the second remains pressure-free. No production
toggle, density reset, hidden coordinate shift, reference-lattice spring or
pressure floor is added. None of the hydrostatic/wall cases is reclassified.

Particle regularization is a separate model and requires transport/remapping
of physical state, conservation/work accounting, free-surface and boundary
conditions, and independent flow/refinement validation. For context,
[Lind et al.'s fluid shifting study](https://doi.org/10.1016/j.jcp.2011.10.027)
explicitly adjusts hydrodynamic variables after moving particles;
[Dehnen and Aly's pairing study](https://arxiv.org/abs/1204.2471) examines kernel
stability, rather than guaranteeing arbitrary 2D particle-lattice recovery.
Our checkerboard derivation and work oracle are independent of those sources.
The [wall audit](fluid-wall-audit.md) and
[source-owned reflection experiment](planar-reflected-experiment.md) remain
separate boundary investigations and supply no missing bulk restoring term here.

The legacy default's measured EOS-gradient defect is significant; retaining it
for compatibility must not imply an energy-consistent summation model. Matched
cubic is the stronger candidate for a separate default-policy evaluation,
because the homogeneous fixed-h pressure force passes this true energy oracle
and the earlier nominal lattice/rest controls. **Do not change the default on
this evidence alone.** The unchanged disorder inputs still fail their original
healing criterion, and sampled-wall/hydrostatic startup remains unsupported.
Any default proposal should run the unchanged physical matrix with explicit
family comparisons, retain a documented compatibility mode, and separately
audit heterogeneous smoothing lengths, density modes, boundaries and coupling:
the fixed-h identity here does not establish those energy adjoints. A default
change is a model-policy decision, not a disorder correction or proof that the
nine original regimes are solved.

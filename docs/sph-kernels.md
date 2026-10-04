# Validated 2D SPH kernels

Lattice calibration input ranges and work budgets are documented in
[Grid and sampling numerical limits](grid-and-sampling-limits.md).

`SphKernels2D` provides compactly supported density, pressure, and viscosity
kernels for the reference fluid solver. The specialized poly6, spiky, and
viscosity forms follow the approach introduced for interactive SPH fluids by
[Müller, Charypar, and Gross](https://matthias-research.github.io/pages/publications/sca03.pdf).
The constants below are re-derived for two spatial dimensions; using the
paper's 3D constants in a 2D simulation would not preserve normalization.

Let `r = |x|` be displacement magnitude and `h > 0` the smoothing length. All
kernels return zero when `r >= h`.

Evaluation uses double-precision norms and the dimensionless ratio `r/h` to
avoid intermediate float overflow or underflow. Gradient components are checked
after multiplication by their unit direction. Final scalar/vector results must
fit in float; overflow throws `std::overflow_error`, while kernel values below
float resolution round to zero. Square-lattice calibration uses a dimensionless
sum directly and rejects a correction factor that rounds to zero. Calibration
therefore remains scale-independent even when an individual dimensional kernel
weight would overflow. This improves arithmetic range, not the solver's physical
consistency or its documented supported regimes.

## Density weight

For `0 <= r < h`:

```text
W_density(x, h) = 4 / (pi h^8) (h^2 - r^2)^3
```

The coefficient satisfies the two-dimensional normalization condition:

```text
integral_0^h 2 pi r W_density(r, h) dr = 1
```

## Pressure weight and gradient

The normalized spiky weight and its radial gradient are:

```text
W_pressure(x, h) = 10 / (pi h^5) (h - r)^3

grad W_pressure(x, h)
    = -30 / (pi h^5) (h - r)^2 x / r
```

The implementation defines the gradient as `(0, 0)` at `r = 0`, where the
direction is undefined. This makes coincident-particle layouts finite and
preserves antisymmetry for every nonzero displacement.

## Viscosity Laplacian

For `0 <= r < h`:

```text
laplacian W_viscosity(x, h) = 40 / (pi h^5) (h - r)
```

The Laplacian is non-negative throughout its support, which is the property the
viscosity force needs to dissipate rather than inject relative kinetic energy.

The normalization and compact-support requirements are standard SPH kernel
consistency conditions; [Cossins' SPH review](https://arxiv.org/abs/1007.1245)
provides a derivation of the particle approximation and its kernel
requirements. Tests numerically integrate both scalar weights for several
smoothing lengths, compare the analytic pressure gradient with a centered
finite difference, and cover symmetry, support boundaries, invalid inputs, and
coincident particles.


## Explicit kernel families

Existing two-argument calls retain the poly6-density/spiky-pressure behavior.
Three-argument density, pressure weight and pressure gradient overloads accept
`SphKernelFamily::Poly6Spiky` or `SphKernelFamily::CubicSpline`; unrecognized
values throw `std::invalid_argument`. The cubic family uses the same normalized
2D scalar weight and its analytic radial derivative, with full support radius
`h` (the conventional smoothing scale is `h/2`). With `u=2*r/h` and
`C=40/(7*pi*h*h)`, the shape is `C*(1-1.5*u*u+0.75*u*u*u)` for `u<1`,
`C*0.25*(2-u)^3` for `1<=u<2`, and zero at or beyond support. Its coincident
gradient is zero; weight and gradient are continuous through both branches.
Double dimensionless intermediates avoid intermediate float overflow, while
unrepresentable final scalar/components throw `std::overflow_error`. Tiny
representable outputs may round or underflow according to float precision.

`SquareLatticeMassScale(dx,h,family)` explicitly calibrates that family's
infinite square-lattice density sum. It changes the caller's chosen initial
mass if applied; it is never applied automatically and is not a correction
for irregular particles or boundaries. `ViscosityLaplacian` remains the
independent Muller operator with the existing diffusion stability bounds.

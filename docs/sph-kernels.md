# Validated 2D SPH kernels

`SphKernels2D` provides compactly supported density, pressure, and viscosity
kernels for the reference fluid solver. The specialized poly6, spiky, and
viscosity forms follow the approach introduced for interactive SPH fluids by
[Müller, Charypar, and Gross](https://matthias-research.github.io/pages/publications/sca03.pdf).
The constants below are re-derived for two spatial dimensions; using the
paper's 3D constants in a 2D simulation would not preserve normalization.

Let `r = |x|` be displacement magnitude and `h > 0` the smoothing length. All
kernels return zero when `r >= h`.

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

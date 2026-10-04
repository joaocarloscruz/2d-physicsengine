# Matched cubic pressure at unequal fixed supports

WCSPH with `CubicSpline` and `Summation` uses each particle's supplied smoothing
length in its density estimate:

```
rho_i = sum_j m_j W(x_i-x_j,h_i)
```

For fixed `h_i`, differentiating this estimate along a velocity field gives
`rhoDot_i = sum_j m_j (v_i-v_j) dot grad_i W(x_i-x_j,h_i)`.
The barotropic specific energy satisfies `du_i/drho_i=p_i/rho_i^2`.
Differentiating `U=sum_i m_i u_i(rho_i)` therefore gives the central pair force

```
F_ij = -m_i*m_j [
    p_i/rho_i^2 * grad_i W(x_i-x_j,h_i)
  + p_j/rho_j^2 * grad_i W(x_i-x_j,h_j)]
F_ji = -F_ij
```

Each pair is evaluated once, so the force conserves linear momentum. Both
gradients are radial, which also preserves angular momentum. Pressure work
`sum_i v_i dot F_i` cancels `dU/dt` at the continuous discrete-operator level,
up to floating-point evaluation. This does not make semi-implicit Euler an
exact energy-conserving time integrator.

For the Tait EOS, write `s=rho/rho0`, `gamma>1`, and sound speed `c`. The
specific energy referenced to `rho0` is

```
u(rho) = c^2/gamma * [(s^(gamma-1)-1)/(gamma-1) + 1/s - 1]
```

When negative pressure is clamped, this primitive is flat and zero for
`s<=1`; the expression applies above one. Tests independently evaluate this
primitive and cubic density weights, perturb positions while keeping **all
smoothing lengths fixed**, and compare its energy gradient with production
forces. Fixed unequal `h` is not a density-adaptive smoothing length: changes
of `h` would contribute additional derivatives. No grad-h correction is claimed.

## One-sided support regression

At separation `.875`, supplied supports `.5` and `1.25` have mean `.875`.
The mean-support cubic gradient is exactly zero at its support boundary, and
the smaller particle's own density gradient is also zero. The larger particle's
density gradient remains nonzero. Previously the pair produced no pressure
force despite that density dependence. The corrected summation force retains
the larger particle's contribution, with an equal and opposite force on its
neighbour. Masses, rest densities and supports remain caller inputs.
The regression also checks separation `.9375`, strictly beyond the mean
support, with both particle storage orders so discovery uses the union of
the two supports.

The change applies only to matched cubic summation pressure with unequal stored
supports. Equal supports use the original arithmetic exactly. Legacy poly6/spiky
defaults and continuity's common-gradient density/pressure adjoint are unchanged.
Viscosity still uses its existing mean-support Laplacian. Sampled wall density,
pressure extrapolation, mirror rates and fluid-rigid traction are unchanged;
the fluid-only energy identity above does not prove an energy identity for walls.
This correction does not solve particle disorder, tensile instability, pairing,
free-surface consistency or the nine expected failures tracked by issue #44.

## Compression diagnostic contract

`MeasureFluidDiagnostics` and `WcsphSolver::getDiagnostics()` retain their common
comparison operator: pair rates use the selected family's pressure gradient at
the mean support, with each neighbour's mass. WCSPH adds its prepared wall mirror
rates, which use a reflected normal relative velocity and boundary volume.
Density diffusion is excluded. Positive compression and absolute rates are
normalized by each particle's rest density.

These comparison rates are the continuity pressure-map operator, rather than
the true derivative of heterogeneous summation density. Even at equal support,
the legacy pressure kernel differs from its scalar density kernel; wall mirror
rates likewise are not derivatives of the wall density summation. Consequently
a zero comparison compression rate can accompany nonzero cubic summation work,
as in the one-sided regression. No public diagnostic field silently changes its
meaning in this fix. A future true summation-rate observer would need explicit
mode and boundary conventions.

## Verification

`run_tests "[variable-support]"` checks an independent one-sided analytic force,
noncollinear unequal masses and supports, clamped and unclamped EOS energy
gradients, second-order central finite-difference convergence, net force, torque
and pressure work. Compatibility checks retain the old equal-support, legacy
and continuity arithmetic. The full existing fluid matrix and issue #44
thresholds remain unchanged.

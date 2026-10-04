# Periodic MAC constant-viscosity diffusion

`PeriodicMacGrid::diffuse` integrates independent periodic face velocities with
constant nonnegative kinematic viscosity. It is a native operation, separate
from [velocity projection](periodic-mac-projection.md). [JavaScript diffusion
bindings](webassembly.md#owned-periodic-mac-diffusion) provide the same solve and
copied diagnostics. This is not a complete
Navier–Stokes solver: advection, applied forces, walls, free surfaces, variable
viscosity and World/SPH coupling remain separate work. No explicit substeps or
automatic projection are hidden in the call.

Include `physics/physics.h` and link the installed `PhysicsEngine::Engine` target.
`MacDiffusionConfig` specifies `kinematicViscosity`, `timeStep`, and positive
`density`. Viscosity has units length²/time; dt is the diffusion interval. Density
only sets energy units, not the velocity solve. With volumetric density and
2D cell area, energy is per unit out-of-plane depth; with areal density it is
energy. The operation advances no stored simulation clock.

## Discrete equation and solver

Each component solves backward Euler:

```
(I + nu*dt*A) w_new = w_old,     A = -L
(L w)ij = (w(i+1,j)-2w(i,j)+w(i-1,j))/dx²
        + (w(i,j+1)-2w(i,j)+w(i,j-1))/dy².
```

Indices wrap on the same anisotropic grid as projection. Forward and backward
edges are counted separately on two-cell axes, even though they reach the same
neighbor. A Fourier mode has amplification
`1/(1+nu*dt*lambda)`, where
`lambda=4 sin²(pi*kx/Nx)/dx² + 4 sin²(pi*ky/Ny)/dy²`.
Constant fields are fixed points. The matrix is symmetric positive definite;
there is no pressure gauge or Poisson null-space removal.

Conjugate gradients solve each component deterministically. The operator is
divided by its constant diagonal to keep its coefficients bounded, and input
velocities are normalized by each component's largest absolute input value.
This scaling does not improve the mathematical condition number. The current
implementation checks the equation residual during iteration and audits it again
after conversion back to stored face velocities. This final audit subtracts the
stored velocities before normalization, including neighboring face differences;
separately normalizing near-equal values could erase a one-ulp perturbation.
It does not accept a small
recursive CG residual alone. No velocity or diagnostics are published until
both solves and all physical checks succeed.

`absoluteVelocityTolerance` is an RMS velocity, not acceleration, divergence,
or a dimensionless linear-solver tolerance. `relativeVelocityTolerance` is
dimensionless. The fixed combined acceptance target is

```
max(absoluteVelocityTolerance, relativeVelocityTolerance*initialVelocityRms)
initialVelocityRms = sqrt(sum(u_old²+v_old²)/N).
```

Each component must achieve target/sqrt(2); the final combined measured
residual must also meet the target. A loose target can accept an unchanged
field that already satisfies the equation to that tolerance. No hidden
accuracy floor or silent tolerance relaxation applies. Large `nu*dt/dx²`,
large mean flow with small fluctuations, or unattainable storage precision can
prevent convergence within the requested budget.

## Bounds, snapshots and failures

The existing hard grid limit is 262144 cells. Diffusion allocates only fixed
length N scratch arrays (conservatively fewer than 16 arrays beyond the stored
state), regardless of iteration count. Defaults allow 1000 **total** iterations
across both components and 100000000 cell visits. Hard ceilings are 1000000
iterations and 1000000000 visits. Diagnostics report total/per-component
iterations and achieved visits. A component pass charges N visits before its
arithmetic; a pass over both components charges 2N. Copies/allocation bookkeeping
are excluded from visits but remain bounded by the grid limit.

Zero viscosity or zero dt is an exact velocity no-op, with zero residual,
dissipation and increments, zero iterations and freshly published diagnostics.
Inputs and energy representability are still validated, and diagnostic work is
charged. A positive-transport constant field also needs zero iterations, but
`zeroTransportNoOp` is false. Zero work budgets fail whenever work is needed.

`lastDiffusion()` returns a copied `MacDiffusionDiagnostics` value describing
the last successful diffusion. `setVelocities` and `project` leave it intact;
`diffuse` leaves `lastProjection()` intact. A failed call preserves both face
arrays and the entire previous diffusion diagnostics, including failures after
one component or after both components have been solved.

Nonfinite or negative viscosity/dt/tolerances, nonpositive density and excessive
controls throw `std::invalid_argument`. Iteration/work exhaustion, unattainable
residual, mean drift or failed energy checks throw `std::runtime_error`.
Unrepresentable derived coefficients/diagonal, scaled coefficients or finite
diagnostic arithmetic throw `std::overflow_error`. Exponent-staged products
support finite energies when a raw velocity square would overflow or underflow;
they do not guarantee that every finite input yields representable intermediate
stencils and diagnostics. Nonzero final staged products that underflow also
fail transactionally. This is a float64 numerical foundation, not arbitrary
precision diffusion.
In particular, a nonzero stored difference that underflows when divided by the
component scale is rejected. Extreme within-component dynamic range can erase
tiny rhs entries during iteration; the final stored audit must still detect
lost nonzero increments instead of reporting a zero residual.

## Mean and energy diagnostics

Periodic diffusion conserves each component mean mathematically. The measured
change must be at most `(16N+128)*epsilon*maxAbsInitialComponent`; diagnostics
expose the initial/final means and separate allowances. Zero components have
zero allowance. This scale-aware bound accounts for ordinary summation/storage
rounding; it is not a proof for every conceivable conditioning regime.

Let `V=dx*dy`, `r=(I+nu*dt*A)w_new-w_old`, and sum over both components. Then

```
E_new-E_old = -rho*V*nu*dt <w_new,A w_new>
              -rho*V/2 ||w_new-w_old||² + rho*V <w_new,r>.
```

`gradientDissipation` is the first nonnegative magnitude, evaluated using
periodic forward-edge squared differences. `incrementKineticEnergy` is the
second magnitude. `residualWork` is the signed final term. Initial/final energies
and `storageEnergyError`, the measured identity discrepancy, are reported.
`residualEnergyBound=rho*V*N*finalVelocityRms*finalResidualRms` is its
Cauchy–Schwarz residual-work bound.

Publication requires energy increase no greater than that residual bound plus
`roundoffEnergyAllowance`, and absolute storage discrepancy no greater than the
roundoff allowance. The latter is `(32N+256)*epsilon` times the largest reported
energy/dissipation/increment/absolute residual-work scale. It has no absolute
unit energy floor and is zero for zero energy. This is a scale-aware arithmetic
guard, not a rigorous forward-error proof. A finite residual permits bounded
energy increase; successful diffusion alone does not certify strict monotonicity
with an arbitrarily loose tolerance.

Diffusion commutes with compatible projection in exact periodic arithmetic and
preserves a divergence-free field. It also diffuses divergence: it does not
eliminate it. Tests check both facts using independently derived Fourier modes.

## Reproduce and achieved values

```sh
cmake --build build --target mac_diffusion_demo engine_tests_runner --parallel 2
build/mac_diffusion_demo
build/run_tests "[mac-diffusion]"
```

Use `.exe` on Windows. The 24×18 example (dx=.1, dy=.17, nu=.07, dt=.2,
rho=1000) takes two total CG iterations and 27648 visits with Release Clang
23.1.1. Residual RMS is 1.368903273374480e-15 against a 1e-10 target. Means remain
(.3,-.2) to roundoff; energy falls from 2607.119999999994 to 1942.338377219706.
Gradient dissipation is 599.862251464679 and increment energy is 64.919371315615.
Residual-work bound is 7.311669284271e-12; storage discrepancy is
4.000981272263e-12. These are achieved fixture values, not universal guarantees.

The independent tests cover anisotropic/two-cell Fourier amplification, mixed
modes with constant mean flow, measured energy identity and physical residual,
continuous shear exponential decay under first-order temporal and second-order
spatial refinement, projection composition, divergence diffusion, exact no-ops,
shared iteration and late-work failures, copied snapshots, invalid controls and
velocity amplitudes 1e-200 and 1e200 with compensating density. A separate
tiny-transport test has nu*dt underflow when multiplied alone, while resolved
grid coefficients produce positive dissipation and a nonzero stored decrement.
Large mean flow with a one-ulp perturbation must satisfy a direct stored-equation
oracle on acceptance or fail transactionally. The installed
package consumer exercises the alternating two-cell amplification. ASan/UBSan
can run these diffusion tests with both MAC sources linked directly.

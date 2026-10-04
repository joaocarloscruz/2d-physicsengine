# Periodic MAC velocity projection

`PeriodicMacGrid` projects double-precision periodic face velocities onto a
discretely divergence-free field. It is a native, independent component: no
advection, viscosity, obstacles, free surfaces, time integration, or World/SPH
coupling. It does not resolve the outstanding SPH #44 validation failures.
Include `physics/physics.h` and link `PhysicsEngine::Engine` from the installed
CMake package. [mac_projection_demo.cpp](../examples/mac_projection_demo.cpp)
is a reproducible example; [tests](../tests/test_periodic_mac_grid.cpp) provide
independent Fourier, summation-by-parts and spatial-refinement checks.

## State, indexing and units

`PeriodicMacGridConfig` fixes `columns=Nx`, `rows=Ny`, `spacingX=dx`, and
`spacingY=dy`. Both dimensions are at least two; their product is at most
262144, checked before allocation. Periods are Nx dx and Ny dy. The origin is
(0,0). Index `i + Nx*j` stores:

* `MacVelocityState::xFaces`: u at (i dx, (j+1/2) dy).
* `MacVelocityState::yFaces`: v at ((i+1/2) dx, j dy).
* Potential and pressure: cell centers ((i+1/2) dx, (j+1/2) dy).

Each array has Nx Ny entries. Periodic faces are stored once, with no duplicate
last row/column. `velocities()`, `config()`, and `lastProjection()` return owning
values; changing a returned value does not change the grid. `setVelocities`
validates and copies complete finite arrays transactionally. It leaves the last
successful projection snapshot intact; that snapshot describes that earlier
operation until another projection succeeds.

Potential φ has units length²/time, and pressure is p=ρφ/dt. Density and dt are
positive finite caller inputs; dt is a pressure-conversion interval, not a
simulation clock or stability step. With volumetric density in kg/m³, velocity
in m/s and spacings in m, pressure is Pa and reported energies are J per meter
of out-of-plane depth. With areal density in kg/m², pressure has units N/m and
energies are J. Density is uniform and does not evolve.

## Compatible operators and Fourier frequency

All indices below wrap periodically:

```
(D u)ij = (u(i+1,j)-u(i,j))/dx + (v(i,j+1)-v(i,j))/dy
(G φ)x,ij = (φ(i,j)-φ(i-1,j))/dx
(G φ)y,ij = (φ(i,j)-φ(i,j-1))/dy.
```

Shifting the finite periodic sums gives `<φ,D u>=-<G φ,u>` under the common
cell-area weight dx dy. Thus A=-DG is symmetric positive semidefinite, with
only the constant scalar null mode. On zero-mean scalars it is positive
definite. For integer mode numbers kx,ky, its eigenvalue is

```
λ = 4 sin²(π kx/Nx)/dx² + 4 sin²(π ky/Ny)/dy².
```

Forward and backward edges remain distinct when an axis has two cells: they
reach the same neighboring cell but both contribute. For its alternating mode
the axis contribution is **4/dx²**, not 2/dx². The analytical tests explicitly
exercise 2×5, 5×2 and 2×2 grids, as well as anisotropic spacings.

Projection solves `A φ = -D u_initial` and forms `u_final=u_initial-G φ`.
Zero-mean unpreconditioned conjugate gradients remove the pressure gauge. The
right-hand side's tiny floating-point mean is removed for compatibility and
reported as `removedDivergenceMean` in initial-divergence units. It is not
discarded from the final acceptance test: actual stored face divergence is
recomputed and includes any remaining mean component.

For staggered sine modes define ax=2 sin(π kx/Nx)/dx and
ay=2 sin(π ky/Ny)/dy. Face amplitudes (ax,ay) are longitudinal and project away;
(ay,-ax) are transverse and are preserved. Cell cosine phases are shifted by
half a cell in each direction. These independently derived amplitudes and
frequencies are used in tests, rather than testing only the implemented stencil
against itself. Fixed physical smooth modes give second-order spatial errors
in divergence and recovered potential; coarse/fine error ratios exceed 3.9
for Nx=16,32,64.

## Tolerances, work and transactional failure

`MacProjectionConfig` specifies density, dt, absolute/relative RMS divergence
tolerances and finite work/iteration limits. The fixed acceptance target is

```
max(absoluteDivergenceTolerance,
    relativeDivergenceTolerance * initialDivergenceRms).
```

The target is based on the original velocity divergence, not a decreasing
recursive residual or rescaled norm. The candidate's recomputed divergence
must meet this target. A recursively small CG residual triggers a candidate
check; if it fails, the Poisson residual is recomputed and CG restarts within
the original budgets. Iteration exhaustion, work exhaustion or unattainable
stored-face accuracy throws `std::runtime_error` without publication.

`cellVisits` charges Nx Ny before every full-grid arithmetic pass (stencil,
vector update, mean or dot/norm pass). Updating both face components together
counts as one pass. Vector allocation/copy bookkeeping is excluded, but memory
is bounded by the hard grid limit. Defaults allow 1000 iterations and
100000000 cell visits; hard ceilings are 1000000 iterations and 1000000000
visits. Zero budgets are permitted and fail if work is needed. Invalid settings
throw `std::invalid_argument`; non-finite derived arithmetic, unrepresentable
geometry/pressure coefficients or cell mass throw `std::overflow_error`.

There is no hidden tolerance floor or silent tolerance relaxation. Float64
precision depends on velocity magnitude, spacing, aspect ratio and cancellation.
Large mean flow with small fluctuations can make a requested divergence
tolerance unachievable after face storage. RMS is evaluated with a scaled norm
so tiny nonzero divergence is not mislabeled an exact zero; extremely small
CG squared norms may still underflow and cause an honest failure. Finite input
does not imply all dot products, pressure or energy diagnostics are representable.

An exactly zero initial divergence leaves velocities exactly unchanged and
publishes zero potential/pressure, zero iterations and coherent charged work,
density/dt diagnostics. Even this operation validates density, dt and derived
coefficients. A field already within a loose requested tolerance may also need
zero iterations, but its `zeroDivergenceNoOp` flag remains false unless its
initial divergence is exactly zero. All failed calls preserve face velocities
and the complete last successful projection snapshot.

## Mean flow, energy and achieved residual

Periodic gradient sums vanish, so mean face velocities are preserved up to
storage rounding. Diagnostics report both initial and final means. Define
V=dx dy, c=Gφ, E=ρV/2 Σ(u²+v²). The ideal discrete relation is

```
E_final-E_initial = ρV <φ,D u_final> - ρV/2 ||c||²
<u_final,c> = -<D u_final,φ>.
```

The diagnostics expose initial/final/correction energies, both pairings, and
`storageEnergyError`, the discrepancy in the first identity using the actual
stored velocity correction. `residualEnergyBound` is
ρV N * finalDivergenceRms * potentialRms, the Cauchy-Schwarz bound on the
residual pairing. `roundoffEnergyAllowance` is 128 machine epsilons times the
largest of one and the three energies. Publication rejects energy above
`initialEnergy + residualEnergyBound + roundoffEnergyAllowance`.

An exact projection is orthogonal and decreases kinetic energy by the
correction energy. Finite or loose tolerance permits the explicitly bounded
residual term; do not claim strict energy nonincrease solely from a successful
call. The independent tests check both pairings, the storage discrepancy and
measured final divergence with tight and loose tolerances.

## Reproduce

```
cmake --build build --target mac_projection_demo engine_tests_runner --parallel 2
build/mac_projection_demo
build/run_tests "[mac]"
```

Use `.exe` on Windows. On Clang 23.1.1 / LLVM-MinGW / Release, the deterministic
24×18 example with dx=.1, dy=.17, ρ=1000, dt=.01 takes one CG iteration and
15984 cell visits. Divergence RMS goes from 16.26736677698646 to
4.246994969107592e-13, below its 1.626736677698646e-9 target. Mean velocities
remain (.3,-.2) to roundoff; energy goes from 47324.03338618049 to
5085.878984055375. The residual energy bound is 2.205461175240641e-9 and
storage energy discrepancy is 4.365574568510056e-11. These are achieved values
for this configuration, not universal attainable-accuracy guarantees.

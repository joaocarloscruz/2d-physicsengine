# Periodic ideal-gas Euler grid

`PeriodicEulerGasGrid` is a standalone finite-volume solver with a first-order default
for a homogeneous ideal gas on a periodic rectangular grid. It evolves density,
two momentum densities and total energy density in double precision. There are
no sources, viscosity, heat conduction, walls, particle or rigid-body coupling,
or shared World clock. [Owned JavaScript bindings](webassembly.md#owned-periodic-ideal-gas-euler-grids)
expose the same native solver and budgets. It is a bounded initial Euler model;
it does not resolve the separate SPH validation failures.

`step()` retains this first-order model and its measured baseline. The explicit
native [`stepSecondOrder()` option](periodic-euler-second-order.md) adds conserved
reconstruction and SSPRK2 with a separate, stricter positivity CFL and bounded
stage retries. The sections below describe the unchanged first-order method.
The browser Euler binding remains first-order-only.

## Model, geometry and ownership

Write `U=(rho,mx,my,E)`, `u=mx/rho`, `v=my/rho`,
`I=E-(mx*mx+my*my)/(2*rho)`, `p=(gamma-1)*I`, and
`c=sqrt(gamma*p/rho)`. `gamma` is finite and strictly greater than one. The
equations are `U_t + F(U)_x + G(U)_y=0`, with

```
F = (mx, mx*u+p, my*u, (E+p)*u)
G = (my, mx*v, my*v+p, (E+p)*v).
```

These are the conservative Euler equations described in the primary
[Clawpack two-dimensional example](https://www.clawpack.org/gallery/pyclaw/gallery/quadrants.html)
and [Euler Riemann reference](https://www.clawpack.org/riemann_book/html/Euler.html).
The implementation and exact test oracle are independently derived, with no
external numerical dependency or copied solver code.

In SI, rho is kg/m³, mx/my are kg/(m² s), E/I/p are J/m³ (pressure Pa), velocity
and sound speed are m/s, and spacing is m. Area integrals are per unit depth:
mass kg/m, momentum kg/s, energy J/m. Reduced units are also valid if consistent.
`EulerGasGridConfig` owns immutable columns, rows, spacingX, spacingY, gamma.
The domain is `[0,columns*spacingX) x [0,rows*spacingY)`, with row-major index
`i+columns*j` and centers `((i+.5)*spacingX,(j+.5)*spacingY)`. Stored values are
cell averages of conserved variables; averaging primitives first is generally
different. Each direction needs at least two cells; two-cell periodic grids
retain both geometrically distinct faces between the same neighboring cells.

`state()`, `primitives()`, `config()` and `lastStep()` return owning value copies.
`EulerGasState` has four arrays: density, momentumX, momentumY, totalEnergy.
`setState()` checks all lengths before scanning values/copying, validates strict
positive rho and stored I, then commits the whole copy. It preserves the clock
and historical lastStep; those diagnostics still describe the earlier step,
not the newly supplied state. The initial default is uniform rho=1, u=v=0, p=1;
the default lastStep is an all-zero "no step yet" record. Initial numbers are
reduced-unit defaults, not admissibility floors.

## Fluxes, positivity and adaptive CFL

Each substep computes global directional speeds
`alphaX=max(|u|+c)` and `alphaY=max(|v|+c)` from its staged old state. The x face
between L and R uses `F*=.5*(F(L)+F(R))-.5*alphaX*(R-L)`; y is analogous. The
old state supplies both directional fluxes, so the update is unsplit forward
Euler. Each face transfer is computed once and contributes equal and opposite
increments to its two cells. No pressure/density floors, slope limiter, energy
repair, conservation rescaling, or hidden regularization is applied.

In exact arithmetic, strict `h*(alphaX/dx+alphaY/dy)<1` is sufficient for
positivity. The admissible set `{rho>0, E-|m|²/(2rho)>0}` is convex. For either
x LF split state `U_s=U+s*F(U)/alpha`, `s=+1` or `-1`, set
`beta=1+s*u/alpha`. Direct cancellation gives

```
rho_s = rho*beta
I_s = I*beta - p*p/(2*rho*alpha*alpha*beta).
```

Since `alpha-|u|>=c`, `alpha²*beta²>=c²=gamma*p/rho`, whereas positivity only
needs `alpha²*beta²>(gamma-1)*p/(2rho)`. This holds for every gamma>1, including
gamma>2. The unsplit update is `(1-Cx-Cy)*U` plus the four neighboring LF split
states with weights Cx/2 or Cy/2, where `Cx=h*alphaX/dx`, `Cy=h*alphaY/dy`.

The implementation rounds computed speeds/rates upward and the computed CFL
duration downward, then takes the minimum with the exact caller maxSubstep and
remaining duration. It recomputes the speeds after each accepted staged update.
`cflSafety` must be strictly in (0,1), default .9. These local roundings are not
a global interval proof for all floating computations: every resulting stored
cell is independently checked for finite values, rho>0, I>0 and representable
derived pressure/sound speed. Failure rejects the entire call. Vacuum and states
whose internal energy is lost in E-K subtraction are outside this strict model.

## Bounds, arithmetic and transaction

The hard limits are 262144 cells, 1000000 substeps and 1000000000 logical cell
visits. Dimensions/product are checked before allocation. User budgets may be
smaller, including zero; they cannot exceed hard limits. `EulerGasStepConfig`
also has positive finite maxSubstep (default .1), maximumSubsteps (10000) and
maximumCellVisits (100000000). Remaining work and substep budgets are checked
before the next staged iteration. There are no positivity retries.

A successful positive step with S substeps charges exactly `(4+6*S)*N` visits:
two initial summary scans, then per substep one each for wave/primitive bounds,
normalization, clearing increments, x faces, y faces and stored update plus
admissibility, followed by two final summary scans. A visit includes bounded
neighbor access and scalar arithmetic. Allocation initialization, owning copies,
and destruction require additional bounded linear work and are not counted as
operator visits. Scratch storage is a fixed number of arrays proportional to N;
the hard cell limit bounds it independently of requested duration. A zero step
still requires exactly 3N visits (two summary scans and a wave/primitive audit),
and can fail on budget/range errors. Success preserves fields and clock while
publishing fresh diagnostics with zeroDurationNoOp=true, substeps=0 and
lastSubstep=0. It is not a no-op on the diagnostic observer.

Complete products, quotients and square roots stage binary exponents to avoid
avoidable intermediate overflow. Transfers normalize shared conserved component
scales using max rho, max E and sqrt(max rho*max E), and use compensated paired
increments. Energy flux splits E and p before time/spacing multiplication.
Geometry area/extents, derived fields, normalized values, transfers, stored
results, integrated diagnostics and represented clock/duration advances must be
finite and representable. Nonzero complete products that round to zero are
rejected. This includes individually underflowing kinetic component products,
even when total internal energy would survive, and extreme dynamic ranges that
lose a nonzero normalized component. Cancellation in E-K cannot be recovered
from the already rounded stored total energy. Very large gamma may make p or c
unrepresentable. These are explicit numerical range limitations, not absolute
physical cutoffs or a promise to support arbitrary finite input scales.

`step()` stages fields, clock and diagnostics. Any invalid input, allocation,
arithmetic, positivity, conservation, duration stagnation, or budget failure
preserves the complete prior publication, even after earlier scratch substeps.
Snapshots do not borrow memory. Simulation objects are not thread-safe.

## Stored diagnostics and validation

Initial/final summaries independently integrate stored rho, mx, my and E, and
derived I/K, using compensated scaled sums. They report absolute momentum
integrals for cancellation-aware comparisons, minimum/maximum rho and p. Actual
mass, signed momentum and total-energy defects are published without rescaling.
Their roundoff guard is `(64*N+96*S+128)*machine_epsilon` times the larger
initial/final positive or absolute integral. It has no unit-size floor; it is a
scale-aware heuristic accumulation guard, not a certified interval bound. Failed
guards reject the call. Internal and kinetic energy may exchange and are not
separately conserved. Total-energy conservation alone is not an entropy or
shock-accuracy guarantee. Diagnostics also report largest computed signal
speeds, actual maximum CFL, final substep, work count, duration and clock bounds.

The headless `euler_gas_demo` emits JSON moving-contact refinement records.
`run_tests "[euler]"` checks an independent physical face-flux update on
anisotropic/two-cell grids, DC/replay/copies, first-order smooth contact and
nonlinear acoustic simple-wave refinement, an independently pressure-matched
exact Sod Riemann solution, stored total-energy conservation, gamma/range/strict
positivity, budgets, zero steps, and complete rollback. The Sod oracle integrates
conserved cell averages and splits quadrature at wave discontinuities. A doubled
domain at identical dx quantifies numerical periodic-image contamination; finite
physical propagation speed alone does not isolate a diffusive numerical stencil.
Rusanov diffusion visibly damps contacts and smears shocks; refinement reduces
these errors. These are local optimized native measurements (gamma=1.4), not
hosted CI claims. The owned JavaScript suite also checks independent split-state,
contact and Sod oracles through the WASM solver:

| Comparison | Resolutions | Conserved error or density RMS |
| --- | --- | --- |
| Moving 2D contact at t=.15 | 32, 64, 128, 256 columns | .0984641, .0663189, .0389790, .0212215 |
| Nonlinear right-moving simple wave at t=.2 | 32, 64, 128, 256 columns | .00431427, .00185346, .000898153, .000431017 |
| Sod exact Riemann window at t=.12 | 128, 256, 512 columns | .0983694, .0746175, .0504520 |

The contact's finest error ratio is 1.84; the nonlinear wave's is 2.08. Shock
errors converge more slowly because the discontinuities remain smeared. The
256-column length-2 Sod window and its 512-column length-4 control (same dx)
had exactly identical stored fields in the comparison window in this run.
The headless contact's total-energy defects were at most 4.45e-16; its actual
maximum CFL was .89999999999999991. These are measurements of these fixtures,
not universal accuracy, error-bound, or exact-energy claims. Reproduce with
`run_tests "[euler]"` and `euler_gas_demo`; native CTest includes both the full
test runner and the JSON example smoke.

```cpp
#include <physics/physics.h>
using namespace PhysicsEngine;
PeriodicEulerGasGrid gas({64, 32, 1.0/64, 1.0/32, 1.4});
auto state = gas.state(); // Uniform reduced-unit rho=1, p=1 by default.
gas.setState(state);      // Validates and owns a complete copy.
EulerGasStepConfig options;
options.cflSafety = .8;
const auto diagnostics = gas.step(.05, options);
const auto primitives = gas.primitives();
```

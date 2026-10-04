# Periodic homogeneous TMz Maxwell fields

`MaxwellGrid` propagates double-precision Ez, Hx and Hy in a uniform periodic
medium. `step` remains lossless; the explicit
[Ohmic step](maxwell-ohmic.md) adds a homogeneous scalar constitutive current. It is independent of Engine/World and the prescribed-field
`ChargedParticle` module. There are no imposed charges/currents, particle-field coupling, material
interfaces, finite-conductor geometry, absorbing boundaries or 3D components.
The native API and [owned WebAssembly API](webassembly.md#periodic-tmz-maxwell-fields)
expose the same bounded model.

Include `physics/physics.h` and link the installed `PhysicsEngine::Engine`
target. The background references are John B. Schneider's
[chapter 3: central differences and Yee stepping](https://eecs.wsu.edu/~schneidj/ufdtd/chap3.pdf),
[chapter 7: numerical dispersion](https://eecs.wsu.edu/~schneidj/ufdtd/chap7.pdf),
[chapter 8: TMz equations and spatial staggering](https://eecs.wsu.edu/~schneidj/ufdtd/chap8.pdf),
and [Meep's Yee lattice description](https://meep.readthedocs.io/en/latest/Yee_Lattice/).
The implementation, synchronous map and invariant below are independently
derived for this module's symmetric update; external code is not incorporated.

## Geometry, state and units

`MaxwellGridConfig` fixes columns Nx, rows Ny, spacings dx/dy, permittivity eps
and permeability mu, and the numerical budgets. Configuration is immutable.
Both dimensions are at least two; their product is at most 262144, checked
before multiplication/allocation. The periods are Nx dx and Ny dy, with origin
(0,0). Index `i + Nx*j` stores:

| Array | Location | SI units |
| --- | --- | --- |
| `ez` | (i dx, j dy) | V/m |
| `hx` | (i dx, (j+1/2) dy) | A/m |
| `hy` | ((i+1/2) dx, j dy) | A/m |

Every periodic sample is stored once: there is no duplicate final row/column.
All three arrays refer to the same time. `getState()` and `getConfig()` return
owned copies; `getDiagnostics()` returns a value. Mutating a returned object
cannot change the grid. `setState(MaxwellFieldState)` checks all three complete
lengths, finite entries and derived diagnostics before publishing anything.
It preserves time, resets last-substep/work counters and reference h to zero,
and recomputes the current field metrics. It does not silently reset a clock.

Permittivity is in F/m and permeability in H/m; wave speed is
c=1/sqrt(eps mu). Defaults eps=mu=1 use reduced units, not vacuum SI constants.
Electric/magnetic/total/modified energies are J per meter of out-of-plane depth:

```
area = dx*dy
electricEnergy = area*eps/2 * sum(Ez²)
magneticEnergy = area*mu/2 * sum(Hx²+Hy²)
totalEnergy = electricEnergy + magneticEnergy.
```

Component means/maxima have their field units. Time, stable timestep, last
substep and `modifiedEnergyStep` are in seconds. `lastSubsteps` and
`lastCellVisits` are counts.

## Equations and synchronous integration

The homogeneous lossless TMz equations are

```
Hx_t = -Ez_y/mu
Hy_t =  Ez_x/mu
Ez_t = (Hy_x-Hx_y)/eps.
```

`Dx+ f=(f(i+1,j)-f(i,j))/dx` and `Dy+` use forward periodic differences;
`Dx- f=(f(i,j)-f(i-1,j))/dx` and `Dy-` use backward differences. A substep h
performs three passes:

```
Hx_half = Hx - h/(2mu)*Dy+ Ez
Hy_half = Hy + h/(2mu)*Dx+ Ez
Ez_new  = Ez + h/eps*(Dx- Hy_half - Dy- Hx_half)
Hx_new  = Hx_half - h/(2mu)*Dy+ Ez_new
Hy_new  = Hy_half + h/(2mu)*Dx+ Ez_new.
```

These exact subflows form a symmetric H-half/E-full/H-half composition, second
order in time. H output is synchronous with E output, rather than exposing the
usual half-time magnetic state. Centered differences at the staggered locations
are second order in space. On a two-cell axis, the forward and backward edges
remain distinct even though both reach the other cell.

`getMagneticDivergence()` reports `Dx+ Hx + Dy+ Hy` at dual corners
((i+1/2)dx,(j+1/2)dy), in A/m². Multiplying by uniform mu gives divergence of B.
Each H kick adds a discrete curl: its divergence increment is
`(-Dx+ Dy+ + Dy+ Dx+)Ez*h/(2mu)=0` in exact arithmetic. Initial nonzero
divergence is preserved, never projected or rejected on a physical tolerance.
Diagnostics `magneticDivergenceRms` and `maxAbsMagneticDivergence` also measure
divergence of H in A/m², not divergence of B. The physical constraint is
`div B = mu div H = 0` for this uniform positive permeability. These RMS/max
magnitudes let callers measure roundoff drift.
Periodic derivative sums also preserve all three component means to roundoff.

## Fourier map, CFL and modified energy

For mode indices mx/my, let

```
ax = 2*sin(pi*mx/Nx)/dx
ay = 2*sin(pi*my/Ny)/dy
g = ax²+ay²
omega = sqrt(g/(eps*mu)), z = h*omega, d = 1-z²/2.
```

Use Ez=e cos(theta), Hx=ay*b sin(theta at Hx), Hy=-ax*b sin(theta at Hy).
Substitution into the three passes gives the independent amplitude map

```
e_new = d*e - h*g*b/eps
b_new = d*b + h*(1-z²/4)*e/mu.
```

Its determinant is one and its trace is 2-z². For z<2 the eigenvalues have
unit modulus and phase advance `phi=2*asin(z/2)`. Thus numerical frequency is
`phi/h`, generally different from both the continuum frequency and the
semidiscrete omega. A traveling eigenmode has synchronous magnetic amplitude
`sqrt(1-z²/4)/(mu*omega)` times (ay,-ax), with the corresponding staggered
cosine phases. Grid anisotropy, mode direction, spacing and h affect dispersion
and polarization; lowering only h does not remove spatial dispersion.

Direct multiplication of this 2x2 map preserves
`eps*(1-z²/4)*e² + mu*g*b²`. Periodic summation by parts extends this to
arbitrary fields and includes unchanged magnetic curl-null/DC components:

```
Q_h = totalEnergy - area*h²/(8mu)*sum((Dx+ Ez)²+(Dy+ Ez)²).
```

Since g <= 4*(dx^-2+dy^-2), a sufficient positive-energy, stable bound is
`S = c*h*sqrt(dx^-2+dy^-2) < 1`. The implementation enforces this strict bound
with configured `0<cflSafety<1`. In exact arithmetic
`(1-S²)*totalEnergy <= Q_h <= totalEnergy`, giving bounded physical energy.
Raw physical energy generally oscillates and is **not** exactly conserved.

`getModifiedEnergy(h)` evaluates Q_h without mutation. It accepts h=0 (raw
energy) or a representable positive reference step satisfying the strict
physical CFL, irrespective of the smaller configured maxSubstep. Current
diagnostics report `modifiedEnergy` at `modifiedEnergyStep=lastSubstep`; initial
and newly set states use h=0. Compare `getModifiedEnergy(h)` before and after
a run with one fixed actual h. If a later call selects a different h, Q_h is a
different quadratic form and is not an invariant across that change. No claim
of exact conservation is made for floating-point sums.

## Bounds, work and failure

`getStableTimeStep()` is the smaller of maxSubstep and a downward-rounded
`cflSafety/(c*hypot(1/dx,1/dy))`. Only the computed CFL duration is rounded:
an exact user limit such as .01 remains .01 under loose CFL. `step(dt)`
partitions positive dt uniformly using `ceil(dt/limit)` and rechecks the actual
quotient h, adding a partition if integer rounding hid a bound violation.
Requested counts are compared with their bounds before conversion to size_t.

Maximum substeps default to 10000 with hard ceiling 1000000. Maximum cell visits
default to 100000000 with hard ceiling 1000000000. Both budgets must be positive.
Each full-grid H kick is one cell pass even though it updates two components;
the E drift is one pass; the fused final energy/means/divergence/gradient
diagnostic loop is one pass. Accepted work is exactly
`N*(3*lastSubsteps+1)`. This accounting bounds arithmetic scans, rather than
every scalar operation. Allocation/copy/finite-input validation and getter/setter
scans are excluded and bounded by the hard cell limit. At that limit the three
stored arrays occupy 6 MiB, plus 6 MiB for the staged step arrays; caller-owned
snapshots and allocator overhead are additional.

`step(0)` is a complete no-op, including clock and previous work diagnostics.
A positive step stages all fields, clock and final diagnostics. Invalid input,
resource exhaustion, allocation failure or numeric failure publishes none of
them. Invalid fields/coefficients use `std::invalid_argument`, cell/count/work
limits use `std::length_error`, and arithmetic/storage/clock range failures use
`std::overflow_error`. `setState` has the same complete publication guarantee.

Finite inputs alone do not guarantee representable updates. Reciprocal spacings
and material constants, domain lengths/area, weighted half-energy coefficients,
curl coefficients `1/(eps*dx)`, `1/(eps*dy)` and the corresponding mu coefficients,
wave speed, CFL rate and
chosen duration must be positive finite doubles. Every positive step also
requires nonzero finite scaled update coefficients. A finite field difference
can overflow (for example +1e308 minus -1e308), and final energy can exceed the
double range even when individual fields fit. These cases throw without a
partial step; fields are never clipped to a numerical floor or ceiling.
Separate weighted electric, magnetic and correction norms accumulate with
`hypot` and are squared once, so representable aggregate energy is not lost to
per-cell square underflow. A mathematically nonzero aggregate whose squared norm
underflows is rejected. Floating-point norm accumulation and quadratic-form
subtraction can still lose precision. RMS
magnetic divergence uses a scaled hypot norm and rejects underflow to zero when
a nonzero sample was measured. Near the physical CFL or extreme scales, a
quadratic-form subtraction can also lose its positive remainder and fail honestly.
There is no arbitrary absolute field, energy or divergence tolerance floor.

All three field means use a binary-scaled compensated reduction in the same
fused diagnostic sweep. Power-of-two scaling retains supported represented
samples exactly during normalization; Neumaier compensation preserves ordinary
signed cancellation. Final division by the cell count stages the normalized
mantissa and combined exponent, so it does not divide each raw field sample by
N. Constant-sample detection returns the stored value directly, avoiding a second
rounding through `N*x` for non-power-of-two counts; canonical zero remains +0.
Constants retain their stored value even in the subnormal range. Means
observe the actual represented fields after either stepping path; they do not
correct the fields to enforce an ideal continuum mean after storage roundoff.

This is a finite-range observer, not an exact arbitrary-precision summation.
Normalization/rescaling that erases any contribution (including partial
underflow), an overflowing final result or a nonzero final mean that underflows
throws before publication. Extreme mixed dynamic ranges can therefore be
rejected even when separate aggregate energy fits. Work counts, allocation
budgets, original wave arithmetic and explicit Ohmic split arithmetic remain
unchanged; no physical unit floor is introduced.

The regression controls use 512x512 unit-spacing samples with eps=mu=1e308,
uniform signed Ez/Hx/Hy of magnitude 6e-319 or 7e-319, and independently weighted
aggregate energy. Electric energy is denorm_min while all three means equal
their stored constants before/after wave and zero-conductivity steps. The old
per-entry division gave zero for 6e-319 and 1.295163e-318 for 7e-319. Four-cell
`[1e16,1,-1e16,0]` controls preserve the independent .25 residual under every
ordering and sign; separate stored-value oracles check observations after wave
and positive-conductivity evolution. These attained controls do not imply exact
discrete-mean preservation for arbitrary floating-point evolution.

## Reproduce

```cpp
PhysicsEngine::MaxwellGridConfig c;
c.columns=32; c.rows=24; c.spacingX=.1; c.spacingY=.17;
PhysicsEngine::MaxwellGrid grid(c);
auto fields=grid.getState();
fields.ez[0]=1;
grid.setState(fields);
grid.step(.01);
auto diagnostics=grid.getDiagnostics();
```

Build/run `maxwell_grid_demo` and `run_tests "[maxwell]"` (add `.exe` on
Windows). Tests independently check the Fourier map and traveling polarization,
two-cell axes, anisotropic grids, nonzero-divergence preservation, fixed-step
energy behavior, replay and rollback. Clang 23.1.1 Release refinement error ratios
were 3.9653/3.9929 in space and 4.00334/4.00083 in time. The deterministic reduced-unit
example uses 32x24 cells, periods 2pi by 2pi, and 1000 steps of .02. Physical energy
ranges from 9.864762139586677 to 9.869604401089347, while modified energy changes
from 9.864762139074985 to 9.864762139075021. Final magnetic divergence RMS is
9.58e-15. These are achieved values for that case, not universal accuracy guarantees.

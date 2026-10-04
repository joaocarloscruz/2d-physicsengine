# Homogeneous periodic TMz Ohmic evolution

`MaxwellGrid::stepOhmic(duration, conductivity)` explicitly adds scalar homogeneous
Ohmic current to the existing periodic TMz fields. `step(duration)` remains
lossless. Configuration objects and `MaxwellGridDiagnostics` retain their
existing fields. See [the Maxwell spatial operators, units and lossless
invariant](maxwell-grids.md) for the state layout and native model.

The conductivity is a finite nonnegative scalar in S/m. Permittivity is F/m,
permeability H/m, Ez V/m, Hx/Hy A/m, and time seconds. All integrated energy/work
below is J per metre of out-of-plane depth. The constitutive current is
`Jz = sigma*Ez`; it changes the electric equation to

```text
epsilon * Ez_t = Dx- Hy - Dy- Hx - sigma*Ez.
```

Faraday evolution is unchanged. This is a homogeneous linear medium with periodic
boundaries, without a finite-conductor interface, imposed sources, PIC or
charged-particle feedback, thermal-network feedback, geometry coupling or a
thermostat. The returned Joule work is an observation, not a stored temperature.
There is no persistent/cumulative native heat ledger.

Richard Fitzpatrick's primary background describes the
[local Ohm law and volume heating](https://farside.ph.utexas.edu/teaching/em/lectures/node57.html)
and [Poynting energy balance in a linear medium](https://farside.ph.utexas.edu/teaching/em/lectures/node89.html).
For uniform media, current does work `E dot J = sigma*Ez^2`; periodic boundaries
remove net boundary flux. The numerical split, bounds and accounts below are
independently derived for this module, not imported from those continuum sources.

## Split operator and stability

For each accepted equal substep h, let `a = sigma/epsilon` mathematically and let
`W_h` denote the existing symmetric magnetic-half-kick / electric-drift /
magnetic-half-kick map. The Ohmic map is

```text
D_half: Ez <- exp(-a*h/2)*Ez; Hx,Hy unchanged
S_h = D_half * W_h * D_half.
```

The update applies the rightmost decay first. Both decays are exact scalar
subflow solutions before floating-point storage. Symmetric composition with the
second-order wave map is second order for fixed finite conductivity as h tends
to zero. Exact decay stability does not ensure accuracy when `a*h` is large:
strongly damped coupled modes must still be refined in time. Uniform E and
constant H have no wave coupling, so uniform E follows the exponential and H
remains unchanged up to scalar decay storage roundoff.

Write physical electric/magnetic energy as `Ue`, `Uh`, total `H = Ue+Uh`, and

```text
C_h(E) = area*h^2/(8*mu) * sum((Dx+ E)^2 + (Dy+ E)^2)
Q_h = Ue - C_h(E) + Uh.
```

The wave map conserves Q_h in exact arithmetic. The existing strict anisotropic
CFL `eta = c*h*hypot(1/dx,1/dy) < 1` gives
`0 <= C_h(E) <= eta^2*Ue`, so its electric part `Ue-C_h` is positive for nonzero E.
A scalar decay factor r scales both Ue and C_h by r^2 while leaving Uh fixed:

```text
Q_h(before) - Q_h(after decay) = (1-r^2)*(Ue - C_h).
```

Thus each decay contracts Q_h and the wave map is its isometry; their composition
contracts this positive quadratic form without an additional conductivity CFL.
In particular `Q_h <= H <= Q_h/(1-eta^2)` bounds physical field energy for a run
with fixed actual h. Physical H itself can oscillate because W_h does not
conserve H. The same reference Q must be used on both sides. Calls that choose
different h have different quadratic forms and no common-Q contraction claim.
Floating-point arithmetic adds measurement/update roundoff to these identities.

Both Ohmic decays change only E. Magnetic kicks add a discrete curl, so the
existing initial `div H` is preserved in exact arithmetic, including a nonzero
initial defect; no divergence projection is performed. Magnetic means remain
constant; the electric mean decays with the homogeneous scalar exponential up
to update and observer roundoff.

## Separate work accounts

For a decay of duration h/2, define
`f = -expm1(-a*h) = 1-r^2`. Using the **pre-decay represented E field**:

```text
Exact decay-subflow Joule = f*Ue
Exact modified-Q dissipation = f*(Ue - C_h).
```

The Joule expression is the analytic integral of `sigma*E(t)^2` over that scalar
subflow, evaluated from its measured pre-decay electric energy. It is **not the
exact work of the unsplit Maxwell-Ohm PDE over the whole step** at finite h.
Splitting error affects that PDE comparison; the independent damped-mode tests
refine both fields and summed Joule work against the unsplit semidiscrete system.
Physical Joule and Q dissipation differ by `f*C_h` and are not interchangeable.

Separately, each wave stage records measured physical-energy change
`H_after_wave - H_before_wave`. Algebraically this equals its C_h change when
W_h exactly conserves Q_h; in floating point it also has wave/measurement
roundoff. It may have either sign. It is calculated from independently observed
wave endpoints, never from a residual chosen to balance heat.

Each decay also records the difference of measured electric energies before and
after storing its rounded field. This **represented measured loss** can differ
from analytic subflow work. Its discrepancy includes both stored-field rounding
and energy-measurement roundoff. For very small `a*h`, the field can round to its
old value while `expm1` retains a positive, representable Joule observation. The
returned residual reports this; no work or field correction hides it.

`MaxwellOhmicStepDiagnostics` is an owning return value for one successful call:

| Field | Meaning |
| --- | --- |
| `conductivity`, `duration` | Requested S/m and s |
| `startTime`, `endTime` | Published clock interval, s |
| `initialPhysicalEnergy`, `finalPhysicalEnergy` | Measured whole-field H, J/m |
| `exactJouleEnergy` | Sum of analytic **decay-subflow** Joule observations |
| `representedElectricEnergyLoss` | Sum of measured electric pre/post decay differences |
| `wavePhysicalEnergyChange` | Sum of separately measured physical wave-stage changes |
| `modifiedEnergyDissipation` | Sum of analytic fixed-h Q decay losses |
| `decayStorageEnergyChange` | Exact Joule minus represented loss; includes measurement roundoff |
| `physicalBalanceResidual` | `Hfinal - Hinitial + exactJouleEnergy - wavePhysicalEnergyChange` |
| `substep`, `substeps`, `cellVisits` | Accepted equal h, count and work |

The residual is observed, not fed back into `exactJouleEnergy` or state. With
represented loss replacing analytic Joule the stage accounts telescope, up to
measurement/accumulation roundoff. For a fixed h, compare native
`getModifiedEnergy(h)` before and after against `modifiedEnergyDissipation`,
with roundoff in the wave and decay stages. No persistent ledger is reset or
mutated by subsequent lossless steps or state replacement because there is none.

## Work, compatibility and finite range

Positive conductivity uses one initial diagnostic sweep and, per substep, two
E-only decay sweeps, three wave sweeps and three endpoint diagnostic sweeps:
`N*(8*substeps+1)` center visits. Each diagnostic sweep fuses energy, means,
divergence and reference-gradient measurement. The current geometry/budget hard
caps remain in force, and this larger work count is checked before scratch
allocation/iteration. Copies and public array conversion costs are outside that
arithmetic-pass counter, as for the lossless model.

All fields, final legacy diagnostics and the return-only work accounts are
staged before publication. Invalid conductivity/duration, budget/clock failure,
unrepresentable exponent, field/energy/ledger arithmetic, or allocation failure
leaves fields, clock and previous diagnostics unchanged and returns no ledger.
The old configuration and `getDiagnostics` layout remain intact; after an
accepted Ohmic call their work reflects the eight-pass split.

Zero conductivity literally delegates to `step(duration)`, preserving its state,
legacy diagnostics and `N*(3*S+1)` work budget exactly. Its returned Joule and Q
losses are zero, and its wave physical change observes the existing lossless
operation's endpoints; no additional throwing measurement occurs after its
publication. Zero duration validates both scalar inputs, leaves the owner and
prior diagnostics untouched, and returns a zero-work ledger at the current time.

Exponent formation scales mantissas/exponents of h, sigma and epsilon before
applying the factor 1/2. It avoids an unnecessary overflow of `sigma/epsilon`
and avoids prematurely rounding a subnormal h/2 before material scaling.
Small decay decrements use `-expm1(-chi)` and `E - E*decrement`; larger chi uses
`E*exp(-chi)` so a decrement rounded to one cannot erase a supported finite E.
The work fraction also uses `expm1` to retain small losses.

A positive complete exponent that underflows, an exponential factor that rounds
to zero, a nonzero field updated to zero, or a nonzero aggregate decay loss that
underflows is rejected. Existing separate electric/magnetic/correction-energy
range checks still apply at intermediate endpoints. Compensated scalar work
sums and scaled residual assembly retain ordinary cancellation, with explicit
rejection if a normalized ledger term disappears. These are float64 supported-
range restrictions, not a promise for all finite input values. No conductivity,
energy, field or residual is clamped or given an arbitrary physical unit floor.

## Native use and reproducible control

```cpp
PhysicsEngine::MaxwellGrid grid;
auto state = grid.getState();
state.ez.assign(state.ez.size(), 1);
grid.setState(state);
const auto report = grid.stepOhmic(.1, .8); // 0.8 S/m, explicit homogeneous current.
// report is a copy; retain/sum it externally if a cumulative account is wanted.
```

`maxwell_ohmic_demo` initializes a bounded periodic (1,2) mode on 32x24 cells,
with eps=2, mu=1.5, sigma=.8, and 100 steps of .02. It prints JSON containing
physical initial/final energy, analytic split Joule, represented loss, the
separate physical wave defect, Q dissipation, residual, div H and work. It checks
the fixed-h Q contraction identity and physical stage balance without feeding a
correction back into either ledger or fields. These illustrative constants use
reduced units; default material values are not vacuum SI constants.

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel 2
ctest --test-dir build --output-on-failure
build/run_tests '[maxwell][ohmic]'
build/maxwell_ohmic_demo
```

Windows uses `.exe`; multiconfiguration generators require the selected build
configuration and executable subdirectory. The installed external consumer also
checks the named Ohmic call without using repository-private headers.

Validation on the pinned LLVM 23.1.1 Windows toolchain: Release and
ASan/UBSan each passed all 27 CTest entries, including the installed external
consumer and the JSON-parsed demo. After adding the unsplit Joule convergence
controls, each targeted Maxwell run passed 24 cases / 14,780 assertions. The
nine explicitly retained fluid expected failures remain unchanged. Temporal
controls cover all three damping regimes on anisotropic and two-cell grids;
spatial refinement covers both axes and an oblique continuum mode. These are
bounded homogeneous controls, not conductor/interface validation.

## Browser parity

The WASM owner exposes the same `stepOhmic` and all 15 return fields as plain
owning JavaScript values; fields/configuration/legacy diagnostics retain their
existing layouts. See [the browser API and lifetime rules](webassembly.md).
Emscripten 6.0.3 / Node 22.16.0 optimized and ASan/UBSan builds passed the full
smoke plus exception-boundary stress. The focused Ohmic controls include 12
independent damped-mode/heat temporal sequences, three continuum refinements,
split-stage accounting, 200 fixed-h Q checks, nonzero div-H preservation, magnetic
means/electric-mean decay, exact zero-conductivity budget/snapshots, extreme
products and retained copies/replay. Each probes-enabled run also passed 1,000
Ohmic stress batches / 14,000 rejected invocations with unchanged snapshots,
stack, live heap and native uncaught-exception count. Probes-off production
passed the full physical smoke and exported none of the test helpers.

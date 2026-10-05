# Bounded planar contact intervals (#91)

`benchmarks/experimental/planar_contact_interval.h` is a standalone diagnostic
operator for a nonrotating body under one constant load on one fixed infinite
plane. It does not replace `World`, contact resolution, integration or public
configuration, and is not installed. A separate
[complete World pipeline experiment](planar-contact-world.md) now stages this
operator before integration for one eligible box and fixed support. Ordinary
`World::step` retains its existing behavior. Dynamic contact graphs, stacks,
moving supports, joints, CCD and sleeping remain outside the experiment.
**Stacks are not fixed.** All nine #44 expected failures and thresholds remain.

This addresses a narrower failure of the previous
[frozen projection experiment](contact-load-integration.md): integrating only the
loaded/unloaded endpoints gives a trapezoid across a friction stop, although the
true velocity reaches zero partway through the interval. The earlier experiment
remains unchanged, including its `.0125` displacement versus the `.01` stop oracle
and its documented stack/friction regressions.

## Operator and contact contract

The tangent/normal basis satisfies `cross(t,n)=1`. Inputs are double mass,
tangent position/velocity, nonnegative gap and normal velocity, constant tangent
and normal forces, external torque, friction coefficients, two patch offsets
`left < right`, depth `ell >= 0`, and duration. Orientation and angular velocity
are fixed at zero relative to the plane: this is an eligibility assumption,
not a rotational integrator. `mu_s >= mu_k >= 0` is required. `Advance` owns no
body and returns at most two supported intervals or one free interval; exceptions
publish no external state. Invalid inputs and nonfinite/underflowed intermediate
products/ratios raise exceptions. Some extreme finite inputs with representable
final mathematical results are deliberately rejected when this bounded arithmetic
cannot represent its intermediate quantities. There is no physical unit floor.

Exactly closed gap with zero normal velocity and `F_n <= 0` activates sustained
support, including exact touching without creating overlap. The normal reaction
is `N=-F_n`. A separating velocity or an outward load releases into the usual
constant-force ballistic trajectory. No starting overlap or geometric padding is
accepted. A genuinely closing trajectory returns `NeedsImpact` at its first
quadratic gap root, with the pre-impact velocity, zero inferred impact impulse,
and only the traversed free prefix. The separating/inward-acceleration root uses
`(-v-sqrt(D))/a`, avoiding cancellation in `sqrt(D)-v` for a tiny starting gap;
the approaching branch uses `2*gap/(sqrt(D)-v)`. At the event the gap is set to
zero and any represented endpoint offset is exposed as `eventGapCorrection`,
separate from the trajectory's work/kinetic ledger. It is not an energy-balanced
position correction or permission to publish this prefix into a World. A zero-duration call returns the unchanged
initial state. A tangent touch with zero discriminant is a free trajectory,
without a fabricated impulse. Extreme near-grazing classification is limited by
double arithmetic; this is not a CCD replacement.

The World experiment delegates the **entire** step on `NeedsImpact`,
unsupported wrench, dynamic-dynamic contact components or excluded lifecycle
features. It must not publish a partial interval and then call ordinary `step`:
that would duplicate loads, change event timing and confuse the impulse budget.
Current strict-overlap narrowphase is unchanged; the analytical closed-plane
contract here does not establish a new production contact onset policy.

## Sliding, stopping and restarting

For nonzero tangent speed, `T=-mu_k*N*sign(v_t)` and
`a_t=(F_t+T)/m`. When acceleration opposes velocity, the candidate stop time is
`-v_t/a_t`. Only the interval up to that time uses this reaction. At the stop,
velocity is zero; if `|F_t| <= mu_s*N`, sticking uses `T=-F_t`. Otherwise sliding
restarts in `sign(F_t)` with its new kinetic reaction. An exact endpoint stop
returns zero velocity with no extra zero-duration interval. With constant load
and `mu_s>=mu_k`, at most one stop/restart is possible.

Each interval uses its own signed displacement. Its work is
`W_ext=F_t*dx+F_n*dg` (using the recorded trajectory distances before any
event gap correction), `W_friction=T*dx <= 0`. Endpoint kinetic change is measured
independently from velocities, rather than defined as a balancing residual.
For supported motion the analytical relation is `Delta K=W_ext+W_friction`;
free intervals retain the same identity with no reaction. Fixed-wall work is
zero; this is inferred reaction accounting, not implemented coupler feedback.
Roundoff can leave a small measured work residual.

## Feasible two-point support wrench

Contact levers are `r_i=s_i*t-ell*n`. Choose `T_i=(T/N)*N_i` for `N>0`; then both
local Coulomb inequalities follow from the aggregate bound. Required force and
spin balance are

```
N_left + N_right = N
T_left + T_right = T
left*N_left + right*N_right + ell*T = -tau
centerOfPressure = (-tau-ell*T)/N
```

The center must lie in the closed patch interval; otherwise the nonrotating
assumption is infeasible. This check is repeated after the stop, because changing
friction sign can make a previously feasible support tip. `UnsupportedWrench`
returns only the feasible prefix and its state/accounting, never negative normals
or an invented rotational constraint. A free trajectory with nonzero torque is
also unsupported. The reported spin reaction is `-tau*h`, about the body center;
it is not the complete wall torque about an arbitrary world origin. Normals sum
and moment balance are subject to represented double rounding.

## Reproduction and evidence

Base `79575a9`, Windows x86_64 / LLVM-MinGW Clang 23.1.1, Release:

```
cmake --build build --parallel 2
build/run_tests "[contact-interval]"
build/planar_contact_interval_diagnostic > build/planar-interval.json
ctest --test-dir build -R planar_contact_interval_diagnostic --output-on-failure
```

The six diagnostic rows use the **stored float** initial speed `.2f` converted
to double, mass 1, `F_n=-8`, `F_t=tau=0`, `mu_s=mu_k=.25`, patch offsets `+/-.5`,
depth `.5`, duration `.5`, and 1/2/4/8/16/32 equal steps. Independent stop time is
`.10000000149011612`, distance `.010000000298023226`, and final speed zero.
The JSON reports measured reaction impulses, external/friction work, endpoint
kinetic change and residual; it labels production defaults unchanged and
`worldBridge=false`. CTest parses this schema on CMake 3.19+; the 3.16 minimum
retains execution support. JSON is also parsed independently during validation.

Ten control cases pass 1,450 assertions: stop/endpoint/partitioning,
continuous slip, signed restart, shifted center of pressure, a support becoming
infeasible after restart, outward release, returning and linear onset roots,
static threshold, torque exclusion, unit scaling, separating quadratic cancellation at tiny/large scales, and explicit range rejection.
This establishes bounded interval behavior; it does not satisfy the general #91
acceptance criteria. Production adoption still needs actual force-once semantics,
impacts, dynamic-pair angular momentum/energy, joint/CCD/sleep/event contracts,
finite solver residuals, exact-touch geometry, and both unchanged quick/full
benchmark matrices with every regression retained.

Release CTest passes 31/31. ASan+UBSan passes all ten interval cases and 1,450
assertions. The unchanged production executable on this base was also run through
the quick 120-row/7,280-step and full 432-row/45,630-step matrices, both with zero
execution failures; their outputs were parsed as finite JSON.
Those runs establish a reproducible baseline, not a matrix comparison to a World
bridge or evidence that stack drift improves. The six bounded interval rows are
archived in [the measured data](data/planar-contact-interval-79575a9-clang23.json).
They all report the same analytic distance and a zero measured work residual for
this dyadic-load case; general inputs need not have a zero roundoff residual.

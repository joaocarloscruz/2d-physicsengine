# Thermal conduction and radiation networks

`ThermalNetwork`, exposed by `physics/physics.h`, is a standalone graph of lumped
heat capacities. It solves

```
C_i * dT_i/dt = sum_j G_ij * (T_j - T_i)
              + sum_j kappa_ij * (T_j^4 - T_i^4) + P_i
```

Temperature T is absolute Kelvin and must be nonnegative. Heat capacity C is in
J/K and must be strictly positive, finite, and have a finite reciprocal.
Conductance G is in W/K and must be finite and nonnegative. External power P is
in watts; positive power heats, negative power cools. Every node uses double
precision. Nodes and undirected links have stable, append-only indexes and const
state observers; self-links, missing endpoints and duplicate links are rejected.
Zero-coefficient links and disconnected nodes are allowed. Conductive and
radiative links have separate append-only indexes and duplicate-pair checks;
both mechanisms may connect the same pair. Their combined count is bounded by
`maxLinks`, including zero-coefficient links.

There is no automatic coupling to rigid bodies, soft bodies, particles, fluids
or `World`, and no convection, fluid advection, latent heat, phase changes,
temperature-dependent capacities or mechanical work. Native and owned WASM APIs
are available. Applications supply nodes and effective exchange coefficients;
the engine does not calculate geometry, view factors or radiation transport.

## Example

```cpp
#include "physics/physics.h"
using namespace PhysicsEngine;

ThermalNetwork graph;
graph.addNode(280.0, 100.0);        // Finite body, 100 J/K.
graph.addNode(350.0, 100.0, true);  // Thermostat holds 350 K.
graph.addLink(0, 1, 2.0);          // 2 W/K.
graph.applyPower(0, 5.0);          // 5 W for this call's entire duration.
graph.step(0.1);
const auto diagnostics = graph.getDiagnostics();
```

The headless `thermal_network_demo` target reapplies 5 W each outer step for 120
seconds, then prints temperature, supplied energies and the energy-budget
residual. It builds with `PHYSICS_BUILD_EXAMPLES=ON` and has a CTest smoke check.

`applyPower` accumulates loads in double precision. Pending power is constant
across every substep and clears only after a successful positive-duration step.
Failed and zero-duration steps retain it. `clearPowers()` clears every pending
load; `clearPowers(index)` clears one. Fixed nodes accept loads as well.
`setTemperature` can directly change a node or reservoir temperature;
`setFixed` pins or unpins the current temperature. These setters retain queued
power and historical accounting. Heat capacities and links are immutable.

## Integration and temperature bounds

The integrator is first-order explicit Euler, using old temperatures for all
equal and opposite link transfers in each substep. It partitions the outer
timestep uniformly when there are no radiative links and chooses

```
h <= min(maxSubstep, safetyFactor * C_i / sum_j G_ij)
```

for every dynamic node with nonzero incident conductance. Defaults are
`maxSubstep=0.01`, `safetyFactor=0.9`, `maxSubsteps=4096`, `maxNodes=100000` and
`maxLinks=300000`. The safety factor must be in `(0,1]`. Reservoir nodes do not
restrict the conduction timestep, because their temperature is held constant.

The implementation rounds conductance row sums and rates conservatively upward,
then applies a small degree-dependent floating-point margin to the bound. The actual partition
timestep must satisfy that representable conduction limit; tolerance for decimal
`maxSubstep` partitions cannot enlarge the physical bound. At an exact theoretical
boundary, this can require one more substep than real-arithmetic division suggests,
including when `safetyFactor=1`. It avoids negative temperatures caused solely by
rounding an outgoing heat transfer above a node's available energy. Temperatures
are not clipped, and actual excessive external cooling is still rejected.

With zero external power, the update is a convex combination of the old node and
neighbour temperatures. It therefore obeys the conduction maximum principle:
temperatures stay between the previous minimum and maximum, including reservoirs,
and nonnegative Kelvin temperatures remain nonnegative, up to floating-point
rounding. This bound depends on the sum of all incident conductances, rather than
only the strongest individual link. Positive external power may increase the
maximum. Negative power can remove too much energy; a step producing negative
Kelvin is rejected without clipping. Reduce its duration or cooling power.
This conduction bound controls stability, rather than prescribing accuracy;
refine the timestep for accuracy. Tests verify analytic two-node exponential
relaxation and first-order convergence. Work is O(nodes + links) per substep.

## Reciprocal radiative exchange

`addRadiationLink(first, second, coefficient)` supplies an effective nonnegative
`kappa` in W/K^4. A link contributes `kappa*(T_b^4-T_a^4)` watts to its first
node and the exact opposite transfer to its second. For a gray body facing a
large black enclosure, the model uses `kappa=emissivity*sigma*area`. The rounded
SI [Stefan–Boltzmann constant](https://physics.nist.gov/cuu/Constants/Table/allascii.txt) is
`sigma=5.670374419e-8 W/(m^2 K^4)`; see NASA's
[radiation law](https://asd.gsfc.nasa.gov/archive/mwmw/mmw_bbody.html).
General surfaces require reciprocal view-factor and emissivity treatment before
supplying this effective coefficient. An arbitrary graph of such links does not
automatically model multiple reflections, participating media, spectral effects,
finite light travel time, or an electromagnetic field. Coefficients are constant.

The positive secant conductance is

```
G_rad = kappa*(T_a+T_b)*(T_a^2+T_b^2)
      <= 4*kappa*max(T_a,T_b)^3.
```

When radiative links are present, each substep recomputes the combined conductive
and radiative row-rate bound from the current staged temperatures. The selected
interval obeys `h*sum_j(G_ij+G_rad_bound_ij)/C_i <= safetyFactor`, with directed
rounding and a degree-dependent margin. This permits a convex update without
external loads and prevents earlier queued heating from invalidating a frozen
radiative bound. The bound is conservative even for equal temperatures, where
the actual exchange is zero. Fixed nodes do not restrict stability, but their
exchanges and queued power are still accounted. No temperature is clipped.

This nonlinear path uses adaptive, generally unequal intervals; it retains the
outer call's constant queued power. The represented interval is checked after
time addition, so it cannot round above the physical bound. Insufficient time
resolution, `maxSubsteps`, or the hard `MaximumRadiativeVisits=100000000` budget
rejects the whole call, even after earlier staged substeps succeeded. The work
charge per substep is `6*nodes + 2*(conductiveLinks+radiativeLinks)`, including
scratch initialization, rate reduction, load/update passes and edge visits.
Copies and final accounting are separately bounded by this topology. Successful
calls report `lastRadiativeVisits`; it is zero for conduction-only and zero-time
calls. Zero time retains queued powers and resets last-call values as before.

Transfers factor the temperature difference as
`(T_b-T_a)*T_high^3*(1+r)*(1+r^2)`, where `r=T_low/T_high`, then scale all factors
with binary exponents. This avoids intermediate fourth-power overflow or
underflow and cancellation between nearly equal fourth powers. Complete
unrepresentable rates, transfers, temperatures or ledgers still reject the call;
subnormal transfers may round to zero. A finite input alone is not sufficient
to guarantee a representable step. State storage and accumulation introduce
roundoff; conservation is checked to that precision, not repaired by rescaling.

For cooling into a fixed zero-K reservoir the independent solution is
`T(t)=T0/(1+3*kappa*T0^3*t/C)^(1/3)`. The `thermal_radiation_demo` prints
temperatures, errors, refinement ratios and energy residuals for three timesteps.
Tests also cover pair/mixed-graph energy, stationary equilibrium, maximum
principle, load-induced stiffness changes, reservoir work, deterministic replay,
range failures and late rollback. These are lumped temperature tests; no spatial
continuum convergence is implied.

```cpp
ThermalNetwork radiator;
radiator.addNode(500, 20);       // 20 J/K body.
radiator.addNode(0, 1, true);    // Ideal cold enclosure, fixed at 0 K.
radiator.addRadiationLink(0, 1, .8 * 5.670374419e-8 * .02);
radiator.step(.1);
```

JavaScript exposes the same radiation methods and owned copied link, node and
diagnostic snapshots. See the [WASM example and lifetime contract](webassembly.md#owned-thermal-networks).

## Energy accounting

`totalEnergy` is `sum(C*T)` in joules and includes the constant finite reference
energies of fixed nodes. This model treats C as temperature independent and uses
zero Kelvin as the energy reference; it does not model absolute chemical energy.
`totalExternalEnergy` integrates every queued power, including fixed-node loads.
`totalReservoirHeat` records heat supplied by thermostats to keep fixed nodes at
their temperatures. Positive values mean heat entered the graph; negative values
mean thermostats removed heat. Each node's `reservoirHeat` records its own
cumulative thermostat exchange, including exchanges between two fixed nodes.
Heating applied directly to a fixed node is offset by equal thermostat removal.

For accepted steps with no direct topology or temperature edits between the
measurements, the budget is

```
change(totalEnergy) = change(totalExternalEnergy) + change(totalReservoirHeat)
```

up to roundoff. Pair conduction and radiation conserve isolated energy. Direct temperature
setters and adding nodes change stored energy outside the integration ledger;
they are caller-controlled edits, not heat inputs recorded by `step`. Pinning or
unpinning does not change stored energy. Last-call external/reservoir energies
and substeps are also reported; successful zero-duration or empty steps report
zero last-call values without resetting cumulative ledgers. Empty graphs report
zero minimum/maximum temperature. Copies own independent state and ledgers.

## Failure behavior

Invalid finite/domain/configuration arguments throw `std::invalid_argument`;
missing indexes throw `std::out_of_range`, duplicate links throw
`std::invalid_argument`, and collection-budget violations throw
`std::length_error`. Each node's `C*T` must be finite when added or set.
Derived arithmetic overflow, aggregate diagnostic energy overflow, crossing
zero Kelvin, timestep underflow, or a substep budget violation throws
`std::runtime_error`. Every failed `step` preserves all node temperatures,
queued powers, reservoir exchanges and diagnostics. Power accumulation overflow
also leaves the pending load unchanged. No intermediate partial step is exposed.
Extreme finite inputs may overflow derived rates, fluxes or ledgers and are
rejected. Link transfers use exponent-scaled multiplication of `h*G*deltaT`, so
a finite full transfer is not rejected merely because an intermediate `G*deltaT`
or `h*G` overflows; the same calculation avoids losing a representable transfer
through intermediate underflow. A genuinely overflowing complete transfer is
rejected transactionally. Identical inputs and builds use deterministic insertion order;
cross-platform bitwise reproduction is not promised.

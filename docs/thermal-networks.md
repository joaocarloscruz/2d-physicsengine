# Thermal conduction networks

`ThermalNetwork`, exposed by `physics/physics.h`, is a standalone graph of lumped
heat capacities. It solves

```
C_i * dT_i/dt = sum_j G_ij * (T_j - T_i) + P_i
```

Temperature T is absolute Kelvin and must be nonnegative. Heat capacity C is in
J/K and must be strictly positive, finite, and have a finite reciprocal.
Conductance G is in W/K and must be finite and nonnegative. External power P is
in watts; positive power heats, negative power cools. Every node uses double
precision. Nodes and undirected links have stable, append-only indexes and const
state observers; self-links, missing endpoints and duplicate links are rejected.
Zero-conductance links and disconnected nodes are allowed.

This is a conduction foundation rather than a complete thermodynamic model.
There is no automatic coupling to rigid bodies, soft bodies, particles, fluids
or `World`, and no radiation, convection, fluid advection, latent heat, phase
changes, temperature-dependent capacities, mechanical work or WASM binding.
Applications choose their own mapping from objects to nodes and conductances.

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
timestep uniformly and chooses

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

up to roundoff. Pair conduction conserves isolated energy. Direct temperature
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

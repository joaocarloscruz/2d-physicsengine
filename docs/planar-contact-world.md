# Experimental planar intervals through the World pipeline

`planar_contact_world_benchmark` compares the ordinary World with the internal
`benchmarks/experimental/planar_contact_bridge.h` strategy. The strategy uses the
[bounded interval operator](planar-contact-interval.md) inside the actual World
pipeline. It is not installed, has no public configuration flag, and does not
change ordinary `World::step` or resolve the stack regressions in #91.

## Publication and lifecycle

World invokes the strategy once after registered and universal forces have
accumulated. Staging reads those loads and proposes an endpoint, optional closed
two-point manifold and integrated reaction impulses. Engine-owned snapshots
check body identity and unchanged state/properties before publication. The
accepted body skips ordinary integration and clears its loads once. The bridge
also preflights float endpoints, contact coordinates and local anchor range.

Closed contacts enter the normal canonical body ordering, cache, island and
Begin/Persist/End event paths even when strict-overlap geometry omits exact
touching. Integrated feature impulses populate the cache. Those already solved
constraints skip warm starting and iterative velocity/position corrections,
preventing double application. Release removes the old cache and emits End.
The reported solver iteration count is zero for these accepted components;
broad-phase statistics count the merged candidates, including exact-touch pairs.
`proposed` and `selected` distinguish a strategy proposal from engine acceptance.

This requires exactly two bodies: an aligned nonrotating rectangle and stationary
fixed rectangular support. The entire tangential interval, including interior
turning points, must remain over the support. Touching must be exactly represented;
overlap, padding and tolerance-based contact onset are excluded. Friction is the
solver's represented geometric-mean mixing, with mixed static friction at least
mixed dynamic friction. Constant Gravity and already queued manual loads are
supported. Opaque force generators are rejected after their one ordinary call.

Joints, particles, CCD, sleeping, velocity caps, warm starting, moving supports,
rotation, tipping and unresolved closing impacts delegate the whole step to the
ordinary path. No partial interval is published before fallback, and forces are
not applied a second time. The experiment does not make World transactional:
exceptions after ordinary integration retain the ordinary partial-step behavior.

## Recorded complete-step comparison

[Raw 52-row output](data/planar-world-d9afeb3-clang23.json) was generated from
native code at `d9afeb3` with Windows Clang 23.1.1 Release. It covers 13 fixtures,
dt 1/64 and 1/128, and 4/10 configured solver iterations for two physical seconds:
19,968 total World calls across both pipelines, with zero execution failures.
Timings include complete World calls and event dispatch; they are nondeterministic.

Representative dt 1/64, 4-iteration results:

| Fixture | Ordinary World | Experimental World |
| --- | --- | --- |
| Rest | final y 0.4945737422; peak penetration 0.0055481586 | y 0.5; zero penetration and spin |
| Stop from stored 0.2 m/s | final x 0.0117904525, vx 0.0000757064 | x 0.0099999998, vx 0 |
| Outward release after 1 s | final y 4.4945735931, vy 8 | y 4.5, vy 8; Begin 1 / Persist 63 / End 1 |
| Opaque force generator | ordinary trajectory | identical trajectory; one generator call per step, zero selected intervals |

The rest and stop paths each accept 128 intervals and record Begin 1 / Persist
127 with no feature changes or iterative impulse reapplication. The stop's
stored-state work and momentum residuals are zero in this row; its peak reaction
torque residual is 1.63e-9 from represented contact coordinates. These are measured
float-state results, separate from the double interval operator's exact formulas.

The original rigid-contact benchmark was compared against immutable `2394015`:
all 120 quick rows and 432 full rows match exactly after removing elapsed time.
That preserves the earlier production gains and regressions. The new benchmark
and `[contact-bridge]` tests cover actual lifecycle/cache/load behavior and
fallback. A finite run or a passing execution status does not establish a
general many-body contact solution.

```sh
cmake --build build --target planar_contact_world_benchmark
build/planar_contact_world_benchmark --quick
build/planar_contact_world_benchmark --output planar-world.json
```

General contact graphs require a coupled support/load solution, impact onset,
rotation and compatible warm-start/CCD/sleep policies. Those remain open in #91.

# Experimental planar intervals through the World pipeline

> **Measured on `526570c` (2026-10-28), Windows x86_64, Release, Clang 23.1.4,
> with the default configuration** (speculative step order, `solverSubsteps = 4`).
> The "ordinary World" in the comparison below is therefore the current default
> solver. A second ordinary column gives the legacy step order
> (`enableSpeculativeContacts = false`) for comparison with the earlier numbers of
> this page; those values come from the unmodified benchmark source compiled with a
> locally force-included copy of `simulation_config.h` that reads the step order
> from an environment variable (a measurement harness that is not in the
> repository). The experimental strategy is unchanged and its numbers match the
> earlier ones.

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

The earlier [raw 52-row output](data/planar-world-d9afeb3-clang23.json) was
generated from native code at `d9afeb3` (Clang 23.1.1) and is not regenerated here;
the raw rows for the numbers below were kept outside the repository. The benchmark
covers 13 fixtures, dt 1/64 and 1/128, and 4/10 configured solver iterations for
two physical seconds: 52 rows and 19,968 total World calls across both pipelines,
with zero execution failures, as before (`--quick` runs 13 rows and 832 calls,
also with zero failures). Timings include complete World calls and event
dispatch; they are nondeterministic.

Representative dt 1/64, 4-iteration results:

| Fixture | Ordinary World, default | Ordinary World, legacy order | Experimental World |
| --- | --- | --- | --- |
| Rest | final y 0.5; zero penetration | final y 0.4947558641; peak penetration 0.0052441359 | y 0.5; zero penetration and spin |
| Stop from stored 0.2 m/s | final x 0.0098425914, vx -0.0000660014; peak penetration 0.0001255608 | final x 0.0117902160, vx 0.0000755970; peak penetration 0.0052462992 | x 0.0099999998, vx 0 |
| Outward release after 1 s | final y 4.5, vy 8; Begin 1 / Persist 64 / End 1 | final y 4.4947557450, vy 8; Begin 1 / Persist 65 / End 1 | y 4.5, vy 8; Begin 1 / Persist 63 / End 1 |
| Opaque force generator | y 0.5; 128 generator calls in 128 steps | y 0.4947558641; 128 generator calls | identical trajectory to the ordinary run; one generator call per step, zero selected intervals |

What changed: the default ordinary World now rests exactly at y 0.5 with no
penetration and releases at y 4.5, so the experimental strategy no longer has an
advantage in those two rows; the legacy-order rows reproduce the old pattern
(sinking to about the penetration slop). The stop row still differs: the legacy
order stopped at x 0.01179 (18% beyond the stopping distance .01), the default
stops at x 0.00984 (1.6% short) with a nonzero residual velocity of -6.6e-5, and
the experimental strategy gives x 0.0099999998 and zero velocity. In the slide
row the default ordinary World ends at x 0.9999209 while the experimental
strategy gives x 1. So the interval operator still ends closer to the analytic
sliding and stopping results, but it is no longer needed to avoid penetration at
rest.

The rest and stop paths each accept 128 intervals (selected steps 128 of 128) and
record Begin 1 / Persist 127 with no feature changes or iterative impulse
reapplication. The stop's
stored-state work and momentum residuals are zero in this row; its peak reaction
torque residual is 1.63e-9 from represented contact coordinates. These are measured
float-state results, separate from the double interval operator's exact formulas.

When the bridge was added, the original rigid-contact benchmark was compared
against immutable `2394015`: all 120 quick rows and 432 full rows matched exactly
after removing elapsed time. That comparison was against a much older solver and
was not repeated for this refresh. The new benchmark and `[contact-bridge]` tests
cover actual lifecycle/cache/load behavior and fallback; on `526570c` the native
suite passes in full (813 cases) and the `planar_contact_world_benchmark_smoke`
CTest entry passes. A finite run or a passing execution status does not establish
a general many-body contact solution.

```sh
cmake --build build --target planar_contact_world_benchmark
build/planar_contact_world_benchmark --quick
build/planar_contact_world_benchmark --output planar-world.json
```

General contact graphs require a coupled support/load solution, impact onset,
rotation and compatible warm-start/CCD/sleep policies. Those remain open in #91.

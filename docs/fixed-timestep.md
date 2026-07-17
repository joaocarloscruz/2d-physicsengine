# Fixed timestep execution

`World::step()` advances exactly once using `SimulationConfig::fixedTimeStep`.
The existing `World::step(deltaTime)` overload remains available for callers
that intentionally control each numerical step.

For variable wall-clock or frame time, use `FixedStepRunner` or
`Engine::advance(elapsedTime)`. The runner accumulates elapsed seconds and
executes no more than `maxSubstepsPerAdvance` on one call:

```cpp
PhysicsEngine::Engine engine;
PhysicsEngine::SimulationConfig config = engine.getSimulationConfig();
config.fixedTimeStep = 1.0f / 120.0f;
config.maxSubstepsPerAdvance = 8;
engine.setSimulationConfig(config);

const PhysicsEngine::FixedStepResult progress = engine.advance(frameTime);
```

`FixedStepResult` reports:

| Field | Meaning |
| --- | --- |
| `stepsPerformed` | Fixed steps executed by this call |
| `simulatedTime` | Time advanced by those steps |
| `remainingTime` | Full unprocessed backlog retained by the runner |
| `interpolationAlpha` | Fractional remainder divided by the fixed timestep |

The substep cap bounds per-call work; it never drops elapsed time. Call
`advance(0)` to process retained backlog without adding more time. Use
`getTotalStepCount()` and `getAccumulatedTime()` for cumulative observation,
or `resetTiming()` to clear both counters without changing simulation state.

Elapsed time is accumulated in double precision. Because the configured fixed
timestep is a float, comparisons use a boundary tolerance of
`fixedTimeStep * 1e-6`. Equivalent elapsed totals therefore produce the same
step sequence once their backlog is processed, with a scheduler boundary error
no greater than that tolerance. The native invariant test also requires
equivalent position results within `1e-6` world units.

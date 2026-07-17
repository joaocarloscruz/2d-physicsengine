# Per-step simulation statistics

After every successful `World::step`, callers can inspect the immutable
`SimulationStatistics` snapshot returned by `getLastStepStatistics()`.
`Engine` and the WebAssembly API expose the same value object.

| Field | Meaning |
| --- | --- |
| `integratedBodyCount` | Non-static rigid bodies integrated |
| `integratedParticleCount` | Particles integrated across all particle systems |
| `broadPhaseCandidateCount` | Pairs generated, summed across solver iterations |
| `narrowPhaseCandidateCount` | Pairs left after collision filtering, summed across iterations |
| `resolvedContactCount` | Contact resolutions performed across solver iterations |
| `solverIterationCount` | Rigid contact solver iterations executed |
| `activeContactCount` | Unique contacts active at the end of the step |
| `fluidIterationCount` | Fluid solver iterations; zero until a fluid solver is configured |

Counts describe work performed, so a single persistent contact can contribute
multiple resolved contacts when the solver runs multiple iterations. The
active contact count is the unique physical-pair view.

Each successful step replaces the whole snapshot; values never accumulate
across steps. A rejected timestep or another exception preserves the most
recent completed snapshot instead of exposing partial work. When
`Engine::advance` performs several fixed substeps, the snapshot describes its
last completed substep. Use the fixed-step result for the number of substeps
performed by the advance call.

```cpp
world.step();
const PhysicsEngine::SimulationStatistics& statistics =
    world.getLastStepStatistics();
```

```javascript
const progress = engine.advance(frameTimeSeconds);
const statistics = engine.getLastStepStatistics();
console.log(progress.stepsPerformed, statistics.resolvedContactCount);
```

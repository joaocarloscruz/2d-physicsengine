# Simulation configuration

`SimulationConfig` makes the engine's numerical policy explicit. A `World`
validates and copies the configuration, so callers cannot partially apply an
invalid update.

```cpp
PhysicsEngine::World world;
PhysicsEngine::SimulationConfig config = world.getSimulationConfig();

config.solverIterations = 16;
config.positionCorrectionFactor = 0.6f;
config.penetrationSlop = 0.002f;
config.warmStartFactor = 0.9f;
config.enableLinearVelocityLimit = false;
config.enableAngularVelocityLimit = false;

world.setSimulationConfig(config);
```

The defaults preserve the behavior that predates this configuration API:

| Setting | Default |
| --- | ---: |
| Solver iterations | `10` |
| Position correction factor | `0.8` |
| Penetration slop | `0.005` |
| Warm-start factor | `0.8` |
| Linear velocity limit | enabled at `200` |
| Angular velocity limit | enabled at `30` |

Correction and warm-start factors must be finite values between zero and one.
Penetration slop must be finite and non-negative. Enabled velocity limits must
be positive and finite. Set either enable flag to `false` when the simulation
must not silently clamp that velocity.

The same value object is available through WebAssembly using
`engine.getSimulationConfig()` and `engine.setSimulationConfig(config)`.

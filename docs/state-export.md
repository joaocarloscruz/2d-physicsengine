# State export and replay

`ExportWorldJson(world, time)` includes `schemaVersion: 1`, caller-supplied simulation
time, all step statistics, rigid-body IDs/poses/velocities/mass/inertia/shape geometry,
sleep/CCD flags and particle-system positions/velocities. IDs are decimal strings to
preserve uint64 precision in JavaScript. `ExportWorldCsv` emits one row per body and
selected work counters; an empty world emits only a header. `Engine::exportJson`
and `exportCsv` are also exposed to JavaScript (pass the time argument explicitly).

`ExportFluidJson(particles, diagnostics, time)` includes fluid state and all solver
diagnostics. `ExportFluidCsv` exports particle states by vector index. Fluid indices
are not stable IDs if the application reorders its particles. All writers use the
classic locale, decimal dots and sufficient precision to round-trip floats.
They reject non-finite numeric values with `invalid_argument`.

Exports are measurement snapshots, not complete restart checkpoints. They do not
serialize force-generator code, contact impulse caches, joint definitions, timing
backlogs or random-number-generator state. To replay the included experiment, run
`replay_experiment` again; its initial scene, forces and fixed timestep are defined
in `examples/replay_experiment.cpp`. The replay CTest checks that two fresh processes
produce identical JSON and CSV. Cross-compiler bitwise equality is not guaranteed.

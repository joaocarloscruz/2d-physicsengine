const assert = require('node:assert/strict');
const createModule = require('./physics_engine.js');

(async () => {
    const physics = await createModule();
    assert.equal(typeof physics.boundaryTestStats, 'function', 'configure PHYSICS_WASM_BOUNDARY_TEST_PROBES=ON');
    const stack = () => physics._emscripten_stack_get_current();
    const probe = new physics.BoundaryTestProbe({number: 1});
    const ordinary = new Error('original JavaScript getter failure');
    const badGetter = {get number() { throw ordinary; }};
    const nested = {get number() {
        assert.throws(() => probe.method({number: 0}), error => error instanceof Error && /probe method/.test(error.message));
        return 4;
    }};
    const failedNested = {get number() { probe.method({number: 0}); }};
    const failures = [
        () => new physics.BoundaryTestProbe({number: -1}),
        () => probe.method({number: 0}),
        () => probe.value,
        () => { probe.value = {number: 0}; },
        () => physics.BoundaryTestProbe.function({number: 0}),
        () => physics.boundaryTestFunction({number: 0}),
        () => physics.BoundaryTestProbe.copy(badGetter),
        () => { physics.BoundaryTestProbe.staticValue = badGetter; },
        () => physics.BoundaryTestProbe.copy(failedNested),
    ];
    function batch() {
        for (const fail of failures) {
            const before = stack();
            assert.throws(fail, error => error instanceof Error && !('excPtr' in error));
            assert.equal(stack(), before);
        }
        assert.throws(() => physics.BoundaryTestProbe.copy(badGetter), error => error === ordinary);
        assert.throws(() => probe.method({}), error => error instanceof TypeError);
        assert.equal(probe.method(), 7); // overload, preserved this
        const result = physics.BoundaryTestProbe.copy(nested);
        assert.deepEqual(result, {number: 4}); // nested invocation and owning return
        result.number = 100;
        physics.BoundaryTestProbe.staticValue = {number: 3};
        assert.deepEqual(physics.BoundaryTestProbe.staticValue, {number: 3});
    }
    // Warm the allocator and conversion/error paths, then compare live allocated
    // bytes (not memory capacity or a timing-dependent high-water mark).
    for (let i = 0; i < 20; ++i) batch();
    const initial = physics.boundaryTestStats(), initialStack = stack();
    assert.equal(initial.uncaught, 0);
    for (let i = 0; i < 2000; ++i) batch();
    const final = physics.boundaryTestStats();
    assert.deepEqual(final, initial);
    assert.equal(stack(), initialStack);
    const grid = new physics.PeriodicMacGrid({columns: 2, rows: 2, spacingX: 1, spacingY: 1});
    const wave = new physics.WaveMembrane(3, 3, 1, 1);
    const gridBefore = grid.getVelocities(), waveBefore = wave.getDisplacements();
    function accessor(size, throws) {
        const values = Array(size).fill(0);
        Object.defineProperty(values, size - 1, {get: () => { throw throws; }});
        return values;
    }
    const proxy = new Proxy(Array(4).fill(0), {get(target, key) {
        if (key === '3') throw ordinary;
        return Reflect.get(target, key);
    }});
    // Shadowed dimension observers must never control bounded preprocessing.
    grid.getConfig = () => ({columns: 1e20, rows: 1e20});
    wave.getCellCount = () => 1e20;
    function foreignBatch() {
        assert.throws(() => grid.setVelocities(accessor(4, ordinary), Array(4).fill(0)), error => error === ordinary);
        assert.throws(() => grid.setVelocities(proxy, Array(4).fill(0)), error => error === ordinary);
        assert.throws(() => wave.setState(accessor(9, ordinary), Array(9).fill(0)), error => error === ordinary);
        assert.throws(() => grid.setVelocities(accessor(4, 73), Array(4).fill(0)), error => error === 73);
        const failedReentrant = Array(4).fill(0);
        Object.defineProperty(failedReentrant, 3, {get: () => { probe.method({number: 0}); }});
        assert.throws(() => grid.setVelocities(failedReentrant, Array(4).fill(0)), error => error instanceof Error && /probe method/.test(error.message));
        const reentrant = Array(4).fill(0);
        Object.defineProperty(reentrant, 3, {get: () => { assert.equal(probe.method(), 7); return 0; }});
        grid.setVelocities(reentrant, Array(4).fill(0));
    }
    for (let i = 0; i < 20; ++i) foreignBatch();
    const foreignBefore = physics.boundaryTestStats(), foreignStack = stack();
    for (let i = 0; i < 1000; ++i) foreignBatch();
    assert.deepEqual(physics.boundaryTestStats(), foreignBefore);
    assert.equal(stack(), foreignStack);
    assert.deepEqual(grid.getVelocities(), gridBefore);
    assert.deepEqual(wave.getDisplacements(), waveBefore);
    grid.delete(); wave.delete();
    function deletingBatch() {
        const receiver = new physics.PeriodicMacGrid({columns: 2, rows: 2, spacingX: 1, spacingY: 1});
        const values = Array(4).fill(0);
        Object.defineProperty(values, 3, {get: () => { receiver.delete(); return 0; }});
        assert.throws(() => receiver.setVelocities(values, Array(4).fill(0)), error => error instanceof Error);
        assert.equal(receiver.isDeleted(), true);
    }
    for (let i = 0; i < 20; ++i) deletingBatch();
    const deleteBefore = physics.boundaryTestStats(), deleteStack = stack();
    for (let i = 0; i < 1000; ++i) deletingBatch();
    assert.deepEqual(physics.boundaryTestStats(), deleteBefore);
    assert.equal(stack(), deleteStack);
    probe.delete();
    assert.equal(physics.boundaryTestStats().objects, 0);
    assert.equal(physics.boundaryTestStats().values, 1); // test static field only
    console.log(`PASS: WASM boundary stress; 2000 batches, stable stack=${initialStack}, live heap=${final.heap}, uncaught=0`);
})().catch(error => { console.error(error); process.exitCode = 1; });

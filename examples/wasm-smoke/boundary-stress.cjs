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
    if (typeof physics.MaxwellGrid === 'function') {
        const config = {columns:2, rows:2, spacingX:1, spacingY:1, permittivity:1,
            permeability:1, cflSafety:.9, maxSubstep:1, maximumSubsteps:10000,
            maximumCellVisits:100000000};
        const maxwell = new physics.MaxwellGrid(config);
        const budget = new physics.MaxwellGrid({...config, maximumCellVisits:15});
        const large = new physics.MaxwellGrid(config);
        const largeFields = {ez:[], hx:[], hy:[]};
        for (let j=0; j<2; ++j) for (let i=0; i<2; ++i) {
            const theta = Math.PI*(i+j)+.31;
            largeFields.ez.push(0);
            largeFields.hx.push(6e153*Math.sin(theta+Math.PI/2));
            largeFields.hy.push(-6e153*Math.sin(theta+Math.PI/2));
        }
        large.setState(largeFields);
        const owners = [maxwell,budget,large];
        const fieldsBefore = owners.map(owner => owner.getState());
        const diagnosticsBefore = owners.map(owner => owner.getDiagnostics());
        // The private native observer must bypass shadowed public methods.
        maxwell.getConfig = () => ({columns:1e20,rows:1e20});
        const zero = () => Array(4).fill(0);
        const fieldState = () => ({ez:zero(),hx:zero(),hy:zero()});
        const maxwellProxy = new Proxy(zero(), {get(target,key) {
            if (key === '3') throw ordinary;
            return Reflect.get(target,key);
        }});
        const fieldGetter = {ez:zero(),hx:zero(),get hy() {throw ordinary;}};
        const maxwellFailures = [
            () => budget.step(.01), // native work budget exception
            () => large.step(.6), // native late energy exception after staged updates
            () => maxwell.setState({ez:accessor(4,ordinary),hx:zero(),hy:zero()}),
            () => maxwell.setState({ez:zero(),hx:maxwellProxy,hy:zero()}),
            () => maxwell.setState(fieldGetter),
            () => maxwell.setState({ez:zero(),hx:zero(),hy:accessor(4,73)}),
            () => {
                const s=fieldState();
                Object.defineProperty(s.hy,3,{get:()=>{budget.step(.01);}});
                maxwell.setState(s);
            },
        ];
        function maxwellBatch() {
            for (let i=0; i<maxwellFailures.length; ++i) {
                const before=stack();
                assert.throws(maxwellFailures[i], error => i===5 ? error===73 :
                    error instanceof Error && !('excPtr' in error));
                assert.equal(stack(),before);
            }
            assert.throws(()=>maxwell.setState(fieldGetter),error=>error===ordinary);
            const s=fieldState();
            let reads=0;
            Object.defineProperty(s.ez,3,{get:()=>{++reads;throw ordinary;}});
            s.hy=[];
            assert.throws(()=>maxwell.setState(s),error=>error instanceof RangeError);
            assert.equal(reads,0); // all three shapes precede any entry
            const successful=fieldState();
            Object.defineProperty(successful.hy,3,{get:()=>{assert.equal(probe.method(),7);return 0;}});
            maxwell.setState(successful); // nested success with copied numeric entries
            const receiver=new physics.MaxwellGrid(config), deleting=fieldState();
            Object.defineProperty(deleting.hy,3,{get:()=>{receiver.delete();return 0;}});
            assert.throws(()=>receiver.setState(deleting),error=>error instanceof Error);
            assert.equal(receiver.isDeleted(),true);
        }
        for (let i=0; i<20; ++i) maxwellBatch();
        const maxwellBefore=physics.boundaryTestStats(),maxwellStack=stack();
        for (let i=0; i<1000; ++i) maxwellBatch();
        assert.deepEqual(physics.boundaryTestStats(),maxwellBefore);
        assert.equal(stack(),maxwellStack);
        for (let i=0; i<owners.length; ++i) {
            assert.deepEqual(owners[i].getState(),fieldsBefore[i]);
            assert.deepEqual(owners[i].getDiagnostics(),diagnosticsBefore[i]);
            owners[i].delete();
        }
        console.log(`PASS: Maxwell boundary stress; 1000 batches/10000 rejection checks, stack=${maxwellStack}, live heap=${maxwellBefore.heap}, uncaught=${maxwellBefore.uncaught}; unchanged fields/diagnostics`);
    }
    if (typeof physics.PeriodicScalarTransport === 'function') {
        const config={columns:2,rows:2,spacingX:1,spacingY:1};
        const options={cflSafety:.9,maxSubstep:.1,maximumSubsteps:10000,maximumCellVisits:100000000};
        const scalar=new physics.PeriodicScalarTransport(config);
        const late=new physics.PeriodicScalarTransport({...config,spacingX:1e-154,spacingY:1e-154});
        late.setState(Array(4).fill(1e308));
        late.setVelocities([1e-154,-1e-154,1e-154,-1e-154],Array(4).fill(0));
        const owners=[scalar,late];
        const snapshot=owner=>({q:owner.getState(),v:owner.getVelocities(),d:owner.getLastStep(),time:owner.getTime()});
        const states=owners.map(snapshot);
        const nativeConfig=physics.PeriodicScalarTransport.prototype.getConfig;
        physics.PeriodicScalarTransport.prototype.getConfig=()=>({columns:1e20,rows:1e20});
        scalar.getConfig=()=>({columns:1e20,rows:1e20});
        const zero=()=>Array(4).fill(0);
        const proxy=new Proxy(zero(),{get(target,key) {
            if(key==='3') throw ordinary;
            return Reflect.get(target,key);
        }});
        const hasProxy=new Proxy(zero(),{getOwnPropertyDescriptor(target,key) {
            if(key==='3') throw ordinary;
            return Reflect.getOwnPropertyDescriptor(target,key);
        }});
        const lengthProxy=new Proxy(zero(),{get(target,key) {
            if(key==='length') throw ordinary;
            return Reflect.get(target,key);
        }});
        const optionGetter={...options,get maximumCellVisits(){throw ordinary;}};
        const failures=[
            ()=>scalar.step(0,{...options,maximumCellVisits:0}),
            ()=>scalar.step(.01,{...options,maximumSubsteps:0}),
            ()=>late.step(.4,{...options,maxSubstep:1}),
            ()=>scalar.setState(accessor(4,ordinary)),
            ()=>scalar.setState(proxy),
            ()=>scalar.setState(hasProxy),
            ()=>scalar.setState(lengthProxy),
            ()=>scalar.setVelocities(accessor(4,ordinary),zero()),
            ()=>scalar.setVelocities(zero(),proxy),
            ()=>scalar.setVelocities(zero(),accessor(4,73)),
            ()=>scalar.step(0,optionGetter),
            ()=>new physics.PeriodicScalarTransport({...config,get rows(){throw ordinary;}}),
            ()=>scalar.step(0,{...options,maximumSubsteps:2**32+1}),
            ()=>scalar.setState(accessor(4,undefined)),
            ()=>scalar.setState(accessor(4,null)),
        ];
        function scalarBatch() {
            for(let i=0;i<failures.length;++i) {
                const before=stack();
                assert.throws(failures[i],error=>i===9?error===73:i===13?error===undefined:
                    i===14?error===null:error instanceof Error&&!('excPtr' in error));
                assert.equal(stack(),before);
            }
            assert.throws(()=>scalar.setState(proxy),error=>error===ordinary);
            assert.throws(()=>scalar.step(0,optionGetter),error=>error===ordinary);
            let reads=0;
            const unread=zero();Object.defineProperty(unread,3,{get:()=>{++reads;throw ordinary;}});
            assert.throws(()=>scalar.setVelocities(unread,[]),error=>error instanceof RangeError);
            assert.equal(reads,0); // both face lengths before any entry access
            const nested=zero();Object.defineProperty(nested,3,{get:()=>{scalar.step(0,{...options,maximumCellVisits:0});}});
            assert.throws(()=>scalar.setState(nested),error=>error instanceof Error&&/budget/.test(error.message));
            const reentrant=zero();Object.defineProperty(reentrant,3,{get:()=>{
                scalar.setState(zero());assert.equal(probe.method(),7);return 0;
            }});
            scalar.setState(reentrant);scalar.setVelocities(zero(),reentrant);
            for(const method of ['setState','setVelocities']) {
                const receiver=new physics.PeriodicScalarTransport(config),deleting=zero();
                Object.defineProperty(deleting,3,{get:()=>{receiver.delete();return 0;}});
                assert.throws(()=>method==='setState'?receiver.setState(deleting):receiver.setVelocities(zero(),deleting),
                    error=>error instanceof Error);
                assert.ok(receiver.isDeleted());
            }
        }
        for(let i=0;i<20;++i) scalarBatch();
        const scalarBefore=physics.boundaryTestStats(),scalarStack=stack();
        for(let i=0;i<1000;++i) scalarBatch();
        assert.deepEqual(physics.boundaryTestStats(),scalarBefore);assert.equal(stack(),scalarStack);
        physics.PeriodicScalarTransport.prototype.getConfig=nativeConfig;
        for(let i=0;i<owners.length;++i) {assert.deepEqual(snapshot(owners[i]),states[i]);owners[i].delete();}
        console.log(`PASS: scalar boundary stress; 1000 batches/21000 rejection checks, stack=${scalarStack}, live heap=${scalarBefore.heap}, uncaught=${scalarBefore.uncaught}; unchanged scalar/advector/clock/diagnostics`);
    }
    require("./handle-lifetime-tests.cjs").stress(physics);
    require("./elastic-wave-tests.cjs").stress(physics, probe);
    require("./electrostatic-tests.cjs").stress(physics, probe);
    require("./maxwell-ohmic-tests.cjs").stress(physics, probe);
    require("./maxwell-mean-tests.cjs").stress(physics);
    probe.delete();
    assert.equal(physics.boundaryTestStats().objects, 0);
    assert.equal(physics.boundaryTestStats().values, 1); // test static field only
    console.log(`PASS: WASM boundary stress; 2000 batches, stable stack=${initialStack}, live heap=${final.heap}, uncaught=0`);
})().catch(error => { console.error(error); process.exitCode = 1; });

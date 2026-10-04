const assert = require('node:assert/strict');

function batch(p) {
    const config = {columns:2, rows:2, spacingX:1, spacingY:1};
    const options = {density:1, timeStep:1, absoluteDivergenceTolerance:1e-10,
        relativeDivergenceTolerance:1e-10, maximumIterations:1000, maximumCellVisits:100000000};
    const material = {density:1, restitution:0, staticFriction:0, dynamicFriction:0};
    const deleted = error => error instanceof Error && /deleted/i.test(error.message) && !('excPtr' in error);
    let reads = 0;
    const dying = new p.PeriodicMacGrid(config);
    assert.throws(() => dying.project({...options, get density() {
        ++reads; dying.delete(); return 1;
    }}), deleted);
    assert.equal(reads, 1);
    assert.ok(dying.isDeleted());

    // Deletion invalidates this handle even when an independent clone retains
    // the native object. Rejection must leave that retained state unchanged.
    const owner = new p.PeriodicMacGrid(config), alias = owner.clone();
    const before = alias.getVelocities();
    assert.throws(() => owner.project({...options, get timeStep() {
        owner.delete(); return 1;
    }}), deleted);
    assert.deepEqual(alias.getVelocities(), before);
    alias.project(options);
    alias.delete();

    const nested = new p.PeriodicMacGrid(config);
    nested.project({...options, get density() {
        assert.throws(() => nested.project({...options, density:0}), /density/i);
        nested.project(options);
        return 1;
    }});
    const ordinary = new Error('deleting getter identity');
    assert.throws(() => nested.project({...options, get density() {
        nested.delete(); throw ordinary;
    }}), error => error === ordinary);

    // Raw base-class argument precedes two value objects in this free function.
    const shape = new p.Circle(1);
    assert.throws(() => p.createRigidBody(shape, material, {get x() {
        shape.delete(); return 0;
    }, y:0}, false), deleted);

    const makeBody = () => {
        const s = new p.Circle(1);
        try { return p.createRigidBody(s, material, {x:0, y:0}, false); }
        finally { s.delete(); }
    };
    // Both shared-pointer argument positions must be validated after the later
    // anchor getter, including when the other argument remains alive.
    for (const which of [0, 1]) {
        const bodies = [makeBody(), makeBody()];
        assert.throws(() => p.createRevoluteJoint(bodies[0], bodies[1], {x:0, y:0}, {
            get x() { bodies[which].delete(); return 0; }, y:0,
        }), deleted);
        bodies[1-which].delete();
    }
    // Deleting a different alias is allowed: the passed handle still owns it.
    const first = makeBody(), second = makeBody(), copy = first.clone();
    const joint = p.createRevoluteJoint(first, second, {x:0, y:0}, {
        get x() { copy.delete(); return 0; }, y:0,
    });
    assert.deepEqual(joint.getAnchorA(), {x:0, y:0});
    first.delete(); second.delete(); joint.delete();

    // Subtype shared ownership (RevoluteJoint -> Joint) must not invoke
    // user-shadowed lifetime methods during conversion or native cleanup.
    const engine = new p.Engine(), a = makeBody(), b = makeBody();
    engine.addBody(a); engine.addBody(b);
    const hinge = p.createRevoluteJoint(a, b, {x:0, y:0}, {x:0, y:0});
    const originalDelete = p.RevoluteJoint.prototype.delete;
    let lifetimeCalls = 0;
    hinge.clone = () => { ++lifetimeCalls; throw ordinary; };
    p.RevoluteJoint.prototype.delete = () => { ++lifetimeCalls; throw ordinary; };
    try {
        engine.addJoint(hinge);
        originalDelete.call(hinge);
        engine.clearBodies();
        engine.delete();
        assert.equal(lifetimeCalls, 0);
    } finally {
        // The original delete is inherited; remove only our temporary shadow.
        delete p.RevoluteJoint.prototype.delete;
        a.delete(); b.delete();
    }

    // Release numeric converters coerce objects; assertion builds can reject
    // them first. Neither path may enter native code with a deleted receiver.
    const numeric = new p.PeriodicMacGrid(config);
    let coercions = 0;
    assert.throws(() => numeric.project({...options, density:{valueOf() {
        ++coercions; numeric.delete(); return 1;
    }}}), error => coercions ? deleted(error) : error instanceof TypeError);
    assert.ok(coercions === 0 || coercions === 1);
    if (!numeric.isDeleted()) numeric.delete();

    // Either Ohmic scalar converter can delete the wired receiver. Retaining
    // an independent alias makes unintended native entry observable through
    // fields, clock and diagnostics even if the allocation remains alive.
    const defaults = new p.MaxwellGrid(), maxwellConfig = defaults.getConfig();
    defaults.delete();
    for (const which of [0, 1]) {
        const receiver = new p.MaxwellGrid({...maxwellConfig, columns:2, rows:2});
        receiver.setState({ez:Array(4).fill(1), hx:Array(4).fill(2), hy:Array(4).fill(-3)});
        const retained = receiver.clone();
        const before = {state:retained.getState(), diagnostics:retained.getDiagnostics()};
        let coercions = 0;
        const args = [.1, .7];
        args[which] = {valueOf() {
            ++coercions; receiver.delete(); return which === 0 ? .1 : .7;
        }};
        assert.throws(() => receiver.stepOhmic(...args),
            error => coercions ? deleted(error) : error instanceof TypeError);
        assert.ok(coercions === 0 || coercions === 1);
        assert.equal(receiver.isDeleted(), coercions === 1);
        assert.deepEqual({state:retained.getState(), diagnostics:retained.getDiagnostics()}, before);
        // The surviving owner remains usable after rejection of the dead handle.
        const report = retained.stepOhmic(.1, .7);
        assert.equal(report.endTime, .1);
        assert.ok(report.exactJouleEnergy > 0);
        if (!receiver.isDeleted()) receiver.delete();
        retained.delete();
    }

    // Enum values are SDK objects with a numeric .value. A foreign replacement
    // must never be coerced by the field setter after value conversion.
    const wave = new p.WaveMembrane(3, 3, 1, 1);
    const waveConfig = wave.getConfig();
    let enumReads = 0, enumCoercions = 0;
    assert.throws(() => wave.setConfig({...waveConfig, boundary:{get value() {
        ++enumReads;
        return {valueOf() { ++enumCoercions; wave.delete(); return 0; }};
    }}}), TypeError);
    assert.equal(enumReads, 1);
    assert.equal(enumCoercions, 0);
    assert.deepEqual(wave.getConfig(), waveConfig);
    wave.delete();

    if (p.BoundaryTestProbe) {
        for (const [method, value] of [['scalarFloat', 1.25], ['scalarInteger', 7],
            ['scalarInt64', 7n], ['scalarUint64', 7n]]) {
            for (const primitiveHook of ['valueOf', Symbol.toPrimitive]) {
                const receiver = new p.BoundaryTestProbe({number:1}), alias = receiver.clone();
                let reads = 0, calls = 0;
                const input = {};
                Object.defineProperty(input, primitiveHook, {get() {
                    ++reads;
                    return () => {
                        ++calls; receiver.delete(); return value;
                    };
                }});
                assert.throws(() => receiver[method](input),
                    error => calls ? deleted(error) : error instanceof TypeError);
                assert.ok(calls === 0 || calls === 1);
                assert.equal(reads, calls);
                assert.equal(alias[method](value), value);
                if (!receiver.isDeleted()) receiver.delete();
                alias.delete();
            }
            const receiver = new p.BoundaryTestProbe({number:1});
            let calls = 0;
            const identity = new Error('scalar coercion identity');
            assert.throws(() => receiver[method]({[Symbol.toPrimitive]() {
                ++calls;
                assert.equal(receiver[method](value), value);
                assert.throws(() => receiver.method({number:1}), /probe method/);
                throw identity;
            }}), error => calls ? error === identity : error instanceof TypeError);
            assert.ok(calls === 0 || calls === 1);
            assert.equal(receiver[method](value), value);
            receiver.delete();
        }
        const scalar = new p.BoundaryTestProbe({number:1});
        assert.equal(scalar.scalarFloat(true), 1);
        assert.equal(scalar.scalarInteger(true), 1);
        assert.equal(scalar.scalarInt64(7), 7n); // SDK accepts a numeric i64 input.
        assert.equal(scalar.scalarInt64(-(1n << 63n)), -(1n << 63n));
        assert.equal(scalar.scalarUint64((1n << 64n) - 1n), (1n << 64n) - 1n);
        assert.throws(() => scalar.scalarFloat(7n), TypeError);
        assert.throws(() => scalar.scalarInteger(7n), TypeError);
        for (const method of ['scalarFloat', 'scalarInteger', 'scalarInt64'])
            assert.throws(() => scalar[method](Symbol('invalid scalar')), TypeError);
        // Objects yielding BigInt differ from objects yielding Number for an
        // i64 wire input; BigInt(value) would wrongly accept the latter.
        let bigintCalls = 0;
        const bigintObject = {[Symbol.toPrimitive]() { ++bigintCalls; return 7n; }};
        try { assert.equal(scalar.scalarInt64(bigintObject), 7n); }
        catch (error) { assert.equal(bigintCalls, 0); assert.ok(error instanceof TypeError); }
        assert.ok(bigintCalls === 0 || bigintCalls === 1);
        assert.throws(() => scalar.scalarInt64({valueOf() { return 7; }}), TypeError);
        assert.throws(() => scalar.scalarFloat({valueOf() { return 7n; }}), TypeError);
        scalar.delete();
        const property = new p.BoundaryTestProbe({number:1});
        assert.throws(() => { property.value = {get number() {
            property.delete(); return 1;
        }}; }, deleted);
        const argument = new p.BoundaryTestProbe({number:1});
        assert.throws(() => new p.BoundaryTestProbe(argument, {get number() {
            argument.delete(); return 1;
        }}), deleted);
    }
}

exports.smoke = batch;
exports.stress = p => {
    for (let i=0; i<20; ++i) batch(p);
    const before = p.boundaryTestStats(), stack = p._emscripten_stack_get_current();
    for (let i=0; i<1000; ++i) batch(p);
    assert.deepEqual(p.boundaryTestStats(), before);
    assert.equal(p._emscripten_stack_get_current(), stack);
    console.log(`PASS: native-handle conversion lifetimes; 1000 batches, stack=${stack}, live heap=${before.heap}, uncaught=${before.uncaught}`);
};

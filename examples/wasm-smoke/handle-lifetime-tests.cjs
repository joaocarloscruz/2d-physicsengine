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

    if (p.BoundaryTestProbe) {
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

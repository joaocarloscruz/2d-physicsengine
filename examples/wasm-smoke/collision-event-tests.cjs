const assert = require('node:assert/strict');

const material = {density: 1, restitution: 0, staticFriction: 0.5, dynamicFriction: 0.5};

function makeBody(physics, shape, position, isStatic, sensor = false) {
    const body = physics.createRigidBody(shape, material, position, isStatic);
    if (sensor) body.setSensor(true);
    return body;
}

// Reads a list into plain objects, then frees it (the caller owns the list).
function drain(list) {
    const events = [];
    for (let i = 0; i < list.size(); ++i) events.push(list.getEvent(i));
    const overflowed = list.overflowed(), dropped = list.getDroppedCount();
    list.delete();
    return {events, overflowed, dropped};
}
function take(engine) { return drain(engine.takeCollisionEvents()); }
function view(physics, events, base = 0n) {
    const names = new Map([[physics.CollisionPhase.Begin, 'Begin'], [physics.CollisionPhase.Persist, 'Persist'],
        [physics.CollisionPhase.End, 'End']]);
    return events.map(e => `${names.get(e.phase)}${e.sensor ? '*' : ''}:${e.bodyAId - base}-${e.bodyBId - base}`);
}
function newEngine(physics) {
    const engine = new physics.Engine();
    const config = engine.getSimulationConfig();
    config.enableLinearVelocityLimit = false;
    engine.setSimulationConfig(config);
    physics.addGravity(engine, {x: 0, y: -9.81});
    return engine;
}

function groundScene(physics) {
    const engine = newEngine(physics);
    const box = physics.Polygon.makeBox(10, 1), ball = new physics.Circle(0.25);
    const ground = makeBody(physics, box, {x: 0, y: 0}, true);
    const body = makeBody(physics, ball, {x: 0, y: 0.8}, false);
    box.delete(); ball.delete();
    engine.addBody(ground); engine.addBody(body);
    return {engine, ground, ball: body};
}

function testLifecycle(physics) {
    const {engine, ground, ball} = groundScene(physics);
    assert.equal(engine.isCollisionEventsEnabled(), false);
    assert.equal(engine.getCollisionEventCapacity(), 1024);
    // Recording is opt-in: a disabled engine records nothing.
    for (let i = 0; i < 30; ++i) engine.step(1 / 60);
    assert.equal(take(engine).events.length, 0);
    engine.enableCollisionEvents(true);
    engine.enableCollisionEvents(true); // idempotent: one registration, one event each
    assert.equal(engine.isCollisionEventsEnabled(), true);
    // Enabled while the ball already rests: the stream starts mid-lifecycle
    // with a Persist and no Begin, and a duplicate enable adds no second copy.
    const {Begin, Persist, End} = physics.CollisionPhase;
    engine.step(1 / 60);
    const resting = take(engine).events;
    assert.equal(resting.length, 1);
    assert.equal(resting[0].phase, Persist);
    // Teleporting the ball away ends the pair, dropping it again begins a new one.
    ball.setPosition({x: 0, y: 3}); ball.setVelocity({x: 0, y: 0});
    const phases = [];
    for (let i = 0; i < 90; ++i) {
        engine.step(1 / 60);
        phases.push(...take(engine).events.map(e => e.phase));
    }
    assert.equal(phases[0], End);
    assert.equal(phases[1], Begin);
    assert.ok(phases.slice(2).every(p => p === Persist), 'a resting ball persists');
    assert.ok(phases.length > 10);

    // Cleanly sequence one lifecycle: fresh scene, land, rest, lift away.
    engine.clearBodies(); ground.delete(); ball.delete(); engine.clearCollisionEvents();
    const scene = groundScene(physics);
    scene.engine.enableCollisionEvents(true);
    const seen = [];
    const groundId = scene.ground.getId(), ballId = scene.ball.getId();
    for (let i = 0; i < 60; ++i) {
        scene.engine.step(1 / 60);
        seen.push(...take(scene.engine).events);
    }
    scene.ball.setVelocity({x: 0, y: 8}); // lift away from the ground
    for (let i = 0; i < 30; ++i) {
        scene.engine.step(1 / 60);
        seen.push(...take(scene.engine).events);
    }
    const names = view(physics, seen);
    const first = seen[0];
    assert.equal(first.phase, Begin);
    assert.equal(first.sensor, false);
    assert.equal(first.bodyAId, groundId < ballId ? groundId : ballId);
    assert.equal(first.bodyBId, groundId < ballId ? ballId : groundId);
    assert.ok(first.bodyAId < first.bodyBId);
    assert.equal(first.contactCount, 1);
    assert.ok(Math.abs(Math.abs(first.normal.y) - 1) < 1e-6 && Math.abs(first.normal.x) < 1e-6);
    assert.ok(first.penetration >= 0);
    assert.ok(Math.abs(first.point0.y - 0.5) < 0.1, `contact lies on the ground top, got ${first.point0.y}`);
    // The lifecycle is exactly Begin, Persist..., End, in that order, once.
    const lifecycle = seen.map(e => e.phase);
    assert.equal(lifecycle.filter(p => p === Begin).length, 1);
    assert.equal(lifecycle.filter(p => p === End).length, 1);
    assert.equal(lifecycle[lifecycle.length - 1], End);
    assert.ok(lifecycle.indexOf(Persist) > 0 && lifecycle.indexOf(Persist) < lifecycle.indexOf(End));
    assert.ok(lifecycle.slice(1, -1).every(p => p === Persist), names.join(' '));
    assert.ok(seen.every(e => !e.sensor));
    const end = seen[seen.length - 1];
    assert.equal(end.bodyAId, first.bodyAId); assert.equal(end.bodyBId, first.bodyBId);
    assert.equal(end.contactCount, 1, 'End carries the last contact geometry');
    assert.ok(Math.abs(end.normal.y) > 0.99);
    scene.ground.delete(); scene.ball.delete(); scene.engine.delete();
    engine.delete();
}

function sensorScene(physics) {
    const engine = newEngine(physics);
    const slab = physics.Polygon.makeBox(4, 1), ballShape = new physics.Circle(0.25);
    const sensor = makeBody(physics, slab, {x: 0, y: 0}, true, true);
    const ball = makeBody(physics, ballShape, {x: 0, y: 3}, false);
    slab.delete(); ballShape.delete();
    engine.addBody(sensor); engine.addBody(ball);
    engine.enableCollisionEvents(true);
    return {engine, sensor, ball};
}

function testSensor(physics) {
    const {engine, sensor, ball} = sensorScene(physics);
    const seen = [];
    for (let i = 0; i < 120; ++i) {
        engine.step(1 / 60);
        seen.push(...take(engine).events);
    }
    const {Begin, Persist, End} = physics.CollisionPhase;
    assert.ok(seen.length > 2);
    assert.ok(seen.every(e => e.sensor === true), 'every event of a sensor pair is sensor-flagged');
    const phases = seen.map(e => e.phase);
    assert.equal(phases[0], Begin);
    assert.equal(phases.filter(p => p === Begin).length, 1);
    assert.equal(phases.filter(p => p === End).length, 1);
    assert.equal(phases[phases.length - 1], End);
    assert.ok(phases.slice(1, -1).every(p => p === Persist));
    const ids = [sensor.getId(), ball.getId()].sort((a, b) => (a < b ? -1 : 1));
    for (const e of seen) { assert.equal(e.bodyAId, ids[0]); assert.equal(e.bodyBId, ids[1]); }
    assert.ok(ball.getPosition().y < -1, 'the sensor never held the ball');
    sensor.delete(); ball.delete(); engine.delete();
}

// Per-step rules from docs/sensors.md and docs/collision-lifecycle-and-ccd.md.
function checkStepOrder(physics, events) {
    const {Begin, Persist, End} = physics.CollisionPhase;
    let ended = false, lastEnd = null;
    for (const e of events) {
        assert.ok(e.bodyAId < e.bodyBId, 'bodyAId is the lower id');
        if (e.phase === End) {
            ended = true;
            if (lastEnd) assert.ok(lastEnd.bodyAId < e.bodyAId || (lastEnd.bodyAId === e.bodyAId && lastEnd.bodyBId < e.bodyBId),
                'End events are sorted by (bodyAId, bodyBId)');
            lastEnd = e;
        } else {
            assert.ok(!ended, 'End events come after Begin/Persist events');
            // Contact events come before sensor events within a step.
        }
    }
    let sensorSeen = false;
    for (const e of events.filter(x => x.phase !== End)) {
        if (e.sensor) sensorSeen = true; else assert.ok(!sensorSeen, 'contact events precede sensor events');
    }
    return {Begin, Persist, End};
}

function crowdScene(physics) {
    const engine = newEngine(physics);
    const box = physics.Polygon.makeBox(20, 1), ball = new physics.Circle(0.25), slab = physics.Polygon.makeBox(2, 0.4);
    const bodies = [makeBody(physics, box, {x: 0, y: 0}, true),
        makeBody(physics, slab, {x: -2, y: 1.5}, true, true)];
    for (const x of [-2, -0.5, 0.5, 2]) bodies.push(makeBody(physics, ball, {x, y: 2.5 + Math.abs(x) * 0.3}, false));
    box.delete(); ball.delete(); slab.delete();
    for (const body of bodies) engine.addBody(body);
    engine.enableCollisionEvents(true);
    return {engine, bodies};
}

function testOrder(physics) {
    const run = () => {
        const {engine, bodies} = crowdScene(physics);
        const steps = [];
        for (let i = 0; i < 150; ++i) {
            engine.step(1 / 60);
            const {events} = take(engine);
            checkStepOrder(physics, events);
            steps.push(view(physics, events, bodies[0].getId()));
        }
        for (const body of bodies) body.delete();
        engine.delete();
        return steps;
    };
    const first = run(), second = run();
    assert.deepEqual(first, second, 'the same scene stepped the same way gives the same events');
    // Per pair the stream is Begin, Persist*, End and alternates correctly.
    const state = new Map();
    for (const step of first) for (const entry of step) {
        const [label, pair] = entry.split(':'); const phase = label.replace('*', '');
        const previous = state.get(pair) ?? 'End';
        if (phase === 'Begin') assert.equal(previous, 'End', `${pair} began twice`);
        else assert.notEqual(previous, 'End', `${pair} ${phase} without Begin`);
        state.set(pair, phase);
    }
    assert.ok(first.flat().some(e => e.startsWith('Begin*')), 'sensor begin recorded');
    assert.ok(first.flat().some(e => e.startsWith('Begin:')), 'contact begin recorded');
}

function testAdvanceAccumulates(physics) {
    const {engine, ground, ball} = groundScene(physics);
    engine.enableCollisionEvents(true);
    const before = engine.getTotalStepCount();
    const result = engine.advance(6 / 60 + 1e-6);
    const performed = Number(engine.getTotalStepCount() - before);
    assert.equal(performed, result.stepsPerformed);
    assert.ok(performed >= 5);
    // A non-clearing snapshot sees everything every substep produced.
    const listA = drain(engine.getCollisionEvents()), listB = drain(engine.getCollisionEvents());
    assert.deepEqual(listA.events, listB.events, 'getCollisionEvents does not clear');
    engine.advance(4 / 60 + 1e-6);
    const later = drain(engine.getCollisionEvents());
    assert.ok(later.events.length > listA.events.length, 'events accumulate across advance() calls until cleared');
    assert.deepEqual(later.events.slice(0, listA.events.length), listA.events);
    engine.clearCollisionEvents();
    assert.equal(drain(engine.getCollisionEvents()).events.length, 0);
    ground.delete(); ball.delete(); engine.delete();
}

function testOverflow(physics) {
    const {engine, ground, ball} = groundScene(physics);
    for (const bad of [0, -1, 1.5, NaN, Infinity, 2 ** 21, 2 ** 53])
        assert.throws(() => engine.setCollisionEventCapacity(bad), `capacity ${bad} must throw`);
    assert.equal(engine.getCollisionEventCapacity(), 1024);
    engine.setCollisionEventCapacity(3);
    assert.equal(engine.getCollisionEventCapacity(), 3);
    engine.enableCollisionEvents(true);
    // Take nothing: events pile up past the capacity.
    for (let i = 0; i < 60; ++i) engine.step(1 / 60);
    const g2 = groundScene(physics); // independent reference count
    g2.engine.enableCollisionEvents(true);
    for (let i = 0; i < 60; ++i) g2.engine.step(1 / 60);
    const reference = drain(g2.engine.getCollisionEvents());
    const total = reference.events.length;
    assert.ok(total > 3 && !reference.overflowed && reference.dropped === 0);
    const capped = drain(engine.getCollisionEvents());
    assert.equal(capped.events.length, 3);
    assert.equal(capped.overflowed, true);
    assert.equal(capped.dropped, total - 3);
    // Body ids are global, so compare the two scenes relative to their ground ids.
    const rebase = (list, id) => list.map(e => ({...e, bodyAId: e.bodyAId - id, bodyBId: e.bodyBId - id}));
    assert.deepEqual(rebase(capped.events, ground.getId()), rebase(reference.events.slice(0, 3), g2.ground.getId()),
        'a full buffer keeps the oldest events');
    assert.equal(capped.events[0].phase, physics.CollisionPhase.Begin);
    // The flag and count belong to the buffer: clear resets them.
    engine.clearCollisionEvents();
    const cleared = drain(engine.getCollisionEvents());
    assert.equal(cleared.events.length, 0); assert.equal(cleared.overflowed, false); assert.equal(cleared.dropped, 0);
    // take() returns the buffer with its overflow state and empties it.
    for (let i = 0; i < 30; ++i) engine.step(1 / 60);
    const taken = take(engine);
    assert.equal(taken.events.length, 3); assert.equal(taken.overflowed, true); assert.ok(taken.dropped > 0);
    const after = take(engine);
    assert.equal(after.events.length, 0); assert.equal(after.overflowed, false);
    // Shrinking the capacity truncates and counts the loss.
    for (let i = 0; i < 5; ++i) engine.step(1 / 60);
    engine.setCollisionEventCapacity(100);
    for (let i = 0; i < 10; ++i) engine.step(1 / 60);
    engine.setCollisionEventCapacity(2);
    const shrunk = drain(engine.getCollisionEvents());
    assert.equal(shrunk.events.length, 2); assert.equal(shrunk.overflowed, true);
    g2.ground.delete(); g2.ball.delete(); g2.engine.delete();
    ground.delete(); ball.delete(); engine.delete();
}

function testRemoval(physics, handles) {
    // Removing a body ends its pairs straight away; no stale listener remains.
    const {engine, ground, ball} = groundScene(physics);
    engine.enableCollisionEvents(true);
    for (let i = 0; i < 60; ++i) engine.step(1 / 60);
    engine.clearCollisionEvents();
    engine.removeBody(ball);
    const removed = take(engine).events;
    assert.equal(removed.length, 1);
    assert.equal(removed[0].phase, physics.CollisionPhase.End);
    engine.addBody(ball);
    for (let i = 0; i < 20; ++i) engine.step(1 / 60);
    engine.clearCollisionEvents();
    engine.clearBodies();
    const cleared = take(engine).events;
    assert.equal(cleared.length, 1);
    assert.equal(cleared[0].phase, physics.CollisionPhase.End);
    // Disabling stops recording but keeps what was recorded.
    engine.addBody(ground); engine.addBody(ball);
    // clearBodies dropped gravity, so start the ball already touching.
    ball.setPosition({x: 0, y: 0.74}); ball.setVelocity({x: 0, y: 0});
    for (let i = 0; i < 5; ++i) engine.step(1 / 60);
    const recorded = drain(engine.getCollisionEvents()).events.length;
    assert.ok(recorded > 0);
    engine.enableCollisionEvents(false);
    assert.equal(engine.isCollisionEventsEnabled(), false);
    for (let i = 0; i < 20; ++i) engine.step(1 / 60);
    assert.equal(drain(engine.getCollisionEvents()).events.length, recorded);
    // A list is an owned snapshot: deleting it twice or using it afterwards throws.
    const list = engine.getCollisionEvents();
    assert.ok(list.size() >= 1);
    assert.throws(() => list.getEvent(list.size()));
    assert.throws(() => list.getEvent(-1));
    assert.throws(() => list.getEvent(1.5));
    list.delete();
    assert.throws(() => list.size());
    assert.throws(() => list.delete());
    // Destroying an engine with the recorder registered must be safe, and the
    // next engine must start with a clean recorder.
    const handlesBefore = handles();
    for (let i = 0; i < 50; ++i) {
        const scene = groundScene(physics);
        scene.engine.enableCollisionEvents(true);
        for (let s = 0; s < 40; ++s) scene.engine.step(1 / 60);
        const list2 = scene.engine.getCollisionEvents();
        assert.ok(list2.size() > 0);
        list2.delete();
        scene.ground.delete(); scene.ball.delete();
        scene.engine.delete();
        const fresh = new physics.Engine();
        assert.equal(fresh.isCollisionEventsEnabled(), false);
        assert.equal(fresh.getCollisionEventCapacity(), 1024);
        assert.equal(fresh.getCollisionEvents().size(), 0);
        fresh.delete();
    }
    assert.equal(handles(), handlesBefore, 'event handles leak no emval references');
    ground.delete(); ball.delete(); engine.delete();
}

function smoke(physics) {
    const handles = typeof physics.count_emval_handles === 'function' ? physics.count_emval_handles : () => 0;
    const before = handles();
    testLifecycle(physics);
    testSensor(physics);
    testOrder(physics);
    testAdvanceAccumulates(physics);
    testOverflow(physics);
    testRemoval(physics, handles);
    assert.equal(handles(), before);
}

// One batch of the boundary stress: every handle is deleted, nothing leaks.
function stress(physics) {
    const scene = groundScene(physics);
    scene.engine.setCollisionEventCapacity(8);
    scene.engine.enableCollisionEvents(true);
    for (let i = 0; i < 12; ++i) scene.engine.step(1 / 60);
    assert.ok(drain(scene.engine.getCollisionEvents()).events.length > 0);
    assert.ok(take(scene.engine).events.length > 0);
    scene.engine.clearBodies();
    scene.engine.clearCollisionEvents();
    scene.ground.delete(); scene.ball.delete(); scene.engine.delete();
}

module.exports = {smoke, stress};

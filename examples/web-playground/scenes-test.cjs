// Node test for the web playground scenes. Run it from build-wasm/wasm after a
// build with -DPHYSICS_WASM_BOUNDARY_TEST_PROBES=ON (the build copies this file
// next to physics_engine.js), or from the source folder once a build exists.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');

const modulePath = fs.existsSync(path.join(__dirname, 'physics_engine.js'))
    ? './physics_engine.js' : '../../build-wasm/wasm/physics_engine.js';
const createModule = require(modulePath);
const Playground = require('./scenes.js');

const STEPS = 600;
const SETTLE_TOLERANCE = 0.05; // metres, from the brief

// Wraps every factory and constructor that returns an Embind handle, so the
// test can prove each handle the playground created was deleted.
function trackHandles(physics) {
    const created = [];
    const track = handle => { created.push(handle); return handle; };
    const facade = Object.create(physics);
    for (const name of ['Engine', 'Circle', 'Capsule']) {
        const Native = physics[name];
        facade[name] = function (...args) { return track(new Native(...args)); };
    }
    facade.Polygon = {makeBox: (w, h) => track(physics.Polygon.makeBox(w, h))};
    for (const name of ['createRigidBody', 'createRevoluteJoint', 'createDistanceJoint',
        'createPrismaticJoint', 'createWeldJoint', 'createWheelJoint', 'createMouseJoint',
        'queryPoint', 'queryCircle']) {
        facade[name] = (...args) => track(physics[name](...args));
    }
    facade.addGravity = (...args) => physics.addGravity(...args);
    return {facade, created};
}

function assertFinitePoses(scene, label) {
    for (const r of scene.bodies) {
        const p = r.body.getPosition(), v = r.body.getVelocity();
        const values = [p.x, p.y, v.x, v.y, r.body.getOrientation(), r.body.getAngularVelocity()];
        for (const value of values) assert.ok(Number.isFinite(value), `${label}: non-finite pose`);
    }
    for (const j of scene.joints) {
        for (const a of [j.joint.getAnchorA(), j.joint.getAnchorB()]) {
            assert.ok(Number.isFinite(a.x) && Number.isFinite(a.y), `${label}: non-finite joint anchor`);
        }
    }
}

function runScene(physics, index, extents) {
    const definition = Playground.SCENES[index];
    const scene = Playground.createScene(physics, index);
    try {
        const dynamic = scene.bodies.filter(r => !r.isStatic);
        const top = dynamic[dynamic.length - 1];
        const startTopY = top.y;
        assert.ok(scene.bodyCount > 3 && dynamic.length > 0, `${definition.name} built no bodies`);
        let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
        for (let step = 0; step < STEPS; ++step) {
            scene.stepOnce();
            for (const r of dynamic) {
                minX = Math.min(minX, r.x); maxX = Math.max(maxX, r.x);
                minY = Math.min(minY, r.y); maxY = Math.max(maxY, r.y);
            }
            if (step % 100 === 99) assertFinitePoses(scene, definition.name);
        }
        assertFinitePoses(scene, definition.name);
        // The renderer draws joint lines from body poses; they must match the engine's anchors.
        for (const j of scene.joints) {
            for (const [engineAnchor, body, local] of [[j.joint.getAnchorA(), j.a, j.localA], [j.joint.getAnchorB(), j.b, j.localB]]) {
                const drawn = Playground.anchorPoint(body, local, 1);
                assert.ok(Math.hypot(drawn.x - engineAnchor.x, drawn.y - engineAnchor.y) < 1e-3,
                    `${definition.name}: ${j.kind} anchor drawn at ${drawn.x}, ${drawn.y} but engine says ${engineAnchor.x}, ${engineAnchor.y}`);
            }
        }
        // Walls are 2 m thick with inner faces at x = +-10 and the floor top at y = -5;
        // a dynamic centre outside this box means a body tunnelled out or escaped.
        const {minX: left, maxX: right, floorTop} = Playground.ARENA;
        assert.ok(minX >= left && maxX <= right, `${definition.name} left the arena in x [${minX}, ${maxX}]`);
        assert.ok(minY >= floorTop - 0.1, `${definition.name} sank below the floor: ${minY}`);
        assert.ok(maxY <= 25, `${definition.name} flew out of the arena: ${maxY}`);
        extents.push(`${definition.name}: x [${minX.toFixed(2)}, ${maxX.toFixed(2)}] y [${minY.toFixed(2)}, ${maxY.toFixed(2)}]`);
        const drift = Math.abs(top.y - startTopY);
        if (definition.name === 'Tower' || definition.name === 'Pyramid') {
            assert.ok(drift <= SETTLE_TOLERANCE,
                `${definition.name} top body moved ${drift} m (limit ${SETTLE_TOLERANCE})`);
            extents.push(`${definition.name} top-body drift after ${STEPS} steps: ${drift.toFixed(4)} m`);
        }
        return {bodies: scene.bodyCount, joints: scene.jointCount};
    } finally {
        scene.dispose();
    }
}

// Spawning, picking, dragging and culling, plus the advance()-based frame loop.
function runInteraction(physics) {
    const scene = Playground.createScene(physics, 0);
    try {
        const initial = scene.bodyCount;
        ['circle', 'box', 'capsule'].forEach((kind, i) => scene.spawn(kind, 4 + 2 * i, 4));
        assert.equal(scene.bodyCount, initial + 3);
        assert.throws(() => scene.spawn('torus', 0, 0), /Unknown shape/);

        const ball = scene.pick(0, 0);
        assert.ok(ball && ball.kind === 'circle', 'pick must find the playground ball');
        assert.equal(scene.pick(-9.5, -5.5), null, 'static walls are not draggable');
        const bodiesBeforeDrag = scene.bodyCount, jointsBeforeDrag = scene.jointCount;
        scene.beginDrag(ball, 0.2, 0.1);
        // The MouseJoint and its ground body are internal to the drag: not listed, not drawn.
        assert.equal(scene.bodyCount, bodiesBeforeDrag);
        assert.equal(scene.jointCount, jointsBeforeDrag);
        const {joint: dragJoint, ground: dragGround} = scene.drag;
        assert.equal(dragJoint.getFrequency(), 5);
        assert.ok(Math.abs(dragJoint.getMaxForce() - 1000 * ball.body.getMass()) < 1e-3, 'drag force cap is 1000 * mass');
        scene.setDragTarget(-6, 3);
        for (let i = 0; i < 180; ++i) scene.stepOnce();
        const p = ball.body.getPosition();
        assert.ok(Math.hypot(p.x + 6, p.y - 3) < 0.6, `dragged ball ended at ${p.x}, ${p.y}`);
        scene.endDrag();
        assert.equal(scene.drag, null);
        assert.ok(dragJoint.isDeleted() && dragGround.isDeleted(), 'ending a drag deletes its handles');
        // A second grab on the same scene starts cleanly, and disposing mid-drag releases it.
        scene.beginDrag(ball, p.x, p.y);
        scene.beginDrag(ball, p.x, p.y); // replaces the first drag
        for (let i = 0; i < 30; ++i) scene.stepOnce();
        assertFinitePoses(scene, 'redrag');

        const dt = scene.fixedTimeStep;
        const before = scene.engine.getTotalStepCount();
        let steps = 0;
        const frame = 1 / 144; // a display faster than the simulation: most frames take no step
        for (let i = 0; i < 288; ++i) {
            const result = scene.advanceFrame(frame);
            steps += result.steps;
            assert.ok(result.alpha >= 0 && result.alpha <= 1, `alpha ${result.alpha}`);
        }
        const expected = Math.round(288 * frame / dt); // exactly 2 s of frames
        assert.equal(steps, expected, 'advanceFrame step count');
        assert.equal(scene.engine.getTotalStepCount() - before, BigInt(steps));
        // A long stall is clamped, so it cannot trigger more than 6 steps.
        assert.ok(scene.advanceFrame(5).steps <= Math.ceil(0.1 / dt));
        assertFinitePoses(scene, 'interaction');

        // Interpolation endpoints.
        const r = scene.bodies.find(b => !b.isStatic);
        const start = Playground.renderPose(r, 0), end = Playground.renderPose(r, 1);
        assert.ok(Math.abs(start.x - r.px) < 1e-12 && Math.abs(end.x - r.x) < 1e-12);

        // Culling removes and deletes spawned bodies that left the world.
        const far = scene.spawn('circle', 0, 0);
        far.body.setPosition({x: 500, y: 0});
        far.x = 500;
        const count = scene.bodyCount;
        scene.cull();
        assert.equal(scene.bodyCount, count - 1);
        assert.ok(far.body.isDeleted());
    } finally {
        scene.dispose();
    }
}

function sceneIndexByName(name) {
    const index = Playground.SCENES.findIndex(scene => scene.name === name);
    assert.ok(index >= 0, `no scene named ${name}`);
    return index;
}

// Steps until `done()` is true or `limit` steps pass; returns the steps taken.
function stepUntil(scene, limit, done) {
    let steps = 0;
    while (steps < limit && !done()) { scene.stepOnce(); ++steps; }
    assert.ok(done(), `condition not reached in ${limit} steps`);
    return steps;
}

// Car and sensors: the motor drives the car forward, the sensor goal registers
// the overlap (found with queryCircle), leaving and re-entering counts again.
function runCar(physics, report) {
    const scene = Playground.createScene(physics, sceneIndexByName('Car and sensors'));
    try {
        const {goal} = scene;
        assert.ok(goal.record.body.isSensor() && goal.record.sensor, 'the goal is a sensor');
        assert.equal(goal.record.body.getUserData(), 7);
        const {chassis, wheels} = goal.car;
        const joints = scene.joints.filter(j => j.kind === 'wheel');
        assert.equal(joints.length, 2);
        const jointsBefore = scene.jointCount, bodiesBefore = scene.bodyCount;

        // Brake: without throttle the car stays where it is.
        for (let i = 0; i < 180; ++i) scene.stepOnce();
        const parkedX = chassis.x;
        for (let i = 0; i < 120; ++i) scene.stepOnce();
        const idleDrift = Math.abs(chassis.x - parkedX);
        assert.ok(idleDrift < 0.3, `idle car rolled ${idleDrift} m`);
        assert.equal(goal.entries, 0);
        assert.equal(goal.inside, false);

        // Drive right with the rear motor.
        scene.input.throttle = 1;
        const x0 = chassis.x;
        let maxSpeed = 0;
        const steps = stepUntil(scene, 900, () => {
            maxSpeed = Math.max(maxSpeed, chassis.body.getVelocity().x);
            return goal.inside;
        });
        assert.ok(chassis.x - x0 > 5, `car moved only ${chassis.x - x0} m`);
        assert.ok(maxSpeed > 1, `car top speed ${maxSpeed} m/s`);
        assert.equal(goal.entries, 1, 'entering the goal counts once');
        assert.ok(joints.every(j => Math.abs(j.joint.getTranslation()) <= 0.31), 'suspension stays within its stroke');
        assert.ok(joints[0].joint.isMotorEnabled() && joints[0].joint.getMotorSpeed() < 0, 'rear motor drives forward');
        assert.ok(wheels.every(w => w.y > -5)); // wheels rest on the terrain
        const reachedAt = steps;

        // Staying inside does not count again; driving out and back in does.
        for (let i = 0; i < 30; ++i) scene.stepOnce();
        assert.equal(goal.entries, 1);
        scene.input.throttle = -1;
        stepUntil(scene, 900, () => !goal.inside);
        assert.equal(goal.entries, 1);
        scene.input.throttle = 1;
        stepUntil(scene, 900, () => goal.inside);
        assert.equal(goal.entries, 2, 're-entering counts again');
        assert.ok(scene.status().includes('goal entries 2'), scene.status());
        assertFinitePoses(scene, 'car');

        // Dragging the chassis uses a MouseJoint without disturbing the joints.
        scene.beginDrag(chassis, chassis.x, chassis.y);
        scene.setDragTarget(chassis.x - 2, chassis.y + 3);
        for (let i = 0; i < 90; ++i) scene.stepOnce();
        scene.endDrag();
        assertFinitePoses(scene, 'car drag');
        assert.equal(scene.jointCount, jointsBefore);
        assert.equal(scene.bodyCount, bodiesBefore);
        report.push(`Car and sensors: idle drift ${idleDrift.toFixed(2)} m, reached goal after ${reachedAt} steps ` +
            `at ${maxSpeed.toFixed(2)} m/s, entries ${goal.entries}`);
    } finally {
        scene.dispose();
    }
}

// Joints gallery: each joint type does what it is there to show.
function runGallery(physics, report) {
    const scene = Playground.createScene(physics, sceneIndexByName('Joints gallery'));
    try {
        const kinds = {};
        for (const j of scene.joints) kinds[j.kind] = (kinds[j.kind] || 0) + 1;
        assert.deepEqual(kinds, {weld: 6, rope: 1, spring: 1, wheel: 1, revolute: 1});
        const [rope] = scene.joints.filter(j => j.kind === 'rope').map(j => j.joint);
        const [spring] = scene.joints.filter(j => j.kind === 'spring').map(j => j.joint);
        const [strut] = scene.joints.filter(j => j.kind === 'wheel').map(j => j.joint);
        const welds = scene.joints.filter(j => j.kind === 'weld').map(j => j.joint);
        const soft = welds.find(j => j.getAngularFrequency() > 0);
        assert.ok(soft && welds.filter(j => j.getAngularFrequency() === 0).length === 5, 'one soft weld, five rigid');
        assert.equal(rope.getMinLength(), 0);
        assert.equal(rope.getMaxLength(), 4);
        assert.ok(Math.abs(spring.getSpringFrequency() - 1.2) < 1e-6);

        const range = () => ({lo: Infinity, hi: -Infinity});
        const strutRange = range(), springRange = range(), ropeRange = range(), softAngle = range();
        let rigidSeparation = 0, rigidAngle = 0;
        const note = (r, v) => { r.lo = Math.min(r.lo, v); r.hi = Math.max(r.hi, v); };
        for (let step = 0; step < 900; ++step) {
            scene.stepOnce();
            note(strutRange, strut.getTranslation());
            note(springRange, spring.getCurrentLength());
            note(ropeRange, rope.getCurrentLength());
            note(softAngle, soft.getAngle());
            for (const weld of welds) {
                if (weld === soft) continue;
                rigidSeparation = Math.max(rigidSeparation, weld.getSeparation());
                rigidAngle = Math.max(rigidAngle, Math.abs(weld.getAngle()));
            }
        }
        assert.ok(rigidSeparation < 0.1, `rigid welds separated by ${rigidSeparation} m`);
        // The ball's impact bends the cantilever briefly; once it has settled each weld is nearly straight.
        assert.ok(rigidAngle < 0.5, `rigid welds bent by ${rigidAngle} rad`);
        const settledAngle = Math.max(...welds.filter(j => j !== soft).map(j => Math.abs(j.getAngle())));
        assert.ok(settledAngle < 0.05, `rigid welds still bent by ${settledAngle} rad after 15 s`);
        assert.ok(softAngle.hi - softAngle.lo > 0.05, `soft weld barely moved: ${softAngle.hi - softAngle.lo} rad`);
        assert.ok(ropeRange.hi <= 4 + 0.02, `rope stretched to ${ropeRange.hi}`);
        assert.ok(ropeRange.lo < 3.6, `rope never went slack: shortest ${ropeRange.lo}`);
        assert.ok(springRange.hi - springRange.lo > 0.3, `spring length range ${springRange.hi - springRange.lo}`);
        assert.ok(strutRange.hi - strutRange.lo > 0.2, `suspension travel ${strutRange.hi - strutRange.lo}`);
        assert.ok(strutRange.lo >= -1.1 - 0.02 && strutRange.hi <= 0.5 + 0.02, 'suspension stays between its stops');
        report.push(`Joints gallery: rigid weld separation ${rigidSeparation.toFixed(4)} m, soft weld swing ` +
            `${(softAngle.hi - softAngle.lo).toFixed(2)} rad, rope length [${ropeRange.lo.toFixed(2)}, ${ropeRange.hi.toFixed(2)}], ` +
            `spring length range ${(springRange.hi - springRange.lo).toFixed(2)} m, suspension travel ` +
            `${(strutRange.hi - strutRange.lo).toFixed(2)} m`);
    } finally {
        scene.dispose();
    }
}

(async () => {
    const physics = await createModule();
    assert.equal(typeof physics.boundaryTestStats, 'function', 'configure PHYSICS_WASM_BOUNDARY_TEST_PROBES=ON');
    assert.equal(typeof physics.count_emval_handles, 'function', 'probe builds must export the emval counter');
    assert.equal(typeof physics.addGravity, 'function', 'module is older than the addGravity binding');
    assert.equal(Playground.SCENES.length, 10);

    const {facade, created} = trackHandles(physics);
    const snapshot = () => ({...physics.boundaryTestStats(), emval: physics.count_emval_handles()});
    const extents = [];
    const counts = [];
    const behaviour = [];
    let firstCounts = null;

    const cycle = () => {
        extents.length = 0; counts.length = 0; behaviour.length = 0;
        for (let i = 0; i < Playground.SCENES.length; ++i) counts.push(runScene(facade, i, extents));
        runInteraction(facade);
        runCar(facade, behaviour);
        runGallery(facade, behaviour);
        // Body and joint counts are the same every time a scene is rebuilt.
        const serialized = JSON.stringify(counts);
        if (firstCounts === null) firstCounts = serialized;
        assert.equal(serialized, firstCounts, 'scene body/joint counts changed between cycles');
    };

    cycle(); // warm-up: allocator pools, string caches
    const baseline = snapshot();
    const baselineHandles = created.length;
    for (const handle of created) assert.ok(handle.isDeleted(), 'a handle survived dispose()');
    cycle();
    cycle();
    const after = snapshot();

    for (const handle of created) assert.ok(handle.isDeleted(), 'a handle survived dispose()');
    assert.ok(created.length > baselineHandles, 'handle tracker saw no handles');
    assert.equal(after.heap, baseline.heap, 'live native heap grew across scene cycles');
    assert.equal(after.emval, baseline.emval, 'JavaScript values retained by native code grew');
    assert.equal(after.values, baseline.values);
    assert.equal(after.objects, baseline.objects);
    assert.equal(after.uncaught, 0);

    for (let i = 0; i < Playground.SCENES.length; ++i) {
        console.log(`scene ${i + 1} ${Playground.SCENES[i].name}: ${counts[i].bodies} bodies, ${counts[i].joints} joints`);
    }
    for (const line of extents) console.log(line);
    for (const line of behaviour) console.log(line);
    console.log(`handles created and deleted: ${created.length}; heap ${baseline.heap} -> ${after.heap}, ` +
        `emval ${baseline.emval} -> ${after.emval}`);
    console.log(`PASS: web playground, 10 scenes x ${STEPS} fixed steps, interaction, handle lifetimes`);
})().catch(error => { console.error(error); process.exit(1); });

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
        'createPrismaticJoint', 'queryPoint']) {
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
        scene.beginDrag(ball, 0.2, 0.1);
        scene.setDragTarget(-6, 3);
        for (let i = 0; i < 180; ++i) scene.stepOnce();
        const p = ball.body.getPosition();
        assert.ok(Math.hypot(p.x + 6, p.y - 3) < 0.6, `dragged ball ended at ${p.x}, ${p.y}`);
        scene.endDrag();

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

(async () => {
    const physics = await createModule();
    assert.equal(typeof physics.boundaryTestStats, 'function', 'configure PHYSICS_WASM_BOUNDARY_TEST_PROBES=ON');
    assert.equal(typeof physics.count_emval_handles, 'function', 'probe builds must export the emval counter');
    assert.equal(typeof physics.addGravity, 'function', 'module is older than the addGravity binding');
    assert.equal(Playground.SCENES.length, 8);

    const {facade, created} = trackHandles(physics);
    const snapshot = () => ({...physics.boundaryTestStats(), emval: physics.count_emval_handles()});
    const extents = [];
    const counts = [];

    const cycle = () => {
        extents.length = 0; counts.length = 0;
        for (let i = 0; i < Playground.SCENES.length; ++i) counts.push(runScene(facade, i, extents));
        runInteraction(facade);
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
    console.log(`handles created and deleted: ${created.length}; heap ${baseline.heap} -> ${after.heap}, ` +
        `emval ${baseline.emval} -> ${after.emval}`);
    console.log(`PASS: web playground, 8 scenes x ${STEPS} fixed steps, interaction, handle lifetimes`);
})().catch(error => { console.error(error); process.exit(1); });

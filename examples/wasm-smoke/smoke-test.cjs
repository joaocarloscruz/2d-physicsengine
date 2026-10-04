const assert = require("node:assert/strict");
const createPhysicsEngineModule = require("./physics_engine.js");

function testQueries(physics) {
    const engine = new physics.Engine();
    const material = {density: 1, restitution: 0, staticFriction: 0, dynamicFriction: 0};
    const shape = new physics.Circle(1);
    const first = physics.createRigidBody(shape, material, {x: 2, y: 0}, true);
    const second = physics.createRigidBody(shape, material, {x: 2, y: 0}, false);
    shape.delete();
    // Reversed registration must not change stable-ID ties.
    engine.addBody(second); engine.addBody(first);
    const firstId = first.getId(), secondId = second.getId();
    assert.equal(typeof firstId, "bigint"); assert.ok(firstId < secondId);
    const start = {x: 0, y: 0}, end = {x: 4, y: 0}, center = {x: 2, y: 0};
    const all = [physics.queryPoint(engine, center), physics.queryCircle(engine, {x: 3.5, y: 0}, 0.5),
        physics.rayCastAll(engine, start, end), physics.sweepCircleAll(engine, start, end, 0.5)];
    for (const results of all) {
        assert.equal(results.size(), 2);
        assert.equal(results.getBodyId(0), firstId); assert.equal(results.getBodyId(1), secondId);
        for (const bad of [-1, 0.5, NaN, Infinity, 2, 2**32, 2**32 + 1]) {
            assert.throws(() => results.getBodyId(bad)); assert.throws(() => results.getBody(bad));
            if (results.getHit) assert.throws(() => results.getHit(bad));
        }
    }
    assert.deepEqual(all[2].getHit(0), {fraction: 0.25, point: {x: 1, y: 0}, normal: {x: -1, y: 0}});
    assert.deepEqual(all[3].getHit(0), {fraction: 0.125, center: {x: 0.5, y: 0}, contactPoint: {x: 1, y: 0}, normal: {x: -1, y: 0}});
    const nearestRay = physics.rayCastNearest(engine, start, end), nearestSweep = physics.sweepCircleNearest(engine, start, end, 0.5);
    for (const results of [nearestRay, nearestSweep]) {
        assert.equal(results.size(), 1); assert.equal(results.getBodyId(0), firstId); results.delete();
    }
    const calls = [f => physics.queryPoint(engine, center, f), f => physics.queryCircle(engine, center, 0.1, f),
        f => physics.rayCastAll(engine, start, end, f), f => physics.rayCastNearest(engine, start, end, f),
        f => physics.sweepCircleAll(engine, start, end, 0.5, f), f => physics.sweepCircleNearest(engine, start, end, 0.5, f)];
    first.setCollisionCategoryBits(2); first.setCollisionMaskBits(4);
    second.setCollisionCategoryBits(4); second.setCollisionMaskBits(2);
    for (const call of calls) {
        const matched = call({categoryBits: 4, maskBits: 2});
        assert.equal(matched.size(), 1); assert.equal(matched.getBodyId(0), firstId); matched.delete();
        for (const filter of [{categoryBits: 0, maskBits: 0xffffffff}, {categoryBits: 2, maskBits: 2}]) {
            const empty = call(filter); assert.equal(empty.size(), 0); empty.delete();
        }
        for (const field of ["categoryBits", "maskBits"]) for (const bad of [-1, 0.5, NaN, Infinity, 2**32, 2**32 + 2])
            assert.throws(() => call({categoryBits: 0xffffffff, maskBits: 0xffffffff, [field]: bad}));
    }
    const rayCopy = all[2].getHit(0); rayCopy.point.x = 99; rayCopy.normal.x = 99;
    const sweepCopy = all[3].getHit(0); sweepCopy.center.x = 99;
    first.setPosition({x: 20, y: 0});
    engine.clearBodies(); first.delete(); second.delete(); engine.delete();
    assert.equal(all[2].getHit(0).point.x, 1); assert.equal(all[3].getHit(0).center.x, 0.5);
    const retained = all[0].getBody(0);
    for (const results of all) { assert.equal(results.getBodyId(0), firstId); results.delete(); }
    assert.deepEqual(retained.getPosition(), {x: 20, y: 0}); retained.delete();

    const polygonEngine = new physics.Engine(), box = physics.Polygon.makeBox(2, 2);
    const polygon = physics.createRigidBody(box, material, {x: 0, y: 0}, true);
    box.delete(); polygonEngine.addBody(polygon); polygon.delete();
    const a = {x: 2, y: 2}, b = {x: 1.8, y: 1.8};
    const cornerMiss = physics.sweepCircleNearest(polygonEngine, a, b, 1);
    assert.equal(cornerMiss.size(), 0); cornerMiss.delete();
    const cornerHit = physics.sweepCircleNearest(polygonEngine, a, {x: 0, y: 0}, 1);
    const hit = cornerHit.getHit(0);
    assert.ok(Math.abs(hit.fraction - (1 - 1/Math.sqrt(2))/2) < 1e-12);
    assert.deepEqual(hit.contactPoint, {x: 1, y: 1});
    assert.ok(Math.abs(hit.normal.x - 1/Math.sqrt(2)) < 1e-7); cornerHit.delete();
    polygonEngine.clearBodies();
    for (const result of [physics.queryPoint(polygonEngine, a), physics.queryCircle(polygonEngine, a, 0),
        physics.rayCastNearest(polygonEngine, a, b), physics.sweepCircleNearest(polygonEngine, a, b, 0)]) {
        assert.equal(result.size(), 0); assert.throws(() => result.getBody(0)); result.delete();
    }
    for (const bad of [-1, NaN, Infinity]) {
        assert.throws(() => physics.queryCircle(polygonEngine, a, bad));
        assert.throws(() => physics.sweepCircleAll(polygonEngine, a, b, bad));
    }
    assert.throws(() => physics.queryPoint(polygonEngine, {x: NaN, y: 0}));
    assert.throws(() => physics.rayCastAll(polygonEngine, a, {x: Infinity, y: 0}));
    polygonEngine.delete();
}

function testWaves(physics) {
    const defaults = new physics.WaveMembrane(3, 3, 1, 1);
    const config = defaults.getConfig();
    assert.equal(config.boundary, physics.WaveBoundary.FixedZero);
    assert.equal(config.maxCells, 1000000);
    config.boundary = physics.WaveBoundary.Periodic;
    config.maxSubstep = 0.001; config.tension = 6; config.surfaceDensity = 2;
    assert.equal(defaults.getConfig().tension, 1);
    const mode = new physics.WaveMembrane(8, 6, 0.3, 0.4, config);
    assert.equal(mode.getWidth(), 8); assert.equal(mode.getHeight(), 6); assert.equal(mode.getCellCount(), 48);
    assert.equal(mode.getSpacingX(), 0.3); assert.equal(mode.getSpacingY(), 0.4);
    const initial = Array.from({length: 48}, (_, i) => Math.cos(2 * Math.PI * ((i % 8) / 8 + Math.floor(i / 8) / 6)));
    const velocities = Array(48).fill(0);
    mode.setState(initial, velocities);
    const omega = 2 * Math.sqrt(3) * Math.hypot(Math.sin(Math.PI / 8) / 0.3, Math.sin(Math.PI / 6) / 0.4);
    const initialEnergy = mode.getDiagnostics().totalEnergy;
    assert.ok(Math.abs(initialEnergy - 0.25 * 2 * 0.3 * 0.4 * 48 * omega**2) < 1e-12);
    const inputCopy = initial.slice(); mode.setState(inputCopy, velocities); inputCopy[0] = 99;
    assert.equal(mode.getCell(0, 0).displacement, initial[0]);
    mode.step(0.2);
    const diag = mode.getDiagnostics(), state = mode.getDisplacements(), velocity = mode.getVelocities();
    assert.equal(diag.lastCellWork, 48 * diag.lastSubsteps);
    assert.equal(diag.lastSubsteps, 200); assert.equal(diag.lastSubstep, 0.001);
    assert.equal(diag.time, 0.2); assert.ok(diag.lastSubstep <= mode.getStableTimeStep());
    const phase = 2 * Math.asin(omega * diag.lastSubstep / 2) * diag.lastSubsteps;
    const amplitude = Math.cos(phase), speedAmplitude = -omega * Math.sqrt(1 - (omega * diag.lastSubstep / 2)**2) * Math.sin(phase);
    for (let i = 0; i < 48; ++i) {
        assert.ok(Math.abs(state[i] - initial[i] * amplitude) < 2e-13);
        assert.ok(Math.abs(velocity[i] - initial[i] * speedAmplitude) < 1e-12);
    }
    assert.ok(diag.totalEnergy <= initialEnergy * (1 + 1e-12));
    assert.ok(diag.totalEnergy > initialEnergy * 0.99);
    const cellSnapshot = mode.getCell(1, 2), arraySnapshot = mode.getDisplacements(), loadSnapshot = mode.getQueuedAccelerations();
    cellSnapshot.displacement = 99; arraySnapshot[0] = 99; loadSnapshot[0] = 99; diag.totalEnergy = 99;
    assert.notEqual(mode.getCell(1, 2).displacement, 99); assert.notEqual(mode.getDisplacements()[0], 99);
    assert.equal(mode.getCell(0, 0).queuedAcceleration, 0); assert.notEqual(mode.getDiagnostics().totalEnergy, 99);
    for (const bad of [-1, 0.5, NaN, Infinity, 2**32, 2**32 + 1]) {
        assert.throws(() => new physics.WaveMembrane(bad, 3, 1, 1));
        assert.throws(() => new physics.WaveMembrane(3, bad, 1, 1, config));
        for (const [x, y] of [[bad, 0], [0, bad]]) {
            assert.throws(() => mode.getCell(x, y));
            assert.throws(() => mode.setCellState(x, y, 0, 0));
            assert.throws(() => mode.queueAcceleration(x, y, 1));
            assert.throws(() => mode.clearAcceleration(x, y));
        }
    }
    assert.throws(() => mode.getCell(8, 0)); assert.throws(() => mode.getCell(0, 6));
    for (const field of ["maxCells", "maxSubsteps", "maxCellWork"]) {
        for (const count of [-1, 0, 0.5, NaN, Infinity, 2**32, 2**32 + 1]) {
            const invalid = {...config, [field]: count};
            assert.throws(() => mode.setConfig(invalid));
            assert.throws(() => new physics.WaveMembrane(8, 6, 0.3, 0.4, invalid));
            assert.equal(mode.getConfig()[field], config[field]);
        }
    }
    assert.throws(() => mode.setConfig({...config, maxCells: 47}));
    assert.throws(() => mode.setConfig({...config, boundary: 99}));
    assert.throws(() => new physics.WaveMembrane(0, 3, 1, 1));
    assert.throws(() => new physics.WaveMembrane(2, 2, 1, 1));
    assert.throws(() => new physics.WaveMembrane(2**31, 2, 1, 1, {...config, maxCells: 2**32 - 1}));
    for (const spacing of [0, -1, NaN, Infinity]) assert.throws(() => new physics.WaveMembrane(3, 3, spacing, 1));
    const beforeInvalid = mode.getDisplacements();
    for (const values of [null, {}, {length: 48}, new Float64Array(48), Array(47).fill(0), new Array(2**32 - 1)])
        assert.throws(() => mode.setState(values, velocities));
    assert.throws(() => mode.setState(beforeInvalid, []));
    assert.throws(() => mode.setState(Array(48), velocities)); // Sparse entries are not numbers.
    assert.throws(() => mode.setState(["1", ...beforeInvalid.slice(1)], velocities));
    assert.throws(() => mode.setState([NaN, ...beforeInvalid.slice(1)], velocities));
    assert.throws(() => mode.setState(Array(48).fill(3), [Infinity, ...velocities.slice(1)]));
    assert.deepEqual(mode.getDisplacements(), beforeInvalid);
    assert.throws(() => mode.setCellState(0, 0, NaN)); assert.throws(() => mode.queueAcceleration(0, 0, Infinity));

    const uniformConfig = {...config, tension: 1, surfaceDensity: 2, maxSubstep: 0.01};
    const uniform = new physics.WaveMembrane(2, 3, 1, 1, uniformConfig);
    uniform.setState(Array(6).fill(3), Array(6).fill(-2));
    for (let y = 0; y < 3; ++y) for (let x = 0; x < 2; ++x) uniform.queueAcceleration(x, y, 4);
    uniform.step(0); assert.deepEqual(uniform.getQueuedAccelerations(), Array(6).fill(4));
    const beforeZero = uniform.getCell(0, 0), beforeDiag = uniform.getDiagnostics();
    for (const dt of [-1, NaN, Infinity, 100]) {
        assert.throws(() => uniform.step(dt)); assert.deepEqual(uniform.getCell(0, 0), beforeZero);
        assert.deepEqual(uniform.getDiagnostics(), beforeDiag);
    }
    uniform.setConfig({...uniformConfig, maxCellWork: 6});
    assert.throws(() => uniform.step(0.5)); assert.deepEqual(uniform.getQueuedAccelerations(), Array(6).fill(4));
    uniform.setConfig({...uniformConfig, maxSubsteps: 1});
    assert.throws(() => uniform.step(0.5)); assert.deepEqual(uniform.getCell(0, 0), beforeZero);
    uniform.setConfig(uniformConfig); uniform.step(0.5);
    for (const cell of uniform.getDisplacements()) assert.ok(Math.abs(cell - 2.5) < 1e-13);
    for (const speed of uniform.getVelocities()) assert.ok(Math.abs(speed) < 1e-13);
    assert.deepEqual(uniform.getQueuedAccelerations(), Array(6).fill(0));
    uniform.queueAcceleration(0, 0, 1e308);
    const overflowState = uniform.getDisplacements(), overflowDiag = uniform.getDiagnostics();
    assert.throws(() => uniform.step(0.01)); assert.deepEqual(uniform.getDisplacements(), overflowState);
    assert.deepEqual(uniform.getDiagnostics(), overflowDiag); assert.equal(uniform.getCell(0, 0).queuedAcceleration, 1e308);
    uniform.clearAcceleration(0, 0); uniform.queueAcceleration(0, 0, 1); uniform.clearAccelerations();
    assert.deepEqual(uniform.getQueuedAccelerations(), Array(6).fill(0));
    uniform.setConfig({...uniformConfig, damping: 0.7}); uniform.setState(Array(6).fill(0), Array(6).fill(2));
    uniform.step(0.5);
    for (const speed of uniform.getVelocities()) assert.ok(Math.abs(speed - 2 * Math.exp(-0.7)) < 2e-14);
    assert.ok(uniform.getDiagnostics().kineticEnergy < 6);

    defaults.setCellState(1, 1, 0.01); defaults.queueAcceleration(1, 1, 2); defaults.step(0.01);
    for (const [x, y] of [[0, 0], [0, 1], [2, 1], [1, 0], [1, 2]]) {
        assert.equal(defaults.getCell(x, y).displacement, 0); assert.equal(defaults.getCell(x, y).velocity, 0);
        assert.throws(() => defaults.setCellState(x, y, 1));
        assert.throws(() => defaults.setCellState(x, y, 0, 1));
        assert.throws(() => defaults.queueAcceleration(x, y, 1));
    }
    const edgesBefore = defaults.getDisplacements();
    assert.throws(() => defaults.setState(Array(9).fill(1), Array(9).fill(0)));
    assert.deepEqual(defaults.getDisplacements(), edgesBefore);
    defaults.setConfig({...defaults.getConfig(), boundary: physics.WaveBoundary.Periodic}); defaults.queueAcceleration(0, 0, 1);
    assert.throws(() => defaults.setConfig({...defaults.getConfig(), boundary: physics.WaveBoundary.FixedZero}));
    assert.equal(defaults.getConfig().boundary, physics.WaveBoundary.Periodic);
    defaults.clearAccelerations(); defaults.setConfig({...defaults.getConfig(), boundary: physics.WaveBoundary.FixedZero});
    mode.delete(); uniform.delete(); defaults.delete();
    assert.equal(cellSnapshot.displacement, 99); assert.equal(arraySnapshot[0], 99); assert.equal(loadSnapshot[0], 99);
    assert.equal(diag.totalEnergy, 99); assert.equal(config.maxSubstep, 0.001);
}

function testGravity(physics) {
    const defaults = new physics.NBodyGravity();
    const config = defaults.getConfig();
    assert.equal(config.gravitationalStrength, 1);
    config.maxSubstep = 0.002;
    assert.equal(defaults.getConfig().maxSubstep, 0.01);
    defaults.delete();
    const orbit = new physics.NBodyGravity(config);
    assert.equal(orbit.addParticle({x: -0.5, y: 0}, {x: 0, y: -Math.SQRT1_2}, 1), 0);
    assert.equal(orbit.addParticle({x: 0.5, y: 0}, {x: 0, y: Math.SQRT1_2}, 1), 1);
    assert.equal(orbit.getParticleCount(), 2);
    orbit.step(0.5);
    const p = orbit.getParticle(0), d = orbit.getDiagnostics();
    assert.ok(Math.hypot(p.position.x + 0.5 * Math.cos(Math.SQRT2 * 0.5),
                         p.position.y + 0.5 * Math.sin(Math.SQRT2 * 0.5)) < 3e-6);
    assert.ok(Math.abs(d.totalEnergy + 0.5) < 2e-6);
    assert.ok(Math.hypot(d.momentum.x, d.momentum.y) < 1e-13);
    assert.ok(d.lastSubsteps >= 250 && d.lastPairWork >= d.lastSubsteps);
    p.position.x = 99; d.centerOfMass.y = 99;
    assert.notEqual(orbit.getParticle(0).position.x, 99);
    assert.notEqual(orbit.getDiagnostics().centerOfMass.y, 99);
    const before = orbit.getParticle(0);
    for (const index of [-1, 0.5, NaN, Infinity, 2, 2**32]) {
        assert.throws(() => orbit.getParticle(index));
        assert.throws(() => orbit.setState(index, {x: 0, y: 0}, {x: 0, y: 0}));
        assert.throws(() => orbit.applyImpulse(index, {x: 0, y: 0}));
    }
    assert.throws(() => orbit.setState(0, {x: Infinity, y: 0}, {x: 0, y: 0}));
    for (const key of ["maxParticles", "maxSubsteps", "maxPairWork"])
        for (const value of [-1, 0, 1.5, NaN, Infinity, 2**32])
            assert.throws(() => orbit.setConfig({...config, [key]: value}));
    assert.throws(() => new physics.NBodyGravity({...config, maxPairWork: 0.5}));
    assert.deepEqual(orbit.getParticle(0), before);
    assert.deepEqual(orbit.getConfig(), config);
    orbit.setConfig({...config, maxPairWork: 1});
    assert.throws(() => orbit.step(0.002));
    assert.deepEqual(orbit.getParticle(0), before);
    assert.equal(orbit.getDiagnostics().lastPairWork, d.lastPairWork);
    orbit.setConfig(config);
    orbit.step(0);
    assert.deepEqual(orbit.getParticle(0), before);
    assert.equal(orbit.getDiagnostics().lastPairWork, 0);
    orbit.delete();
    assert.equal(p.position.x, 99); // Plain snapshot remains usable after deletion.
    assert.equal(before.mass, 1);

    const free = new physics.NBodyGravity({...config, gravitationalStrength: 0});
    free.addParticle({x: 1e10, y: 0});
    free.addParticle({x: 1e10, y: 0}); // G=0 coincidence is allowed.
    free.setState(0, {x: 1e10, y: 0}, {x: 0, y: 2});
    free.applyImpulse(0, {x: 0, y: 1});
    free.step(0.1);
    assert.ok(Math.abs(free.getParticle(0).position.y - 0.3) < 1e-13);
    assert.equal(free.getParticle(0).velocity.y, 3);
    free.delete();

    const singular = new physics.NBodyGravity(config);
    singular.addParticle({x: 0, y: 0}); singular.addParticle({x: 0, y: 0});
    assert.throws(() => singular.step(0.002));
    assert.equal(singular.getParticle(0).position.x, 0);
    singular.setConfig({...config, softening: 0.1});
    singular.step(0.002);
    assert.equal(singular.getParticle(0).velocity.x, 0);
    assert.ok(Math.abs(singular.getDiagnostics().potentialEnergy + 10) < 1e-12);
    singular.delete();
}

async function main() {
    const physics = await createPhysicsEngineModule();
    testGravity(physics);
    testWaves(physics);
    testQueries(physics);
    const integerEngine = new physics.Engine();
    const integerConfig = integerEngine.getSimulationConfig();
    for (const key of ["maxSubstepsPerAdvance", "solverIterations", "maximumCcdImpacts"])
        for (const value of [-1, 0, 0.5, 3.5, NaN, Infinity, 2**31, 2**32, 2**32 + 1]) {
            assert.throws(() => integerEngine.setSimulationConfig({...integerConfig, [key]: value}));
            assert.deepEqual(integerEngine.getSimulationConfig(), integerConfig);
        }
    const validCounts = {...integerConfig, maxSubstepsPerAdvance: 3, solverIterations: 7, maximumCcdImpacts: 9};
    integerEngine.setSimulationConfig(validCounts);
    assert.deepEqual(integerEngine.getSimulationConfig(), validCounts);
    integerEngine.delete();
    const indexedParticles = physics.createParticleSystem();
    indexedParticles.reserve(0); indexedParticles.reserve(2);
    indexedParticles.addParticle({x: 7, y: 0}, {x: 0, y: 0}, 2);
    for (const index of [-1, 0.5, NaN, Infinity, 1, 2**32, 2**32 + 1]) {
        assert.throws(() => indexedParticles.getParticlePosition(index));
        assert.throws(() => indexedParticles.getParticleVelocity(index));
        assert.throws(() => indexedParticles.applyForce(index, {x: 100, y: 0}));
        assert.throws(() => indexedParticles.removeParticle(index));
        assert.equal(indexedParticles.size(), 1);
    }
    for (const count of [-1, 0.5, NaN, Infinity, 2**32, 2**32 + 1])
        assert.throws(() => indexedParticles.reserve(count));
    indexedParticles.applyForce(0, {x: 2, y: 0}); indexedParticles.step(0.5);
    assert.ok(Math.abs(indexedParticles.getParticlePosition(0).x - 7.125) < 1e-6);
    assert.equal(indexedParticles.getParticleVelocity(0).x, 0.5);
    indexedParticles.removeParticle(0); assert.equal(indexedParticles.size(), 0);
    indexedParticles.delete();
    const soft = new physics.SoftBody();
    const softConfig = soft.getConfig();
    assert.equal(softConfig.maxParticles, 100000);
    softConfig.maxSubstep = 0.001;
    assert.equal(soft.getConfig().maxSubstep, 0.01);
    const oscillator = new physics.SoftBody(softConfig);
    assert.equal(oscillator.addParticle({x: 0, y: 0}, {x: 0, y: 0}, 1, true), 0);
    assert.equal(oscillator.addParticle({x: 1.2, y: 0}, {x: 0, y: 0}, 1, false), 1);
    assert.equal(oscillator.addSpring(0, 1, 1, 4), 0);
    oscillator.step(1);
    assert.ok(Math.abs(oscillator.getParticle(1).position.x - (1 + 0.2 * Math.cos(2))) < 2e-5);
    assert.ok(Math.abs(oscillator.getParticle(1).velocity.x + 0.4 * Math.sin(2)) < 2e-5);
    assert.equal(oscillator.getParticleCount(), 2);
    assert.equal(oscillator.getSpringCount(), 1);
    assert.equal(oscillator.getSpring(0).stiffness, 4);
    assert.ok(oscillator.getDiagnostics().lastSubsteps >= 1000);
    assert.ok(Math.abs(oscillator.getDiagnostics().kineticEnergy + oscillator.getDiagnostics().elasticEnergy - 0.08) < 2e-6);
    const softSnapshot = oscillator.getParticle(1);
    softSnapshot.position.x = 99; softSnapshot.force.x = 99;
    assert.notEqual(oscillator.getParticle(1).position.x, 99);
    assert.equal(oscillator.getAccumulatedForce(1).x, 0);
    const springCopy = oscillator.getSpring(0); springCopy.stiffness = 99;
    assert.equal(oscillator.getSpring(0).stiffness, 4);
    const softDiagnosticsCopy = oscillator.getDiagnostics(); softDiagnosticsCopy.totalMass = 99;
    assert.equal(oscillator.getDiagnostics().totalMass, 2);
    for (const index of [-1, 0.5, NaN, Infinity, 2, 2**32, 2**32 + 1]) {
        assert.throws(() => oscillator.getParticle(index));
        assert.throws(() => oscillator.getAccumulatedForce(index));
        assert.throws(() => oscillator.setParticleState(index, {x: 0, y: 0}, {x: 0, y: 0}));
        assert.throws(() => oscillator.setFixed(index, false));
        assert.throws(() => oscillator.applyImpulse(index, {x: 1, y: 0}));
        assert.throws(() => oscillator.applyForce(index, 1, 0));
        assert.throws(() => oscillator.clearForces(index));
        assert.throws(() => oscillator.addSpring(index, 1, 1, 1, 0));
        assert.throws(() => oscillator.addSpring(0, index, 1, 1));
    }
    for (const index of [-1, 0.5, NaN, Infinity, 1, 2**32]) assert.throws(() => oscillator.getSpring(index));
    for (const field of ["maxSubsteps", "maxParticles", "maxSprings"]) {
        for (const count of [-1, 0.5, NaN, Infinity, 0, 2**32, 2**32 + 1]) {
            const invalid = {...softConfig, [field]: count};
            assert.throws(() => oscillator.setConfig(invalid));
            assert.throws(() => new physics.SoftBody(invalid));
            assert.equal(oscillator.getConfig()[field], softConfig[field]);
        }
    }
    assert.throws(() => oscillator.setUniformAcceleration({x: NaN, y: 0}));
    assert.throws(() => oscillator.setParticleState(1, {x: NaN, y: 0}));
    const forceConfig = soft.getConfig(); forceConfig.maxSubsteps = 2;
    soft.setConfig(forceConfig);
    soft.addParticle({x: 0, y: 0}, {x: 0, y: 0}, 2, false);
    soft.applyForce(0, 4, 0); soft.step(0);
    assert.equal(soft.getAccumulatedForce(0).x, 4);
    const forceBefore = soft.getParticle(0), diagnosticsBefore = soft.getDiagnostics();
    for (const dt of [-1, NaN, Infinity, 0.5]) {
        assert.throws(() => soft.step(dt));
        assert.deepEqual(soft.getParticle(0), forceBefore);
        assert.deepEqual(soft.getDiagnostics(), diagnosticsBefore);
    }
    forceConfig.maxSubsteps = 100; soft.setConfig(forceConfig); soft.step(0.5);
    assert.ok(Math.abs(soft.getParticle(0).position.x - 0.25) < 1e-6);
    assert.ok(Math.abs(soft.getParticle(0).velocity.x - 1) < 1e-6);
    assert.equal(soft.getAccumulatedForce(0).x, 0);
    soft.applyImpulse(0, {x: 2, y: 0}); assert.equal(soft.getParticle(0).velocity.x, 2);
    soft.applyForce(0, {x: 3, y: 0}); soft.clearForces(0); assert.equal(soft.getAccumulatedForce(0).x, 0);
    soft.applyForce(0, 3, 0); soft.clearForces(); assert.equal(soft.getAccumulatedForce(0).x, 0);
    soft.setUniformAcceleration({x: 0, y: -2}); assert.equal(soft.getUniformAcceleration().y, -2);
    soft.setFixed(0, true); soft.setParticleState(0, {x: 1, y: 0});
    soft.step(0.1); assert.equal(soft.getParticle(0).position.x, 1);
    const retainedSoft = soft.getParticle(0), retainedSpring = oscillator.getSpring(0);
    soft.delete(); oscillator.delete();
    assert.equal(retainedSoft.position.x, 1); assert.equal(retainedSpring.first, 0);
    assert.equal(softSnapshot.position.x, 99); // Deep JS snapshot survives deletion.
    const boundedSoft = new physics.SoftBody({...softConfig, maxParticles: 2, maxSprings: 1});
    boundedSoft.addParticle({x: 0, y: 0}); boundedSoft.addParticle({x: 1, y: 0});
    boundedSoft.addSpring(0, 1, 1, 1);
    assert.throws(() => boundedSoft.addParticle({x: 2, y: 0}));
    assert.throws(() => boundedSoft.addSpring(1, 0, 1, 1));
    assert.throws(() => boundedSoft.setConfig({...softConfig, maxParticles: 1}));
    assert.equal(boundedSoft.getParticleCount(), 2); assert.equal(boundedSoft.getSpringCount(), 1);
    boundedSoft.setConfig({...softConfig, maxParticles: 3, maxSprings: 1});
    boundedSoft.addParticle({x: 2, y: 0}); assert.throws(() => boundedSoft.addSpring(1, 2, 1, 1));
    assert.equal(boundedSoft.getSpringCount(), 1);
    boundedSoft.delete();
    const thermal = new physics.ThermalNetwork();
    const thermalConfig = thermal.getConfig();
    assert.equal(thermalConfig.maxNodes, 100000);
    const conduction = new physics.ThermalNetwork(thermalConfig);
    assert.equal(conduction.addNode(400, 2), 0); assert.equal(conduction.addNode(300, 3, false), 1);
    assert.equal(conduction.addLink(0, 1, 1), 0);
    const energyBefore = conduction.getDiagnostics().totalEnergy;
    conduction.step(1);
    const heatA = conduction.getNode(0), heatB = conduction.getNode(1);
    assert.ok(heatA.temperature < 400 && heatA.temperature > heatB.temperature);
    assert.ok(heatB.temperature > 300);
    assert.ok(Math.abs(conduction.getDiagnostics().totalEnergy - energyBefore) < 1e-10);
    assert.equal(conduction.getNodeCount(), 2); assert.equal(conduction.getLinkCount(), 1);
    assert.equal(conduction.getLink(0).conductance, 1);
    const linkCopy = conduction.getLink(0); linkCopy.conductance = 99;
    assert.equal(conduction.getLink(0).conductance, 1);
    const thermalDiagnosticsCopy = conduction.getDiagnostics(); thermalDiagnosticsCopy.totalEnergy = 99;
    assert.ok(Math.abs(conduction.getDiagnostics().totalEnergy - energyBefore) < 1e-10);
    heatA.temperature = 99; assert.notEqual(conduction.getNode(0).temperature, 99);
    for (const index of [-1, 0.5, NaN, Infinity, 2, 2**32, 2**32 + 1]) {
        assert.throws(() => conduction.getNode(index));
        assert.throws(() => conduction.setTemperature(index, 300));
        assert.throws(() => conduction.setFixed(index, true));
        assert.throws(() => conduction.applyPower(index, 1));
        assert.throws(() => conduction.clearPowers(index));
        assert.throws(() => conduction.addLink(index, 1, 1));
        assert.throws(() => conduction.addLink(0, index, 1));
    }
    for (const index of [-1, 0.5, NaN, Infinity, 1, 2**32]) assert.throws(() => conduction.getLink(index));
    for (const field of ["maxSubsteps", "maxNodes", "maxLinks"]) {
        for (const count of [-1, 0.5, NaN, Infinity, 0, 2**32, 2**32 + 1]) {
            const invalid = {...thermalConfig, [field]: count};
            assert.throws(() => conduction.setConfig(invalid));
            assert.throws(() => new physics.ThermalNetwork(invalid));
            assert.equal(conduction.getConfig()[field], thermalConfig[field]);
        }
    }
    assert.throws(() => conduction.setTemperature(0, -1));
    assert.throws(() => conduction.addLink(1, 0, 1));
    assert.throws(() => conduction.applyPower(0, Infinity));
    assert.throws(() => new physics.ThermalNetwork({...thermalConfig, safetyFactor: 2}));
    thermal.addNode(300, 2); thermal.applyPower(0, 4); thermal.step(0);
    assert.equal(thermal.getNode(0).externalPower, 4);
    const heatingConfig = thermal.getConfig(); heatingConfig.maxSubsteps = 2; thermal.setConfig(heatingConfig);
    const heatingBefore = thermal.getNode(0), heatingDiagnostics = thermal.getDiagnostics();
    for (const dt of [-1, NaN, Infinity, 0.5]) {
        assert.throws(() => thermal.step(dt));
        assert.deepEqual(thermal.getNode(0), heatingBefore);
        assert.deepEqual(thermal.getDiagnostics(), heatingDiagnostics);
    }
    heatingConfig.maxSubsteps = 100; thermal.setConfig(heatingConfig); thermal.step(0.5);
    assert.ok(Math.abs(thermal.getNode(0).temperature - 301) < 1e-10);
    assert.equal(thermal.getNode(0).externalPower, 0);
    assert.ok(Math.abs(thermal.getDiagnostics().totalExternalEnergy - 2) < 1e-12);
    thermal.applyPower(0, -10000);
    const coolingBefore = thermal.getNode(0), coolingDiagnostics = thermal.getDiagnostics();
    assert.throws(() => thermal.step(0.5));
    assert.deepEqual(thermal.getNode(0), coolingBefore); assert.deepEqual(thermal.getDiagnostics(), coolingDiagnostics);
    thermal.clearPowers(0); assert.equal(thermal.getNode(0).externalPower, 0);
    thermal.applyPower(0, 1); thermal.clearPowers(); assert.equal(thermal.getNode(0).externalPower, 0);
    thermal.setFixed(0, true); thermal.applyPower(0, 4); thermal.step(0.5);
    assert.ok(Math.abs(thermal.getNode(0).temperature - 301) < 1e-10);
    assert.ok(Math.abs(thermal.getNode(0).reservoirHeat + 2) < 1e-12);
    assert.ok(Math.abs(thermal.getDiagnostics().totalReservoirHeat + 2) < 1e-12);
    thermal.setFixed(0, false); thermal.setTemperature(0, 350);
    const retainedNode = thermal.getNode(0), retainedLink = conduction.getLink(0);
    thermal.delete(); conduction.delete();
    assert.equal(retainedNode.temperature, 350); assert.equal(retainedLink.second, 1); assert.equal(heatA.temperature, 99);
    const boundedHeat = new physics.ThermalNetwork({...thermalConfig, maxNodes: 2, maxLinks: 1});
    boundedHeat.addNode(300, 1); boundedHeat.addNode(300, 1); boundedHeat.addLink(0, 1, 1);
    assert.throws(() => boundedHeat.addNode(300, 1));
    assert.throws(() => boundedHeat.setConfig({...thermalConfig, maxNodes: 1}));
    assert.equal(boundedHeat.getNodeCount(), 2); assert.equal(boundedHeat.getLinkCount(), 1);
    boundedHeat.setConfig({...thermalConfig, maxNodes: 3, maxLinks: 1});
    boundedHeat.addNode(300, 1); assert.throws(() => boundedHeat.addLink(1, 2, 1));
    assert.equal(boundedHeat.getLinkCount(), 1);
    boundedHeat.delete();
    const charge = new physics.ChargedParticle({x: 0, y: 0}, {x: 2, y: 0}, 3, 6);
    charge.step(Math.PI / 8, {electric: {x: 0, y: 0}, magnetic: 2});
    assert.ok(Math.abs(charge.getPosition().x - 0.5) < 1e-12);
    assert.ok(Math.abs(charge.getPosition().y + 0.5) < 1e-12);
    assert.ok(Math.abs(charge.getVelocity().y + 2) < 1e-12);
    assert.ok(Math.abs(charge.getKineticEnergy() - 6) < 1e-12);
    assert.equal(charge.getMass(), 3);
    assert.equal(charge.getCharge(), 6);
    const snapshot = charge.getPosition();
    snapshot.x = 99; // Observers return JS values, not borrowed mutable state.
    assert.ok(Math.abs(charge.getPosition().x - 0.5) < 1e-12);
    assert.throws(() => charge.setState({x: NaN, y: 0}, {x: 0, y: 0}));
    assert.throws(() => charge.step(-1, {electric: {x: 0, y: 0}, magnetic: 0}));
    charge.setState({x: 0, y: 0}, {x: 0, y: 0});
    charge.step(0.5, {electric: {x: 2, y: 0}, magnetic: 0});
    assert.ok(Math.abs(charge.getPosition().x - 0.5) < 1e-12);
    assert.ok(Math.abs(charge.getVelocity().x - 2) < 1e-12);
    charge.delete();
    const neutral = new physics.ChargedParticle();
    assert.equal(neutral.getCharge(), 0);
    neutral.delete();
    const engine = new physics.Engine();
    const simulationConfig = engine.getSimulationConfig();
    assert.equal(simulationConfig.solverIterations, 10);
    assert.ok(Math.abs(simulationConfig.fixedTimeStep - 1 / 60) < 0.000001);
    assert.equal(simulationConfig.maxSubstepsPerAdvance, 8);
    assert.equal(simulationConfig.restitutionVelocityThreshold, 1.0);
    assert.ok(Math.abs(simulationConfig.velocityTolerance - 0.0001) < 0.000001);
    assert.ok(Math.abs(simulationConfig.maxPositionCorrection - 0.2) < 0.000001);
    assert.equal(simulationConfig.enableLinearVelocityLimit, true);
    simulationConfig.solverIterations = 4;
    simulationConfig.restitutionVelocityThreshold = 0.75;
    simulationConfig.enableLinearVelocityLimit = false;
    simulationConfig.enableAngularVelocityLimit = false;
    engine.setSimulationConfig(simulationConfig);
    const configuredSimulation = engine.getSimulationConfig();
    assert.equal(configuredSimulation.solverIterations, 4);
    assert.equal(configuredSimulation.restitutionVelocityThreshold, 0.75);
    assert.equal(configuredSimulation.enableLinearVelocityLimit, false);
    assert.equal(configuredSimulation.enableAngularVelocityLimit, false);
    const shape = new physics.Circle(1.0);
    const body = physics.createRigidBody(
        shape,
        {
            density: 1.0,
            restitution: 0.5,
            staticFriction: 0.6,
            dynamicFriction: 0.4,
        },
        { x: 0.0, y: 0.0 },
        false,
    );

    // The body owns a copy: deleting the JavaScript shape must be safe.
    shape.delete();

    body.setCollisionCategoryBits(0x00000002);
    body.setCollisionMaskBits(0x00000004);
    assert.equal(body.getCollisionCategoryBits(), 0x00000002);
    assert.equal(body.getCollisionMaskBits(), 0x00000004);

    body.setVelocity({ x: 3.0, y: 0.0 });
    engine.addBody(body);
    engine.step(0.5);

    const position = body.getPosition();
    assert.ok(Math.abs(position.x - 1.5) < 0.0001, `unexpected x position: ${position.x}`);
    assert.equal(position.y, 0);

    const particles = physics.createParticleSystem();
    particles.addParticle({ x: 0.0, y: 0.0 }, { x: 4.0, y: 0.0 }, 1.0);
    engine.addParticleSystem(particles);
    engine.step(0.25);
    const particlePosition = particles.getParticlePosition(0);
    assert.ok(
        Math.abs(particlePosition.x - 1.0) < 0.0001,
        `unexpected particle x position: ${particlePosition.x}`,
    );

    simulationConfig.fixedTimeStep = 0.1;
    simulationConfig.maxSubstepsPerAdvance = 2;
    engine.setSimulationConfig(simulationConfig);
    engine.resetTiming();
    const fixedProgress = engine.advance(0.25);
    assert.equal(fixedProgress.stepsPerformed, 2);
    assert.ok(Math.abs(fixedProgress.simulatedTime - 0.2) < 0.000001);
    assert.ok(Math.abs(fixedProgress.remainingTime - 0.05) < 0.000001);
    assert.equal(engine.getTotalStepCount(), 2n);

    const caughtUp = engine.advance(0.05);
    assert.equal(caughtUp.stepsPerformed, 1);
    assert.ok(engine.getAccumulatedTime() < 0.000001);
    const statistics = engine.getLastStepStatistics();
    assert.equal(statistics.integratedBodyCount, 1);
    assert.equal(statistics.integratedParticleCount, 1);
    assert.equal(statistics.broadPhaseCandidateCount, 0);
    assert.equal(statistics.resolvedContactCount, 0);
    assert.equal(statistics.solverIterationCount, 4);
    assert.equal(statistics.fluidIterationCount, 0);

    const exported = JSON.parse(engine.exportJson(1));
    assert.equal(exported.schemaVersion, 1);
    assert.equal(exported.bodies.length, 1);
    assert.equal(exported.bodies[0].shape.type, "circle");
    assert.equal(typeof exported.bodies[0].id, "string");
    assert.ok(engine.exportCsv(1).startsWith("time,id,"));
    assert.throws(() => body.setVelocity({ x: NaN, y: 0 }));
    const box = physics.Polygon.makeBox(1, 1);
    const second = physics.createRigidBody(box, {density: 1, restitution: 0,
        staticFriction: 0, dynamicFriction: 0}, {x: 8, y: 0}, false);
    box.delete();
    second.setCollisionMaskBits(0);
    engine.addBody(second);
    const joint = physics.createDistanceJoint(body, second, 2, {x: 0, y: 0}, {x: 0, y: 0});
    engine.addJoint(joint);
    // World and joint retain their shared bodies after JS handles are deleted.
    second.delete(); body.delete(); joint.delete();
    for (let i = 0; i < 20; ++i) engine.stepFixed();
    assert.equal(JSON.parse(engine.exportJson(2)).bodies.length, 2);
    engine.clearBodies();
    assert.equal(JSON.parse(engine.exportJson(2)).bodies.length, 0);
    const rotorShape = new physics.Circle(1);
    const material = {density: 1, restitution: 0, staticFriction: 0, dynamicFriction: 0};
    const support = physics.createRigidBody(rotorShape, material, {x: 0, y: 0}, true);
    const rotor = physics.createRigidBody(rotorShape, material, {x: 0, y: 0}, false);
    rotorShape.delete();
    rotor.setMass(2); // Unit-radius disk: I = m*r^2/2 = 1.
    rotor.setCollisionMaskBits(0);
    engine.addBody(support); engine.addBody(rotor);
    const hinge = physics.createRevoluteJoint(support, rotor, {x: 0, y: 0}, {x: 0, y: 0});
    assert.equal(hinge.isMotorEnabled(), false);
    assert.equal(hinge.areLimitsEnabled(), false);
    hinge.setMotor(true, 10, 2);
    assert.equal(hinge.getMotorSpeed(), 10);
    assert.equal(hinge.getMaxMotorTorque(), 2);
    engine.addJoint(hinge);
    engine.step(0.25);
    assert.ok(Math.abs(rotor.getAngularVelocity() - 0.5) < 1e-6);
    assert.ok(Math.abs(hinge.getMotorTorque() - 2) < 1e-6);
    engine.step(0);
    assert.equal(hinge.getMotorTorque(), 0);
    hinge.setLimits(true, -0.1, 0.2);
    assert.equal(hinge.areLimitsEnabled(), true);
    assert.ok(Math.abs(hinge.getLowerLimit() + 0.1) < 1e-6);
    assert.ok(Math.abs(hinge.getUpperLimit() - 0.2) < 1e-6);
    assert.throws(() => hinge.setLimits(true, 2, 1));
    assert.throws(() => hinge.setMotor(true, NaN, 1));
    support.delete(); rotor.delete();
    for (let i = 0; i < 300; ++i) engine.step(1 / 120);
    assert.ok(Math.abs(hinge.getAngle() - 0.2) < 1e-5);
    assert.ok(Math.abs(hinge.getAnchorA().x - hinge.getAnchorB().x) < 1e-6);
    engine.removeJoint(hinge);
    hinge.delete();
    engine.clearBodies();
    const sliderShape = new physics.Circle(0.1);
    const rail = physics.createRigidBody(sliderShape, material, {x: 0, y: 0}, true);
    const carriage = physics.createRigidBody(sliderShape, material, {x: 0, y: 0}, false);
    sliderShape.delete();
    carriage.setMass(2);
    carriage.setCollisionMaskBits(0);
    engine.addBody(rail); engine.addBody(carriage);
    const slider = physics.createPrismaticJoint(rail, carriage, {x: 1, y: 0}, {x: 0, y: 0}, {x: 0, y: 0});
    assert.equal(slider.isMotorEnabled(), false);
    assert.equal(slider.areLimitsEnabled(), false);
    slider.setMotor(true, 10, 4);
    engine.addJoint(slider);
    engine.step(0.25);
    assert.ok(Math.abs(slider.getTranslationSpeed() - 0.5) < 1e-6);
    assert.ok(Math.abs(slider.getMotorForce() - 4) < 1e-6);
    slider.setLimits(true, -0.1, 0.2);
    assert.throws(() => slider.setLimits(true, 2, 1));
    assert.throws(() => slider.setMotor(true, 1, -1));
    assert.equal(slider.getMaxMotorForce(), 4);
    assert.equal(slider.getMotorSpeed(), 10);
    assert.equal(slider.areLimitsEnabled(), true);
    rail.delete(); carriage.delete();
    for (let i = 0; i < 300; ++i) engine.step(1 / 120);
    assert.ok(Math.abs(slider.getTranslation() - 0.2) < 1e-5);
    assert.ok(Math.abs(slider.getTransverseError()) < 1e-6);
    assert.ok(Math.abs(slider.getAngle()) < 1e-6);
    engine.step(0);
    assert.equal(slider.getMotorForce(), 0);
    engine.clearBodies();
    assert.ok(Math.abs(slider.getTranslation() - 0.2) < 1e-5); // Joint retains both endpoints.
    slider.delete();
    engine.delete();
    particles.delete();
    console.log("PASS: configuration, stepping, filtering, lifetimes, owned spatial queries, joint motors/limits, exports, particles, electromagnetic motion, soft-body oscillator/loads, thermal conservation/accounting, N-body gravity, and membrane waves");
}

main().catch((error) => {
    console.error(error);
    process.exitCode = 1;
});

const assert = require("node:assert/strict");
const createPhysicsEngineModule = require("./physics_engine.js");

async function main() {
    const physics = await createPhysicsEngineModule();
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
    engine.delete();
    particles.delete();
    console.log("PASS: configuration, stepping, filtering, shape/body/joint lifetimes, exports, and particles");
}

main().catch((error) => {
    console.error(error);
    process.exitCode = 1;
});

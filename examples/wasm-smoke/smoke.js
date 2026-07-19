async function runSmokeTest() {
    const output = document.querySelector("#output");

    try {
        const physics = await createPhysicsEngineModule();
        const engine = new physics.Engine();
        const simulationConfig = engine.getSimulationConfig();
        simulationConfig.solverIterations = 4;
        simulationConfig.restitutionVelocityThreshold = 0.75;
        simulationConfig.fixedTimeStep = 0.1;
        simulationConfig.maxSubstepsPerAdvance = 2;
        simulationConfig.enableLinearVelocityLimit = false;
        simulationConfig.enableAngularVelocityLimit = false;
        engine.setSimulationConfig(simulationConfig);
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

        body.setCollisionCategoryBits(0x00000002);
        body.setCollisionMaskBits(0x00000004);
        body.setVelocity({ x: 3.0, y: 0.0 });
        engine.addBody(body);
        engine.step(0.5);

        const particles = physics.createParticleSystem();
        particles.addParticle({ x: 0.0, y: 0.0 }, { x: 4.0, y: 0.0 }, 1.0);
        engine.addParticleSystem(particles);
        engine.step(0.25);
        const position = body.getPosition();
        const particlePosition = particles.getParticlePosition(0);

        engine.resetTiming();
        const fixedProgress = engine.advance(0.25);
        const statistics = engine.getLastStepStatistics();

        const passed = Math.abs(position.x - 2.25) < 0.0001
            && position.y === 0
            && Math.abs(particlePosition.x - 1.0) < 0.0001
            && body.getCollisionCategoryBits() === 0x00000002
            && body.getCollisionMaskBits() === 0x00000004
            && engine.getSimulationConfig().solverIterations === 4
            && engine.getSimulationConfig().restitutionVelocityThreshold === 0.75
            && fixedProgress.stepsPerformed === 2
            && Math.abs(fixedProgress.remainingTime - 0.05) < 0.000001
            && engine.getTotalStepCount() === 2n
            && statistics.integratedBodyCount === 1
            && statistics.integratedParticleCount === 1
            && statistics.solverIterationCount === 4
            && statistics.fluidIterationCount === 0;
        output.textContent = passed
            ? "PASS: configuration, fixed stepping, filtering, bodies, and particles"
            : `FAIL: body=(${position.x}, ${position.y}), particle=(${particlePosition.x}, ${particlePosition.y})`;
        output.dataset.result = passed ? "pass" : "fail";

        engine.delete();
        particles.delete();
        body.delete();
        shape.delete();
    } catch (error) {
        output.textContent = `FAIL: ${error instanceof Error ? error.message : String(error)}`;
        output.dataset.result = "fail";
        throw error;
    }
}

runSmokeTest();

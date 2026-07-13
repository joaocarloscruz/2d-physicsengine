async function runSmokeTest() {
    const output = document.querySelector("#output");

    try {
        const physics = await createPhysicsEngineModule();
        const engine = new physics.Engine();
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

        body.setVelocity({ x: 3.0, y: 0.0 });
        engine.addBody(body);
        engine.step(0.5);

        const position = body.getPosition();
        const particles = physics.createParticleSystem();
        particles.addParticle({ x: 0.0, y: 0.0 }, { x: 4.0, y: 0.0 }, 1.0);
        engine.addParticleSystem(particles);
        engine.step(0.25);
        const particlePosition = particles.getParticlePosition(0);

        const passed = Math.abs(position.x - 2.25) < 0.0001
            && position.y === 0
            && Math.abs(particlePosition.x - 1.0) < 0.0001;
        output.textContent = passed
            ? "PASS: rigid body and particle system stepped"
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

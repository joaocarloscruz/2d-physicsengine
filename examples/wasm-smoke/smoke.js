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
        const passed = Math.abs(position.x - 1.5) < 0.0001 && position.y === 0;
        output.textContent = passed
            ? `PASS: body moved to (${position.x}, ${position.y})`
            : `FAIL: unexpected position (${position.x}, ${position.y})`;
        output.dataset.result = passed ? "pass" : "fail";

        engine.delete();
        body.delete();
        shape.delete();
    } catch (error) {
        output.textContent = `FAIL: ${error instanceof Error ? error.message : String(error)}`;
        output.dataset.result = "fail";
        throw error;
    }
}

runSmokeTest();

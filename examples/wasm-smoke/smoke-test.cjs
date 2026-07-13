const assert = require("node:assert/strict");
const createPhysicsEngineModule = require("./physics_engine.js");

async function main() {
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

    engine.delete();
    particles.delete();
    body.delete();
    shape.delete();
    console.log("PASS: rigid body and particle system stepped");
}

main().catch((error) => {
    console.error(error);
    process.exitCode = 1;
});

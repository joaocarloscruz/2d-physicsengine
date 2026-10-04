#include <physics/physics.h>
#include <cmath>
int main() {
    const auto triangle = PhysicsEngine::Polygon::MakeTriangle({-2, 0}, {2, 0}, {0, 5});
    if (std::abs(triangle.GetCentroid().y - 5.0 / 3.0) > 1e-6
        || std::abs(triangle.Recentered().GetInertia(1) - 37.0 / 18.0) > 1e-6) return 1;
    PhysicsEngine::World world;
    auto body = std::make_shared<PhysicsEngine::RigidBody>(PhysicsEngine::Circle(1), PhysicsEngine::Material{});
    body->SetVelocity({2, 0}); world.addBody(body); world.step(0.5f);
    const auto hit = PhysicsEngine::RayCastNearest(world, {-2, 0}, {4, 0});
    const auto points = PhysicsEngine::QueryPoint(world, {1, 0});
    const auto circles = PhysicsEngine::QueryCircle(world, {1, 0}, 0.25f);
    const auto circleHit = PhysicsEngine::SweepCircleNearest(world, {-2, 0}, {4, 0}, 0.5f);
    PhysicsEngine::Engine engine;
    engine.addBody(body);
    if (PhysicsEngine::QueryPoint(engine, {1, 0}) != points
        || PhysicsEngine::QueryCircle(engine, {1, 0}, 0.25f) != circles
        || PhysicsEngine::RayCastAll(engine, {-2, 0}, {4, 0}).size() != 1
        || PhysicsEngine::SweepCircleAll(engine, {-2, 0}, {4, 0}, 0.5f).size() != 1) return 1;
    const auto engineRay = PhysicsEngine::RayCastNearest(engine, {-2, 0}, {4, 0});
    const auto engineSweep = PhysicsEngine::SweepCircleNearest(engine, {-2, 0}, {4, 0}, 0.5f);
    engine.clearBodies();
    if (!engineRay || !engineSweep || engineRay->body != body || engineSweep->body != body
        || !PhysicsEngine::QueryPoint(engine, {1, 0}).empty()
        || std::abs(engineRay->hit.fraction - 1.0 / 3.0) > 1e-6
        || std::abs(engineSweep->hit.fraction - 0.25) > 1e-6) return 1;
    auto support = std::make_shared<PhysicsEngine::RigidBody>(PhysicsEngine::Circle(1), PhysicsEngine::Material{}, PhysicsEngine::Vector2{}, true);
    PhysicsEngine::PrismaticJoint slider(support, body);
    if (std::abs(slider.getTranslation() - 1) > 1e-6) return 1;
    slider.setMotor(true, 2, 4); slider.setLimits(true, -1, 3);
    if (!slider.isMotorEnabled() || !slider.areLimitsEnabled() || slider.getMotorForce() != 0) return 1;
    PhysicsEngine::ChargedParticle charge({}, {1, 0}, 1, 1);
    charge.step(0.5, {{}, 2});
    PhysicsEngine::SoftBody soft;
    soft.addParticle({}, {2, 0});
    soft.step(0.25);
    PhysicsEngine::ThermalNetwork thermal;
    thermal.addNode(300, 2);
    thermal.applyPower(0, 4);
    thermal.step(0.5);
    PhysicsEngine::NBodyGravity gravity;
    gravity.addParticle({}, {2, 0});
    gravity.step(0.25);
    PhysicsEngine::WaveMembraneConfig waveConfig;
    waveConfig.boundary = PhysicsEngine::WaveBoundary::Periodic;
    PhysicsEngine::WaveMembrane wave(2, 2, 1, 1, waveConfig);
    wave.setState(std::vector<double>(4), std::vector<double>(4, 2));
    wave.step(0.25);
    PhysicsEngine::PeriodicMacGridConfig macConfig;
    macConfig.columns=2; macConfig.rows=2; macConfig.spacingX=.25; macConfig.spacingY=.5;
    PhysicsEngine::PeriodicMacGrid mac(macConfig);
    PhysicsEngine::MacVelocityState macState; macState.xFaces={1,-1,1,-1}; macState.yFaces.assign(4,.5);
    mac.setVelocities(macState);
    const auto projection=mac.project();
    if(projection.finalDivergenceRms>projection.targetDivergenceRms
        ||std::abs(mac.velocities().xFaces[0])>1e-12
        ||mac.velocities().yFaces!=macState.yFaces) return 1;
    mac.setVelocities(macState);
    PhysicsEngine::MacDiffusionConfig diffusionOptions;
    diffusionOptions.kinematicViscosity=.25; diffusionOptions.timeStep=.5;
    const auto diffusion=mac.diffuse(diffusionOptions);
    const double expectedDiffused=1/(1+.25*.5*4/(.25*.25));
    if(std::abs(mac.velocities().xFaces[0]-expectedDiffused)>1e-12
        ||mac.velocities().yFaces!=macState.yFaces
        ||diffusion.finalResidualRms>diffusion.targetResidualRms
        ||mac.lastDiffusion().cellVisits!=diffusion.cellVisits
        ||mac.lastProjection().diagnostics.cellVisits!=projection.cellVisits) return 1;
    if (std::abs(wave.getDisplacements()[0] - 0.5) > 1e-12
        || std::abs(wave.getDiagnostics().kineticEnergy - 8) > 1e-12) return 1;
    return std::abs(body->position.x-1) < 1e-6f
        && !PhysicsEngine::ExportWorldJson(world).empty()
        && hit && hit->body == body && std::abs(hit->hit.fraction - 1.0 / 3.0) < 1e-6
        && points.size() == 1 && points[0] == body
        && circles.size() == 1 && circles[0] == body
        && circleHit && circleHit->body == body && std::abs(circleHit->hit.fraction - 0.25) < 1e-6
        && std::abs(circleHit->hit.center.x + 0.5f) < 1e-6
        && std::abs(circleHit->hit.contactPoint.x) < 1e-6
        && std::abs(charge.getVelocity().x - std::cos(1.0)) < 1e-12
        && std::abs(soft.getParticles()[0].position.x - 0.5f) < 1e-6f
        && std::abs(thermal.getNodes()[0].temperature - 301) < 1e-10
        && std::abs(gravity.getParticles()[0].position.x - 0.5) < 1e-12 ? 0 : 1;
}

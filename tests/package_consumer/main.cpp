#include <physics/physics.h>
#include <cmath>
int main() {
    PhysicsEngine::World world;
    auto body = std::make_shared<PhysicsEngine::RigidBody>(PhysicsEngine::Circle(1), PhysicsEngine::Material{});
    body->SetVelocity({2, 0}); world.addBody(body); world.step(0.5f);
    const auto hit = PhysicsEngine::RayCastNearest(world, {-2, 0}, {4, 0});
    const auto points = PhysicsEngine::QueryPoint(world, {1, 0});
    auto support = std::make_shared<PhysicsEngine::RigidBody>(PhysicsEngine::Circle(1), PhysicsEngine::Material{}, PhysicsEngine::Vector2{}, true);
    PhysicsEngine::PrismaticJoint slider(support, body);
    if (std::abs(slider.getTranslation() - 1) > 1e-6) return 1;
    PhysicsEngine::ChargedParticle charge({}, {1, 0}, 1, 1);
    charge.step(0.5, {{}, 2});
    PhysicsEngine::SoftBody soft;
    soft.addParticle({}, {2, 0});
    soft.step(0.25);
    PhysicsEngine::ThermalNetwork thermal;
    thermal.addNode(300, 2);
    thermal.applyPower(0, 4);
    thermal.step(0.5);
    return std::abs(body->position.x-1) < 1e-6f
        && !PhysicsEngine::ExportWorldJson(world).empty()
        && hit && hit->body == body && std::abs(hit->hit.fraction - 1.0 / 3.0) < 1e-6
        && points.size() == 1 && points[0] == body
        && std::abs(charge.getVelocity().x - std::cos(1.0)) < 1e-12
        && std::abs(soft.getParticles()[0].position.x - 0.5f) < 1e-6f
        && std::abs(thermal.getNodes()[0].temperature - 301) < 1e-10 ? 0 : 1;
}

#include <physics/physics.h>
#include <cmath>
int main() {
    PhysicsEngine::World world;
    auto body = std::make_shared<PhysicsEngine::RigidBody>(PhysicsEngine::Circle(1), PhysicsEngine::Material{});
    body->SetVelocity({2, 0}); world.addBody(body); world.step(0.5f);
    return std::abs(body->position.x-1) < 1e-6f && !PhysicsEngine::ExportWorldJson(world).empty() ? 0 : 1;
}

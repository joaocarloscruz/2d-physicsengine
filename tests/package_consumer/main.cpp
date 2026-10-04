#include <physics/physics.h>
#include <cmath>
int main() {
    PhysicsEngine::World world;
    auto body = std::make_shared<PhysicsEngine::RigidBody>(PhysicsEngine::Circle(1), PhysicsEngine::Material{});
    body->SetVelocity({2, 0}); world.addBody(body); world.step(0.5f);
    const auto hit = PhysicsEngine::RayCastNearest(world, {-2, 0}, {4, 0});
    const auto points = PhysicsEngine::QueryPoint(world, {1, 0});
    return std::abs(body->position.x-1) < 1e-6f
        && !PhysicsEngine::ExportWorldJson(world).empty()
        && hit && hit->body == body && std::abs(hit->hit.fraction - 1.0 / 3.0) < 1e-6
        && points.size() == 1 && points[0] == body ? 0 : 1;
}

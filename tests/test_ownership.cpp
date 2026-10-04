#include "catch_amalgamated.hpp"
#include "physics/core/world.h"
#include <type_traits>

using namespace PhysicsEngine;

TEST_CASE("Bodies own shapes independently of their source lifetime", "[ownership]") {
    RigidBodyPtr body;
    SECTION("circle") {
        Circle source(2);
        body = std::make_shared<RigidBody>(source, Material{1, 0});
    }
    SECTION("polygon") {
        Polygon source = Polygon::MakeBox(4, 4);
        body = std::make_shared<RigidBody>(source, Material{1, 0});
    }
    World world;
    world.addBody(body);
    body->SetVelocity({1, 0});
    world.step(0.5f);
    REQUIRE(body->GetPosition().x == Catch::Approx(0.5f));
    REQUIRE(body->GetAABB().min.x == Catch::Approx(-1.5f));
    REQUIRE(body->shape->GetArea() > 0);
    STATIC_REQUIRE_FALSE(std::is_copy_constructible_v<RigidBody>);
}

#include "catch_amalgamated.hpp"
#include "../include/physics/core/world.h"
#include "../include/physics/core/rigidbody.h"
#include "../include/physics/core/shape.h"
#include "../include/physics/math/vector2.h"
#include "../include/physics/core/force_generator.h"

using namespace Catch;

class GravityGenerator : public IForceGenerator {
public:
    GravityGenerator(const Vector2& gravity) : gravity(gravity) {}

    void applyForce(RigidBody* body) override {
        body->ApplyForce(gravity * body->GetMass());
    }

private:
    Vector2 gravity;
};

TEST_CASE("World operations are correct", "[World]") {
    World world;
    Circle circle(1.0f);

    SECTION("Add and Remove Body") {
        RigidBody body(&circle, 1.0f, Vector2(0.0f, 0.0f));
        world.addBody(&body);
        REQUIRE(world.getBodies().size() == 1);
        REQUIRE(world.getBodies()[0] == &body);

        world.removeBody(&body);
        REQUIRE(world.getBodies().empty());
    }

    SECTION("Step") {
        RigidBody body(&circle, 1.0f, Vector2(0.0f, 0.0f));
        body.SetVelocity(Vector2(1.0f, 2.0f));
        world.addBody(&body);

        world.step(0.1f);

        REQUIRE(body.GetPosition().x == Approx(0.1f));
        REQUIRE(body.GetPosition().y == Approx(0.2f));
    }

    SECTION("Force") {
        RigidBody body(&circle, 1.0f, Vector2(0.0f, 0.0f));
        world.addBody(&body);

        auto gravity = std::make_unique<GravityGenerator>(Vector2(0.0f, -9.8f));
        world.addForce(&body, std::move(gravity));

        world.step(0.1f);

        REQUIRE(body.GetVelocity().y == Approx(-0.98f));
        REQUIRE(body.GetPosition().y == Approx(-0.098f));
    }
}
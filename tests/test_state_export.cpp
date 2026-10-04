#include "catch_amalgamated.hpp"
#include "physics/physics.h"
#include <limits>
#include <locale>

using namespace PhysicsEngine;
TEST_CASE("World export retains IDs shapes state and statistics", "[export]") {
    World world;
    auto body = std::make_shared<RigidBody>(Circle(0.5f), Material{}, Vector2(1.25f, -2));
    world.addBody(body); world.step(0);
    const auto json = ExportWorldJson(world, 0.5);
    REQUIRE(json.find("\"schemaVersion\":1") != std::string::npos);
    REQUIRE(json.find("\"id\":\""+std::to_string(body->GetId())+"\"") != std::string::npos);
    REQUIRE(json.find("\"position\":[1.25,-2]") != std::string::npos);
    REQUIRE(json.find("\"integratedBodyCount\":1") != std::string::npos);
    REQUIRE(json.find("\"type\":\"circle\"") != std::string::npos);
    REQUIRE(ExportWorldCsv(world).find("1.25,-2") != std::string::npos);
    REQUIRE_THROWS_AS(ExportWorldJson(world, std::numeric_limits<double>::infinity()), std::invalid_argument);
    body->position.x = std::numeric_limits<float>::quiet_NaN();
    REQUIRE_THROWS_AS(ExportWorldJson(world), std::invalid_argument);
    REQUIRE_THROWS_AS(ExportWorldCsv(world), std::invalid_argument);
}

TEST_CASE("Fluid exports include diagnostics and reject nonfinite state", "[export]") {
    std::vector<FluidParticle> particles{FluidParticle(Vector2(1.5f, 2))};
    FluidDiagnostics diagnostics; diagnostics.densityIterations = 7;
    const auto json = ExportFluidJson(particles, diagnostics);
    REQUIRE(json.find("\"densityIterations\":7") != std::string::npos);
    REQUIRE(json.find("\"position\":[1.5,2]") != std::string::npos);
    REQUIRE(ExportFluidCsv(particles).find("1.5,2") != std::string::npos);
    particles[0].density = std::numeric_limits<float>::quiet_NaN();
    REQUIRE_THROWS_AS(ExportFluidJson(particles, diagnostics), std::invalid_argument);
}

TEST_CASE("Exports use decimal dots under a different process locale", "[export]") {
    struct CommaPunctuation : std::numpunct<char> { char do_decimal_point() const override { return ','; } };
    const auto previous = std::locale();
    struct RestoreLocale { std::locale locale; ~RestoreLocale() { std::locale::global(locale); } } restore{previous};
    std::locale::global(std::locale(previous, new CommaPunctuation));
    World world;
    world.addBody(std::make_shared<RigidBody>(Circle(0.5f), Material{}, Vector2(1.25f, 0)));
    REQUIRE(ExportWorldJson(world).find("[1.25,0]") != std::string::npos);
}

TEST_CASE("Public state setters reject invalid values before mutation", "[validation]") {
    RigidBody body(Circle(1), Material{});
    const float nan = std::numeric_limits<float>::quiet_NaN();
    REQUIRE_THROWS_AS(body.SetPosition({nan, 0}), std::invalid_argument);
    REQUIRE_THROWS_AS(body.SetOrientation(nan), std::invalid_argument);
    REQUIRE_THROWS_AS(body.SetVelocity({0, nan}), std::invalid_argument);
    REQUIRE_THROWS_AS(body.ApplyForce({nan, 0}), std::invalid_argument);
    REQUIRE(body.position == Vector2{});
    World world;
    REQUIRE_THROWS_AS(world.addBody(nullptr), std::invalid_argument);
    REQUIRE_THROWS_AS(world.addUniversalForce(nullptr), std::invalid_argument);
}

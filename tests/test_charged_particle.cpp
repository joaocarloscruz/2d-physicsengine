#include "catch_amalgamated.hpp"
#include "physics/physics.h"

#include <cmath>
#include <limits>
#include <stdexcept>

using namespace PhysicsEngine;
namespace {
constexpr double pi = 3.14159265358979323846;
void sameState(const ChargedParticle& a, const ChargedParticle& b, double tolerance = 1e-12) {
    REQUIRE(a.getPosition().x == Catch::Approx(b.getPosition().x).margin(tolerance));
    REQUIRE(a.getPosition().y == Catch::Approx(b.getPosition().y).margin(tolerance));
    REQUIRE(a.getVelocity().x == Catch::Approx(b.getVelocity().x).margin(tolerance));
    REQUIRE(a.getVelocity().y == Catch::Approx(b.getVelocity().y).margin(tolerance));
}
}

TEST_CASE("A charged particle accelerates analytically in a uniform electric field", "[electromagnetic]") {
    ChargedParticle p({1, -2}, {3, 4}, 2, -3);
    p.step(0.4, {{2, -4}, 0});
    REQUIRE(p.getVelocity().x == Catch::Approx(1.8));
    REQUIRE(p.getVelocity().y == Catch::Approx(6.4));
    REQUIRE(p.getPosition().x == Catch::Approx(1.96));
    REQUIRE(p.getPosition().y == Catch::Approx(0.08));
}

TEST_CASE("Cyclotron motion has the correct radius period and charge handedness", "[electromagnetic]") {
    for (const double sign : {-1.0, 1.0}) {
        ChargedParticle p({}, {2, 0}, 3, sign * 6);
        p.step(pi / 8, {{}, 2}); // |omega|=4, a quarter revolution.
        REQUIRE(p.getPosition().x == Catch::Approx(0.5).margin(1e-14));
        REQUIRE(p.getPosition().y == Catch::Approx(-sign * 0.5).margin(1e-14));
        REQUIRE(p.getVelocity().x == Catch::Approx(0).margin(1e-14));
        REQUIRE(p.getVelocity().y == Catch::Approx(-sign * 2));
        p.step(3 * pi / 8, {{}, 2});
        sameState(p, ChargedParticle({}, {2, 0}, 3, sign * 6));
    }
}

TEST_CASE("Magnetic motion preserves kinetic energy over many gyroperiods", "[electromagnetic]") {
    ChargedParticle p({1, 2}, {3, -4}, 7, 2);
    const auto initialEnergy = p.getKineticEnergy();
    for (int i = 0; i < 10000; ++i) p.step(0.01, {{}, -11});
    REQUIRE(p.getKineticEnergy() == Catch::Approx(initialEnergy).epsilon(2e-12));
    const double omega = -22.0 / 7;
    REQUIRE(p.getPosition().x + p.getVelocity().y / omega
        == Catch::Approx(1 - 4 / omega).margin(2e-12));
    REQUIRE(p.getPosition().y - p.getVelocity().x / omega
        == Catch::Approx(2 - 3 / omega).margin(2e-12));
}

TEST_CASE("Crossed fields produce charge-independent electric cross magnetic drift", "[electromagnetic]") {
    const UniformElectromagneticField field{{6, -3}, 2};
    for (const double charge : {-3.0, 2.0}) {
        ChargedParticle p({}, {}, 5, charge);
        const double period = 2 * pi * 5 / std::abs(charge * 2);
        p.step(period, field);
        REQUIRE(p.getPosition().x == Catch::Approx(-1.5 * period));
        REQUIRE(p.getPosition().y == Catch::Approx(-3 * period));
        REQUIRE(p.getVelocity().x == Catch::Approx(0).margin(1e-12));
        REQUIRE(p.getVelocity().y == Catch::Approx(0).margin(1e-12));
    }
    ChargedParticle drifting({1, 2}, {-1.5, -3}, 5, -7);
    drifting.step(0.37, field);
    REQUIRE(drifting.getVelocity().x == Catch::Approx(-1.5));
    REQUIRE(drifting.getVelocity().y == Catch::Approx(-3));
    REQUIRE(drifting.getPosition().x == Catch::Approx(1 - 1.5 * 0.37));
    REQUIRE(drifting.getPosition().y == Catch::Approx(2 - 3 * 0.37));
}

TEST_CASE("Electric work matches kinetic energy change with magnetic deflection", "[electromagnetic]") {
    ChargedParticle p({-1, 2}, {3, -4}, 7, -2);
    const double before = p.getKineticEnergy();
    p.step(0.7, {{2, 5}, -3});
    const double work = -2 * (2 * (p.getPosition().x + 1) + 5 * (p.getPosition().y - 2));
    REQUIRE(p.getKineticEnergy() - before == Catch::Approx(work).margin(1e-12));
}

TEST_CASE("Constant field evolution composes across steps and reverses under time reversal", "[electromagnetic]") {
    const UniformElectromagneticField field{{2, -3}, 4};
    ChargedParticle coarse({-1, 2}, {3, -4}, 7, -2);
    auto fine = coarse;
    coarse.step(1.7, field);
    for (int i = 0; i < 170; ++i) fine.step(0.01, field);
    sameState(coarse, fine);
    ChargedParticle reversed(coarse.getPosition(), {-coarse.getVelocity().x, -coarse.getVelocity().y}, 7, -2);
    reversed.step(1.7, {field.electric, -field.magnetic});
    sameState(reversed, ChargedParticle({-1, 2}, {-3, 4}, 7, -2));
}

TEST_CASE("Weak magnetic fields approach electric acceleration without cancellation", "[electromagnetic]") {
    for (const double magnetic : {0.0, 1e-16, -1e-12, 1e-8}) {
        ChargedParticle p({1, -2}, {3, 4}, 2, -3);
        auto electric = p;
        p.step(0.4, {{2, -4}, magnetic});
        electric.step(0.4, {{2, -4}, 0});
        sameState(p, electric, 4e-8);
    }
    // Compare series and trigonometric branches to independent long-double
    // formulas for a particle initially at rest in a purely X electric field.
    for (const double theta : {0.0099999, 0.01, 0.0100001, -0.01}) {
        ChargedParticle p({}, {}, 1, 1);
        p.step(theta > 0 ? theta : -theta, {{1, 0}, theta > 0 ? 1.0 : -1.0});
        const long double t = std::abs(theta), w = theta > 0 ? 1 : -1;
        REQUIRE(p.getPosition().x == Catch::Approx(double(1 - std::cos(t))).margin(1e-18));
        REQUIRE(p.getPosition().y == Catch::Approx(double(-w * (t - std::sin(t)))).margin(1e-18));
    }
}

TEST_CASE("Neutral charges and zero fields are ballistic at extreme finite scales", "[electromagnetic]") {
    const double maximum = std::numeric_limits<double>::max();
    ChargedParticle neutral({1, 2}, {3, -4}, 1e-300, 0);
    neutral.step(0.5, {{maximum, -maximum}, maximum});
    sameState(neutral, ChargedParticle({2.5, 0}, {3, -4}));
    ChargedParticle fieldFree({}, {1, -1}, 1e-300, maximum);
    fieldFree.step(0.25);
    sameState(fieldFree, ChargedParticle({0.25, -0.25}, {1, -1}));
    ChargedParticle scaled({}, {1, 0}, 1e-300, 1e100);
    scaled.step(1e-200, {{}, 1e-200}); // q/m overflows, theta=1 is finite.
    REQUIRE(scaled.getVelocity().x == Catch::Approx(std::cos(1.0)));
    REQUIRE(scaled.getVelocity().y == Catch::Approx(-std::sin(1.0)));
    REQUIRE(scaled.getPosition().x == Catch::Approx(1e-200 * std::sin(1.0)).epsilon(1e-12));
    ChargedParticle energetic({}, {1e154, 0}, std::numeric_limits<double>::denorm_min(), 0);
    REQUIRE(energetic.getKineticEnergy() == Catch::Approx(0.5e308 * std::numeric_limits<double>::denorm_min()).epsilon(1e-12));
    ChargedParticle hugeVelocity({}, {1.5e308, 1.5e308}, 1e-309, 0);
    REQUIRE(hugeVelocity.getKineticEnergy() == Catch::Approx(2.25e307).epsilon(1e-12));
}

TEST_CASE("Charged-particle validation and overflow preserve state", "[electromagnetic]") {
    const double infinity = std::numeric_limits<double>::infinity();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    REQUIRE_THROWS_AS(ChargedParticle({}, {}, 0, 1), std::invalid_argument);
    REQUIRE_THROWS_AS(ChargedParticle({}, {}, -1, 1), std::invalid_argument);
    REQUIRE_THROWS_AS(ChargedParticle({}, {}, infinity, 1), std::invalid_argument);
    REQUIRE_THROWS_AS(ChargedParticle({}, {}, 1, nan), std::invalid_argument);
    REQUIRE_THROWS_AS(ChargedParticle({nan, 0}), std::invalid_argument);
    ChargedParticle p({1, 2}, {3, 4}, 1, 2);
    const auto before = p;
    REQUIRE_THROWS_AS(p.setState({9, 9}, {infinity, 0}), std::invalid_argument);
    REQUIRE_THROWS_AS(p.step(-1), std::invalid_argument);
    REQUIRE_THROWS_AS(p.step(infinity), std::invalid_argument);
    REQUIRE_THROWS_AS(p.step(0.1, {{nan, 0}, 0}), std::invalid_argument);
    REQUIRE_THROWS_AS(p.step(0, {{}, infinity}), std::invalid_argument);
    REQUIRE_THROWS_AS(p.step(1, {{1e308, 0}, 0}), std::overflow_error);
    REQUIRE_THROWS_AS(p.step(1e308), std::overflow_error);
    sameState(p, before, 0);
    p.step(0, {{1e308, 0}, 1e308});
    sameState(p, before, 0);
}

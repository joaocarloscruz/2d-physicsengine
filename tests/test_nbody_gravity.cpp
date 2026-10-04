#include "catch_amalgamated.hpp"
#include "physics/physics.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

using namespace PhysicsEngine;
namespace {
constexpr double pi = 3.14159265358979323846;
NBodyGravity circularBinary(double h) {
    NBodyGravityConfig config;
    config.maxSubstep = h;
    config.maxSubsteps = 6000;
    NBodyGravity system(config);
    system.addParticle({-0.5, 0}, {0, -std::sqrt(0.5)});
    system.addParticle({0.5, 0}, {0, std::sqrt(0.5)});
    return system;
}
double orbitalError(const NBodyGravity& system, double time) {
    const auto p = system.getParticles()[0].position;
    return std::hypot(p.x + 0.5 * std::cos(std::sqrt(2.0) * time), p.y + 0.5 * std::sin(std::sqrt(2.0) * time));
}
void sameState(const NBodyGravity& a, const NBodyGravity& b) {
    REQUIRE(a.getParticles().size() == b.getParticles().size());
    for (std::size_t i = 0; i < a.getParticles().size(); ++i) {
        REQUIRE(a.getParticles()[i].position.x == b.getParticles()[i].position.x);
        REQUIRE(a.getParticles()[i].position.y == b.getParticles()[i].position.y);
        REQUIRE(a.getParticles()[i].velocity.x == b.getParticles()[i].velocity.x);
        REQUIRE(a.getParticles()[i].velocity.y == b.getParticles()[i].velocity.y);
    }
}
}

TEST_CASE("NBodyGravity pair force attracts with unequal mass accelerations", "[NBodyGravity]") {
    NBodyGravityConfig config;
    config.gravitationalStrength = 1.5;
    config.softening = 3;
    NBodyGravity system(config);
    system.addParticle({0, 0}, {}, 2);
    system.addParticle({4, 0}, {}, 3);
    REQUIRE(system.getDiagnostics().potentialEnergy == Catch::Approx(-1.8).epsilon(0).margin(1e-15));
    system.step(1e-6);
    REQUIRE(system.getParticles()[0].velocity.x / 1e-6 == Catch::Approx(0.144).epsilon(0).margin(1e-12));
    REQUIRE(system.getParticles()[1].velocity.x / 1e-6 == Catch::Approx(-0.096).epsilon(0).margin(1e-12));
    REQUIRE(system.getDiagnostics().momentum.x == Catch::Approx(0).epsilon(0).margin(1e-21));
    REQUIRE(system.getParticles()[0].velocity.y == 0);
    REQUIRE(system.getDiagnostics().lastSubsteps == 1);
    REQUIRE(system.getDiagnostics().lastPairWork == 4); // Initial force, guard, final force, potential.
}

TEST_CASE("NBodyGravity Plummer force is the negative potential gradient", "[NBodyGravity]") {
    NBodyGravityConfig config;
    config.gravitationalStrength = 1.5;
    config.softening = 3;
    NBodyGravity system(config);
    system.addParticle({}, {}, 2);
    system.addParticle({4, 0}, {}, 3);
    constexpr double delta = 1e-5;
    system.setState(0, {delta, 0}, {});
    const double plus = system.getDiagnostics().potentialEnergy;
    system.setState(0, {-delta, 0}, {});
    const double minus = system.getDiagnostics().potentialEnergy;
    const double numericalForce = -(plus - minus) / (2 * delta);
    REQUIRE(numericalForce == Catch::Approx(0.288).epsilon(0).margin(5e-11));
}

TEST_CASE("NBodyGravity circular binary follows orbital phase and period", "[NBodyGravity]") {
    auto system = circularBinary(0.001);
    const double period = 2 * pi / std::sqrt(2.0);
    system.step(period / 4);
    REQUIRE(system.getParticles()[0].position.x == Catch::Approx(0).epsilon(0).margin(2e-6));
    REQUIRE(system.getParticles()[0].position.y == Catch::Approx(-0.5).epsilon(0).margin(1e-6));
    system.step(period / 4);
    REQUIRE(orbitalError(system, period / 2) < 3e-6);
    system.step(period / 2);
    REQUIRE(orbitalError(system, period) < 3e-6);
    REQUIRE(std::hypot(system.getParticles()[0].position.x, system.getParticles()[0].position.y)
        == Catch::Approx(0.5).epsilon(0).margin(1e-6));
}

TEST_CASE("NBodyGravity Verlet refinement converges at second order", "[NBodyGravity]") {
    auto coarse = circularBinary(0.02);
    auto medium = circularBinary(0.01);
    auto fine = circularBinary(0.005);
    coarse.step(0.5);
    medium.step(0.5);
    fine.step(0.5);
    const double e0 = orbitalError(coarse, 0.5), e1 = orbitalError(medium, 0.5), e2 = orbitalError(fine, 0.5);
    REQUIRE(e0 > 3.8 * e1);
    REQUIRE(e1 > 3.8 * e2);
    REQUIRE(e2 < 1e-5);
}

TEST_CASE("NBodyGravity isolated orbit has bounded energy and conserved angular momentum", "[NBodyGravity]") {
    auto system = circularBinary(0.005);
    const auto initial = system.getDiagnostics();
    REQUIRE(initial.totalEnergy == Catch::Approx(-0.5).epsilon(0).margin(2e-16));
    double maximumEnergyError = 0;
    for (int i = 0; i < 4000; ++i) {
        system.step(0.005);
        const auto d = system.getDiagnostics();
        maximumEnergyError = std::max(maximumEnergyError, std::abs(d.totalEnergy - initial.totalEnergy));
        REQUIRE(d.angularMomentum == Catch::Approx(initial.angularMomentum).epsilon(0).margin(5e-13));
        REQUIRE(d.momentum.x == Catch::Approx(0).epsilon(0).margin(1e-13));
        REQUIRE(d.momentum.y == Catch::Approx(0).epsilon(0).margin(1e-13));
    }
    REQUIRE(maximumEnergyError < 2e-8);
}

TEST_CASE("NBodyGravity unequal binary center of mass drifts freely", "[NBodyGravity]") {
    NBodyGravityConfig config;
    config.maxSubstep = 0.005;
    NBodyGravity system(config);
    system.addParticle({-0.6, 0}, {0.2, -0.3 - 0.6 * std::sqrt(5.0)}, 2);
    system.addParticle({0.4, 0}, {0.2, -0.3 + 0.4 * std::sqrt(5.0)}, 3);
    const auto before = system.getDiagnostics();
    auto repeat = system;
    for (int i = 0; i < 1000; ++i) {
        system.step(0.005);
        repeat.step(0.005);
    }
    const auto after = system.getDiagnostics();
    REQUIRE(after.totalMass == 5);
    REQUIRE(after.centerOfMass.x == Catch::Approx(1).epsilon(0).margin(2e-12));
    REQUIRE(after.centerOfMass.y == Catch::Approx(-1.5).epsilon(0).margin(2e-12));
    REQUIRE(after.momentum.x == Catch::Approx(before.momentum.x).epsilon(0).margin(2e-12));
    REQUIRE(after.momentum.y == Catch::Approx(before.momentum.y).epsilon(0).margin(2e-12));
    sameState(system, repeat);
}

TEST_CASE("NBodyGravity zero strength permits coincident free motion", "[NBodyGravity]") {
    NBodyGravityConfig config;
    config.gravitationalStrength = 0;
    config.maxPairWork = 1;
    NBodyGravity system(config);
    system.addParticle({}, {1, 0}, 2);
    system.addParticle({}, {-1, 0}, 3);
    system.applyImpulse(0, {2, 4});
    system.step(0.1);
    REQUIRE(system.getParticles()[0].position.x == Catch::Approx(0.2).epsilon(0).margin(1e-15));
    REQUIRE(system.getParticles()[0].position.y == Catch::Approx(0.2).epsilon(0).margin(1e-15));
    REQUIRE(system.getParticles()[1].position.x == Catch::Approx(-0.1).epsilon(0).margin(1e-15));
    REQUIRE(system.getDiagnostics().potentialEnergy == 0);
    REQUIRE(system.getDiagnostics().lastPairWork == 0);
}

TEST_CASE("NBodyGravity distinguishes softened coincidence from singular unsoftened state", "[NBodyGravity]") {
    NBodyGravity singular;
    singular.addParticle({});
    singular.addParticle({});
    const auto before = singular;
    REQUIRE_THROWS_AS(singular.step(0.01), std::runtime_error);
    REQUIRE_THROWS_AS(singular.getDiagnostics(), std::runtime_error);
    sameState(singular, before);
    auto config = singular.getConfig();
    config.softening = 1;
    singular.setConfig(config);
    REQUIRE(singular.getDiagnostics().potentialEnergy == -1);
    singular.step(0.01);
    sameState(singular, before);
}

TEST_CASE("NBodyGravity unresolved singular crossings reject without partial advancement", "[NBodyGravity]") {
    NBodyGravity system;
    system.addParticle({-0.5, 0}, {50, 0});
    system.addParticle({0.5, 0}, {-50, 0});
    const auto before = system;
    REQUIRE_THROWS_AS(system.step(0.02), std::runtime_error);
    sameState(system, before);
    REQUIRE(system.getDiagnostics().lastSubsteps == 0);
    auto config = system.getConfig();
    config.softening = 0.2;
    system.setConfig(config);
    system.step(0.02);
    REQUIRE(system.getParticles()[0].position.x > system.getParticles()[1].position.x);
    REQUIRE(std::isfinite(system.getDiagnostics().totalEnergy));
    REQUIRE(system.getDiagnostics().lastSubsteps > 1);
}

TEST_CASE("NBodyGravity total pair work includes trials and diagnostic work", "[NBodyGravity]") {
    auto system = circularBinary(0.01);
    auto config = system.getConfig();
    config.maxPairWork = 4;
    system.setConfig(config);
    system.step(0.01);
    REQUIRE(system.getDiagnostics().lastPairWork == 4);
    const auto before = system;
    REQUIRE_THROWS_AS(system.step(0.02), std::runtime_error);
    sameState(system, before);
    REQUIRE(system.getDiagnostics().lastPairWork == 4);
    REQUIRE(system.getDiagnostics().lastSubsteps == 1);
    config.maxPairWork = 3; // Even a one-step diagnostic pass must fit.
    system.setConfig(config);
    REQUIRE_THROWS_AS(system.step(0.01), std::runtime_error);
    sameState(system, before);
    config.maxPairWork = 1;
    system.setConfig(config);
    system.setState(0, {-0.5, 0}, {50, 0});
    system.setState(1, {0.5, 0}, {-50, 0});
    const auto fast = system;
    REQUIRE_THROWS_AS(system.step(0.01), std::runtime_error);
    sameState(system, fast);
}

TEST_CASE("NBodyGravity substep caps preserve decimal partitions and reject large work", "[NBodyGravity]") {
    NBodyGravityConfig config;
    config.gravitationalStrength = 0;
    config.maxSubstep = 0.01;
    config.maxSubsteps = 7;
    NBodyGravity system(config);
    system.addParticle({}, {1, 0});
    system.step(0.07);
    REQUIRE(system.getDiagnostics().lastSubsteps == 7);
    const auto before = system;
    // The quotient may look like an integer within roundoff, but a real excess
    // must not be accepted with h > maxSubstep.
    REQUIRE_THROWS_AS(system.step(std::nextafter(0.07, 1.0)), std::runtime_error);
    sameState(system, before);
    REQUIRE_THROWS_AS(system.step(0.07001), std::runtime_error);
    sameState(system, before);
    REQUIRE(system.getDiagnostics().lastSubsteps == 7);
    system.step(0);
    REQUIRE(system.getDiagnostics().lastSubsteps == 0);
    REQUIRE(system.getDiagnostics().lastPairWork == 0);
    NBodyGravity empty;
    empty.step(1);
    REQUIRE(empty.getDiagnostics().totalMass == 0);
}

TEST_CASE("NBodyGravity frequency partitions enforce the representable duration bound", "[NBodyGravity]") {
    NBodyGravityConfig config;
    config.softening = 1;
    config.frequencySafety = 0.1;
    config.maxSubstep = 1;
    config.maxSubsteps = 7;
    NBodyGravity system(config);
    system.addParticle({});
    system.addParticle({});
    // At softened coincidence the tidal bound is exactly four and the state
    // does not move, so seven equal durations above the downward-rounded .05
    // limit must reject even though the quotient is within integer roundoff.
    const double limit = std::nextafter(0.05, 0.0);
    const double duration = std::nextafter(7 * limit, 1.0);
    const auto before = system;
    REQUIRE_THROWS_AS(system.step(duration), std::runtime_error);
    sameState(system, before);
    config.maxSubsteps = 8;
    system.setConfig(config);
    REQUIRE_NOTHROW(system.step(duration));
    REQUIRE(system.getDiagnostics().lastSubsteps == 8);
}

TEST_CASE("NBodyGravity validates finite arguments and topology budgets", "[NBodyGravity]") {
    const double inf = std::numeric_limits<double>::infinity();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    NBodyGravity system;
    REQUIRE_THROWS_AS(system.addParticle({}, {}, 0), std::invalid_argument);
    REQUIRE_THROWS_AS(system.addParticle({}, {}, -1), std::invalid_argument);
    REQUIRE_THROWS_AS(system.addParticle({}, {}, inf), std::invalid_argument);
    REQUIRE_THROWS_AS(system.addParticle({nan, 0}), std::invalid_argument);
    REQUIRE_THROWS_AS(system.addParticle({}, {0, inf}), std::invalid_argument);
    system.addParticle({});
    REQUIRE_THROWS_AS(system.setState(0, {2, 3}, {nan, 0}), std::invalid_argument);
    REQUIRE(system.getParticles()[0].position.x == 0);
    REQUIRE_THROWS_AS(system.setState(1, {}, {}), std::out_of_range);
    REQUIRE_THROWS_AS(system.applyImpulse(1, {}), std::out_of_range);
    REQUIRE_THROWS_AS(system.applyImpulse(0, {inf, 0}), std::invalid_argument);
    REQUIRE_THROWS_AS(system.step(-1), std::invalid_argument);
    REQUIRE_THROWS_AS(system.step(inf), std::invalid_argument);
    REQUIRE_THROWS_AS(system.step(nan), std::invalid_argument);
    auto config = system.getConfig();
    config.maxParticles = 1;
    system.setConfig(config);
    REQUIRE_THROWS_AS(system.addParticle({}), std::length_error);
    config.gravitationalStrength = -1;
    REQUIRE_THROWS_AS(system.setConfig(config), std::invalid_argument);
    config = system.getConfig(); config.softening = -1;
    REQUIRE_THROWS_AS(NBodyGravity(config), std::invalid_argument);
    config = system.getConfig(); config.maxPairWork = 0;
    REQUIRE_THROWS_AS(NBodyGravity(config), std::invalid_argument);
    config = system.getConfig(); config.maxSubsteps = 0;
    REQUIRE_THROWS_AS(NBodyGravity(config), std::invalid_argument);
    config = system.getConfig(); config.maxSubstep = 0;
    REQUIRE_THROWS_AS(NBodyGravity(config), std::invalid_argument);
    config = system.getConfig(); config.frequencySafety = 1.01;
    REQUIRE_THROWS_AS(NBodyGravity(config), std::invalid_argument);
}

TEST_CASE("NBodyGravity arithmetic and impulse overflow are transactional", "[NBodyGravity]") {
    SECTION("Impulse") {
        NBodyGravity system;
        system.addParticle({}, {}, 1e-308);
        REQUIRE_THROWS_AS(system.applyImpulse(0, {1, 10}), std::overflow_error);
        REQUIRE(system.getParticles()[0].velocity.x == 0);
        REQUIRE(system.getParticles()[0].velocity.y == 0);
    }
    SECTION("Staged ballistic drift") {
        NBodyGravityConfig config;
        config.gravitationalStrength = 0;
        NBodyGravity system(config);
        system.addParticle({}, {1, 0});
        system.addParticle({}, {std::numeric_limits<double>::max(), 0});
        const auto before = system;
        REQUIRE_THROWS_AS(system.step(2), std::overflow_error);
        sameState(system, before);
    }
    SECTION("Tidal rate") {
        NBodyGravity system;
        system.addParticle({});
        system.addParticle({1e-200, 0});
        const auto before = system;
        REQUIRE_THROWS_AS(system.step(0.01), std::overflow_error);
        sameState(system, before);
    }
    SECTION("Aggregate diagnostics before publication") {
        NBodyGravityConfig config;
        config.gravitationalStrength = 0;
        NBodyGravity system(config);
        system.addParticle({}, {}, 1e308);
        system.addParticle({1, 0}, {}, 1e308);
        const auto before = system;
        REQUIRE_THROWS_AS(system.step(0.01), std::overflow_error);
        sameState(system, before);
    }
}

TEST_CASE("NBodyGravity preserves representable tiny-mass acceleration and aggregate energies", "[NBodyGravity]") {
    const double tiny = std::numeric_limits<double>::denorm_min();
    NBodyGravityConfig config;
    config.maxSubstep = 1;
    NBodyGravity small(config);
    small.addParticle({}, {}, tiny);
    small.addParticle({1e-100, 0}, {}, tiny);
    small.step(1);
    const double acceleration = (tiny / 1e-100) / 1e-100;
    REQUIRE(small.getParticles()[0].velocity.x == Catch::Approx(acceleration).epsilon(2e-15));
    REQUIRE(small.getParticles()[1].velocity.x == Catch::Approx(-acceleration).epsilon(2e-15));
    config.gravitationalStrength = 0;
    NBodyGravity energy(config);
    energy.addParticle({}, {1, 0}, tiny);
    energy.addParticle({}, {1, 0}, tiny);
    REQUIRE(energy.getDiagnostics().kineticEnergy == tiny);
    config.gravitationalStrength = 1;
    config.softening = 1;
    NBodyGravity potential(config);
    for (int i = 0; i < 3; ++i) potential.addParticle({}, {}, 1e-162);
    REQUIRE(potential.getDiagnostics().potentialEnergy == -tiny);
}

#include "catch_amalgamated.hpp"
#include "physics/physics.h"

#include <cmath>
#include <limits>
#include <stdexcept>

using namespace PhysicsEngine;
namespace {
double energy(const SoftBody& body) {
    const auto d = body.getDiagnostics();
    return d.kineticEnergy + d.elasticEnergy;
}
SoftBody oscillator(double h, double damping = 0.0) {
    SoftBodyConfig c;
    c.maxSubstep = h;
    SoftBody body(c);
    body.addParticle({-0.6f, 0.0f}, {}, 2.0);
    body.addParticle({0.6f, 0.0f}, {}, 3.0);
    body.addSpring(0, 1, 1.0, 30.0, damping);
    return body;
}
double extension(const SoftBody& body) {
    return double(body.getParticles()[1].position.x) - body.getParticles()[0].position.x - 1.0;
}
}

TEST_CASE("SoftBody applies the Hooke force with each particle mass", "[SoftBody]") {
    SoftBody body;
    body.addParticle({0, 0}, {}, 2.0);
    body.addParticle({3, 0}, {}, 1.0);
    body.addSpring(0, 1, 1.0, 4.0);
    body.step(0.0001);
    REQUIRE(body.getParticles()[0].velocity.x == Catch::Approx(0.0004).margin(1e-10));
    REQUIRE(body.getParticles()[1].velocity.x == Catch::Approx(-0.0008).margin(1e-10));
    REQUIRE(body.getParticles()[0].position.x == Catch::Approx(2e-8).margin(1e-14));
    REQUIRE(body.getDiagnostics().lastSubsteps == 1);
}

TEST_CASE("SoftBody two-mass oscillator matches analytic phase and period", "[SoftBody]") {
    auto body = oscillator(0.001);
    const double amplitude = extension(body);
    const double omega = std::sqrt(30.0 * (1.0 / 2.0 + 1.0 / 3.0));
    const double period = 2.0 * std::acos(-1.0) / omega;
    body.step(period / 4.0);
    REQUIRE(extension(body) == Catch::Approx(0.0).margin(3e-6));
    body.step(period / 4.0);
    REQUIRE(extension(body) == Catch::Approx(-amplitude).margin(3e-6));
    body.step(period / 2.0);
    REQUIRE(extension(body) == Catch::Approx(amplitude).margin(3e-6));
    REQUIRE(energy(body) == Catch::Approx(0.5 * 30.0 * amplitude * amplitude).epsilon(1e-4));
}

TEST_CASE("SoftBody refinement converges at second order", "[SoftBody]") {
    auto coarse = oscillator(0.02);
    auto medium = oscillator(0.01);
    auto fine = oscillator(0.005);
    const double exact = extension(coarse) * std::cos(5.0 * 0.3);
    coarse.step(0.3);
    medium.step(0.3);
    fine.step(0.3);
    const double e0 = std::abs(extension(coarse) - exact);
    const double e1 = std::abs(extension(medium) - exact);
    const double e2 = std::abs(extension(fine) - exact);
    REQUIRE(e0 > 3.5 * e1);
    REQUIRE(e1 > 3.5 * e2);
    REQUIRE(e2 < 1e-5);
}

TEST_CASE("SoftBody preserves free system linear and angular momentum", "[SoftBody]") {
    SoftBody body;
    body.addParticle({-1, 0}, {0.5f, 1}, 2);
    body.addParticle({1, 0}, {-0.3f, -0.2f}, 3);
    body.addParticle({0, 1.5f}, {0.2f, -0.4f}, 4);
    body.addSpring(0, 1, 2, 15, 3);
    body.addSpring(1, 2, 1.8, 20, 5);
    body.addSpring(2, 0, 1.8, 25, 2);
    const auto before = body.getDiagnostics();
    const auto angular = [](const SoftBody& b) {
        double result = 0;
        for (const auto& p : b.getParticles())
            result += p.mass * (double(p.position.x) * p.velocity.y - double(p.position.y) * p.velocity.x);
        return result;
    };
    const double angularBefore = angular(body);
    for (int i = 0; i < 500; ++i) body.step(0.01);
    const auto after = body.getDiagnostics();
    REQUIRE(after.momentumX == Catch::Approx(before.momentumX).margin(1e-5));
    REQUIRE(after.momentumY == Catch::Approx(before.momentumY).margin(1e-5));
    REQUIRE(angular(body) == Catch::Approx(angularBefore).margin(2e-5));
}

TEST_CASE("SoftBody fixed anchors have finite mass and remain pinned", "[SoftBody]") {
    SoftBody body;
    body.addParticle({0, 2}, {}, 3, true);
    body.addParticle({1, 2}, {}, 2);
    body.addSpring(0, 1, 1, 50, 5);
    body.setUniformAcceleration({0, -10});
    body.applyImpulse(0, {100, 100});
    for (int i = 0; i < 1000; ++i) body.step(0.01);
    REQUIRE(body.getParticles()[0].position == Vector2(0, 2));
    REQUIRE(body.getParticles()[0].velocity == Vector2());
    REQUIRE(body.getDiagnostics().totalMass == 5.0);
    body.setFixed(1, true);
    REQUIRE(body.getParticles()[1].velocity == Vector2());
    body.setFixed(1, false);
    body.applyImpulse(1, {4, 0});
    REQUIRE(body.getParticles()[1].velocity.x == 2.0f);
}

TEST_CASE("SoftBody pair dashpot has exact exponential decay", "[SoftBody]") {
    SoftBodyConfig c;
    c.maxSubstep = 0.1;
    SoftBody body(c);
    body.addParticle({0, 0}, {-1, 0}, 2);
    body.addParticle({10, 0}, {1, 0}, 3);
    body.addSpring(0, 1, 10, 0, 4);
    const double initialEnergy = energy(body);
    body.step(0.1);
    const double relative = 2.0 * std::exp(-4.0 * (0.5 + 1.0 / 3.0) * 0.1);
    REQUIRE(double(body.getParticles()[1].velocity.x) - body.getParticles()[0].velocity.x ==
            Catch::Approx(relative).margin(1e-7));
    REQUIRE(body.getDiagnostics().momentumX == Catch::Approx(1.0).margin(2e-7));
    REQUIRE(energy(body) < initialEnergy);
}

TEST_CASE("SoftBody damping dissipates oscillation energy without center drag", "[SoftBody]") {
    auto body = oscillator(0.002, 4);
    const double initialEnergy = energy(body);
    body.step(4.0);
    REQUIRE(energy(body) < 0.001 * initialEnergy);
    REQUIRE(body.getDiagnostics().momentumX == Catch::Approx(0.0).margin(1e-6));
}

TEST_CASE("SoftBody unlinked particles integrate acceleration and impulses", "[SoftBody]") {
    SoftBody body;
    body.addParticle({0, 0}, {1, 0}, 2);
    body.applyImpulse(0, {2, 0});
    body.setUniformAcceleration({0, -10});
    body.step(0.5);
    const auto& p = body.getParticles()[0];
    REQUIRE(p.position.x == Catch::Approx(1.0).margin(1e-6));
    REQUIRE(p.position.y == Catch::Approx(-1.25).margin(1e-6));
    REQUIRE(p.velocity.y == Catch::Approx(-5.0).margin(1e-6));
}

TEST_CASE("SoftBody stiffness and compressed geometry reduce the timestep", "[SoftBody]") {
    SoftBodyConfig c;
    c.maxSubstep = 1.0;
    SoftBody body(c);
    body.addParticle({0, 0});
    body.addParticle({1, 0});
    body.addSpring(0, 1, 1, 1000);
    body.step(0.1);
    REQUIRE(body.getDiagnostics().lastSubsteps >= 18);
    body.setParticleState(1, {0.01f, 0});
    body.step(0.001);
    REQUIRE(body.getDiagnostics().lastSubsteps >= 2);
    REQUIRE(std::isfinite(energy(body)));
}

TEST_CASE("SoftBody positive-rest collapse and exhausted budgets roll back", "[SoftBody]") {
    SoftBodyConfig c;
    c.maxSubstep = 0.01;
    c.maxSubsteps = 10;
    SoftBody body(c);
    body.addParticle({0, 0});
    body.addParticle({1, 0});
    body.addSpring(0, 1, 1, 10);
    body.step(0.1); // Exact budget remains usable despite roundoff.
    REQUIRE(body.getDiagnostics().lastSubsteps == 10);
    REQUIRE_THROWS_AS(body.step(0.101), std::runtime_error);
    REQUIRE(body.getParticles()[1].position == Vector2(1, 0));
    REQUIRE(body.getDiagnostics().lastSubsteps == 10);
    body.setParticleState(1, {0, 0});
    REQUIRE_THROWS_AS(body.step(0.01), std::runtime_error);
    REQUIRE(body.getParticles()[0].position == Vector2());
    REQUIRE(body.getParticles()[1].position == Vector2());
    body.setParticleState(1, {1, 0});
    c.maxSubsteps = 1;
    body.setConfig(c);
    body.setParticleState(1, {0.00001f, 0});
    REQUIRE_THROWS_AS(body.step(0.01), std::runtime_error);
    REQUIRE(body.getParticles()[1].position.x == 0.00001f);
}

TEST_CASE("SoftBody zero-rest collapsed springs are well defined", "[SoftBody]") {
    SoftBody body;
    body.addParticle({0, 0});
    body.addParticle({0, 0});
    body.addSpring(0, 1, 0, 10, 5);
    body.step(0.1);
    REQUIRE(energy(body) == 0.0);
    REQUIRE(body.getDiagnostics().maxStrain == 0.0);
}

TEST_CASE("SoftBody large motion cannot jump across a spring collapse", "[SoftBody]") {
    SoftBody body;
    body.addParticle({0, 0}, {}, 1, true);
    body.addParticle({1, 0}, {-1000, 0});
    body.addSpring(0, 1, 1, 10);
    REQUIRE_THROWS_AS(body.step(0.01), std::runtime_error);
    REQUIRE(body.getParticles()[1].position == Vector2(1, 0));
    REQUIRE(body.getParticles()[1].velocity == Vector2(-1000, 0));
    body.setParticleState(1, {1, 0}, {-10, 0});
    body.step(0.02);
    REQUIRE(body.getParticles()[1].position.x > 0.79f);
    REQUIRE(body.getParticles()[1].position.x < 0.81f);
}

TEST_CASE("SoftBody validates finite inputs topology and collection budgets", "[SoftBody]") {
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    SoftBody body;
    REQUIRE_THROWS_AS(body.addParticle({nan, 0}), std::invalid_argument);
    REQUIRE_THROWS_AS(body.addParticle({}, {nan, 0}), std::invalid_argument);
    REQUIRE_THROWS_AS(body.addParticle({}, {}, 0), std::invalid_argument);
    REQUIRE_THROWS_AS(body.addParticle({}, {}, inf), std::invalid_argument);
    REQUIRE_THROWS_AS(body.addParticle({}, {}, std::numeric_limits<double>::denorm_min()), std::invalid_argument);
    REQUIRE_THROWS_AS(body.addParticle({}, {1, 0}, 1, true), std::invalid_argument);
    SoftBody coincident;
    coincident.addParticle({});
    coincident.addParticle({});
    REQUIRE_THROWS_AS(coincident.addSpring(0, 1, 1, 1), std::invalid_argument);
    body.addParticle({0, 0}, {}, 1, true);
    body.addParticle({1, 0});
    REQUIRE_THROWS_AS(body.addSpring(0, 2, 1, 10), std::out_of_range);
    REQUIRE_THROWS_AS(body.addSpring(0, 0, 1, 10), std::invalid_argument);
    REQUIRE_THROWS_AS(body.addSpring(0, 1, -1, 10), std::invalid_argument);
    REQUIRE_THROWS_AS(body.addSpring(0, 1, 1, -1), std::invalid_argument);
    REQUIRE_THROWS_AS(body.addSpring(0, 1, 1, 10, -1), std::invalid_argument);
    REQUIRE_THROWS_AS(body.addSpring(0, 1, inf, 10), std::invalid_argument);
    REQUIRE_THROWS_AS(body.addSpring(0, 1, 1, inf), std::invalid_argument);
    REQUIRE_THROWS_AS(body.addSpring(0, 1, 1, 10, inf), std::invalid_argument);
    body.addSpring(0, 1, 1, 10);
    REQUIRE_THROWS_AS(body.addSpring(1, 0, 1, 10), std::invalid_argument);
    REQUIRE_THROWS_AS(body.setParticleState(0, {nan, 0}), std::invalid_argument);
    REQUIRE_THROWS_AS(body.setParticleState(0, {}, {1, 0}), std::invalid_argument);
    REQUIRE_THROWS_AS(body.applyImpulse(0, {nan, 0}), std::invalid_argument);
    REQUIRE_THROWS_AS(body.applyImpulse(10, {}), std::out_of_range);
    REQUIRE_THROWS_AS(body.setFixed(10, true), std::out_of_range);
    REQUIRE_THROWS_AS(body.setUniformAcceleration({nan, 0}), std::invalid_argument);
    REQUIRE_THROWS_AS(body.step(-1), std::invalid_argument);
    REQUIRE_THROWS_AS(body.step(inf), std::invalid_argument);
    auto c = body.getConfig();
    c.maxParticles = 2;
    c.maxSprings = 1;
    body.setConfig(c);
    REQUIRE_THROWS_AS(body.addParticle({2, 0}), std::length_error);
    c.maxParticles = 1;
    REQUIRE_THROWS_AS(body.setConfig(c), std::length_error);
    c = body.getConfig();
    c.maxParticles = 3;
    body.setConfig(c);
    body.addParticle({2, 0});
    REQUIRE_THROWS_AS(body.addSpring(1, 2, 1, 10), std::length_error);
    c = body.getConfig();
    c.maxSubsteps = 0;
    REQUIRE_THROWS_AS(SoftBody(c), std::invalid_argument);
    c = body.getConfig();
    c.maxSubstep = 0;
    REQUIRE_THROWS_AS(SoftBody(c), std::invalid_argument);
    c = body.getConfig();
    c.stabilityFactor = 1.01;
    REQUIRE_THROWS_AS(SoftBody(c), std::invalid_argument);
}

TEST_CASE("SoftBody overflow rejection does not partly commit state", "[SoftBody]") {
    SoftBody body;
    body.addParticle({0, 0}, {1, 0});
    body.addParticle({0, 0}, {std::numeric_limits<float>::max(), 0});
    REQUIRE_THROWS_AS(body.step(2.0), std::runtime_error);
    REQUIRE(body.getParticles()[0].position == Vector2());
    REQUIRE(body.getParticles()[1].position == Vector2());
    REQUIRE(body.getDiagnostics().lastSubsteps == 0);
}

TEST_CASE("SoftBody rope and cloth topology stay finite deterministically", "[SoftBody]") {
    SoftBody body;
    constexpr int side = 5;
    for (int y = 0; y < side; ++y)
        for (int x = 0; x < side; ++x)
            body.addParticle({float(x), float(y)}, {}, 1, y == side - 1);
    for (int y = 0; y < side; ++y) {
        for (int x = 0; x < side; ++x) {
            const std::size_t i = y * side + x;
            if (x + 1 < side) body.addSpring(i, i + 1, 1, 300, 4);
            if (y + 1 < side) body.addSpring(i, i + side, 1, 300, 4);
            if (x + 1 < side && y + 1 < side) {
                body.addSpring(i, i + side + 1, std::sqrt(2.0), 100, 2);
                body.addSpring(i + 1, i + side, std::sqrt(2.0), 100, 2);
            }
        }
    }
    body.setUniformAcceleration({0, -9.81f});
    auto repeat = body;
    for (int i = 0; i < 2000; ++i) {
        body.step(1.0 / 120.0);
        repeat.step(1.0 / 120.0);
    }
    REQUIRE(std::isfinite(energy(body)));
    REQUIRE(body.getDiagnostics().maxStrain < 0.5);
    for (std::size_t i = 0; i < body.getParticles().size(); ++i) {
        REQUIRE(body.getParticles()[i].position == repeat.getParticles()[i].position);
        REQUIRE(body.getParticles()[i].velocity == repeat.getParticles()[i].velocity);
    }
    body.step(0);
    REQUIRE(body.getDiagnostics().lastSubsteps == 0);
    SoftBody empty;
    empty.step(1);
    REQUIRE(empty.getDiagnostics().lastSubsteps == 0);
}

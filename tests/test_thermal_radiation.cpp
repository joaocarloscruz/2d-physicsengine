#include "catch_amalgamated.hpp"
#include "physics/physics.h"
#include <cmath>
#include <limits>

using namespace PhysicsEngine;
namespace {
ThermalNetwork Radiator(double h) {
    ThermalNetworkConfig c;
    c.maxSubstep = h;
    ThermalNetwork n(c);
    n.addNode(4, 2);
    n.addNode(0, 1, true);
    n.addRadiationLink(0, 1, .25);
    return n;
}
void SameState(const ThermalNetwork &a, const ThermalNetwork &b) {
    REQUIRE(a.getNodes().size() == b.getNodes().size());
    for (std::size_t i = 0; i < a.getNodes().size(); ++i) {
        REQUIRE(a.getNodes()[i].temperature == b.getNodes()[i].temperature);
        REQUIRE(a.getNodes()[i].externalPower == b.getNodes()[i].externalPower);
        REQUIRE(a.getNodes()[i].reservoirHeat == b.getNodes()[i].reservoirHeat);
    }
    const auto x = a.getDiagnostics(), y = b.getDiagnostics();
    REQUIRE(x.totalEnergy == y.totalEnergy);
    REQUIRE(x.totalExternalEnergy == y.totalExternalEnergy);
    REQUIRE(x.totalReservoirHeat == y.totalReservoirHeat);
    REQUIRE(x.lastExternalEnergy == y.lastExternalEnergy);
    REQUIRE(x.lastReservoirHeat == y.lastReservoirHeat);
    REQUIRE(x.lastSubsteps == y.lastSubsteps);
    REQUIRE(x.lastRadiativeVisits == y.lastRadiativeVisits);
}
} // namespace

TEST_CASE("Radiative cooling follows the Stefan Boltzmann analytic solution at "
          "first order",
          "[Thermal][radiation]") {
    // C dT/dt=-kappa*T^4 => T=T0/(1+3*kappa*T0^3*t/C)^(1/3).
    const double exact = 4 / std::cbrt(7.0);
    double errors[3];
    for (int k = 0; k < 3; ++k) {
        auto n = Radiator(std::ldexp(1.0, -9 - k));
        n.step(.25);
        const double temperature = n.getNodes()[0].temperature;
        errors[k] = std::abs(temperature - exact);
        REQUIRE(temperature < exact); // Forward Euler overcools a decreasing convex solution.
        REQUIRE(n.getNodes()[1].temperature == 0);
        const auto d = n.getDiagnostics();
        REQUIRE(d.totalEnergy - 8 == Catch::Approx(d.totalReservoirHeat).margin(2e-14));
        REQUIRE(d.lastRadiativeVisits == 14 * d.lastSubsteps);
        REQUIRE(d.totalExternalEnergy == 0);
    }
    REQUIRE(errors[0] / errors[1] > 1.95);
    REQUIRE(errors[0] / errors[1] < 2.1);
    REQUIRE(errors[1] / errors[2] > 1.95);
    REQUIRE(errors[1] / errors[2] < 2.1);
    REQUIRE(errors[2] < .002);
}

TEST_CASE("Radiative pair transfer is reciprocal and agrees with an "
          "independent fourth power",
          "[Thermal][radiation]") {
    ThermalNetworkConfig c;
    c.maxSubstep = 1;
    ThermalNetwork n(c);
    n.addNode(2, 1);
    n.addNode(1, 2);
    n.addRadiationLink(0, 1, 1.0 / 16);
    const double q = .125 / 16 * (std::pow(1.0, 4) - std::pow(2.0, 4));
    n.step(.125);
    REQUIRE(n.getNodes()[0].temperature == 2 + q);
    REQUIRE(n.getNodes()[1].temperature == 1 - q / 2);
    REQUIRE(n.getDiagnostics().totalEnergy == 4);
    REQUIRE(n.getDiagnostics().lastSubsteps == 1);
    REQUIRE(n.getDiagnostics().lastRadiativeVisits == 14);
}

TEST_CASE("Conduction radiation powers and reservoirs share a closed thermal ledger",
          "[Thermal][radiation]") {
    ThermalNetworkConfig c;
    c.maxSubstep = 1;
    ThermalNetwork n(c);
    n.addNode(1, 2);
    n.addNode(2, 3, true);
    n.addLink(0, 1, .25);
    n.addRadiationLink(0, 1, 1.0 / 16);
    n.applyPower(0, .5);
    n.applyPower(1, .25);
    const double q = .125 * (.25 + 15.0 / 16);
    n.step(.125);
    REQUIRE(n.getNodes()[0].temperature == 1 + (q + .0625) / 2);
    REQUIRE(n.getNodes()[1].temperature == 2);
    REQUIRE(n.getNodes()[1].reservoirHeat == q - .03125);
    auto d = n.getDiagnostics();
    REQUIRE(d.lastExternalEnergy == .09375);
    REQUIRE(d.lastReservoirHeat == q - .03125);
    REQUIRE(d.totalEnergy - 8 == d.totalExternalEnergy + d.totalReservoirHeat);
    REQUIRE(d.lastRadiativeVisits == 16);
    for (int i = 0; i < 100; ++i) {
        n.applyPower(0, .5);
        n.applyPower(1, .25);
        n.step(.125);
    }
    d = n.getDiagnostics();
    REQUIRE(d.totalEnergy - 8 ==
            Catch::Approx(d.totalExternalEnergy + d.totalReservoirHeat).margin(5e-13));
    REQUIRE(n.getNodes()[0].externalPower == 0);
    REQUIRE(n.getNodes()[1].externalPower == 0);
}

TEST_CASE("Adaptive radiative stepping preserves the graph maximum principle "
          "and energy",
          "[Thermal][radiation]") {
    for (bool fixed : {false, true}) {
        ThermalNetworkConfig c;
        c.safetyFactor = 1;
        c.maxSubstep = 1;
        ThermalNetwork n(c);
        n.addNode(0, 1);
        for (std::size_t j = 1; j <= 7; ++j) {
            n.addNode(1 + .5 * j, 1 + .25 * j, fixed && j == 7);
            n.addRadiationLink(0, j, .1 * j);
            n.addLink(0, j, .2 * j);
        }
        const double initial = n.getDiagnostics().totalEnergy;
        auto replay = n;
        for (int i = 0; i < 20; ++i) {
            n.step(.125);
            replay.step(.125);
            const auto d = n.getDiagnostics();
            REQUIRE(d.minimumTemperature >= 0);
            REQUIRE(d.maximumTemperature <= 4.5);
            REQUIRE(d.totalEnergy - initial == Catch::Approx(d.totalReservoirHeat).margin(3e-12));
        }
        SameState(n, replay);
    }
}

TEST_CASE("Radiative equilibrium is stationary and fixed nodes exchange "
          "reciprocal heat",
          "[Thermal][radiation]") {
    ThermalNetwork n;
    n.addNode(3, 1);
    n.addNode(3, 2, true);
    n.addRadiationLink(0, 1, 2);
    n.step(.1);
    REQUIRE(n.getNodes()[0].temperature == 3);
    REQUIRE(n.getNodes()[1].reservoirHeat == 0);
    n.setFixed(0, true);
    n.setTemperature(0, 2);
    n.applyPower(1, 4);
    n.step(.125);
    REQUIRE(n.getNodes()[0].temperature == 2);
    REQUIRE(n.getNodes()[1].temperature == 3);
    REQUIRE(n.getNodes()[0].reservoirHeat == Catch::Approx(-16.25).margin(1e-13));
    REQUIRE(n.getNodes()[1].reservoirHeat == Catch::Approx(15.75).margin(1e-13));
    REQUIRE(n.getDiagnostics().lastReservoirHeat == Catch::Approx(-.5).margin(1e-13));
}

TEST_CASE("Radiative stability rates are recomputed after queued heating", "[Thermal][radiation]") {
    ThermalNetworkConfig c;
    c.maxSubstep = .125;
    ThermalNetwork n(c);
    n.addNode(0, 1);
    n.addNode(0, 1, true);
    n.addRadiationLink(0, 1, 1);
    n.applyPower(0, 64);
    n.step(.25);
    const auto d = n.getDiagnostics();
    REQUIRE(d.lastSubsteps > 2);
    REQUIRE(n.getNodes()[0].temperature >= 0);
    REQUIRE(n.getNodes()[0].temperature < 8);
    REQUIRE(d.totalEnergy ==
            Catch::Approx(d.totalExternalEnergy + d.totalReservoirHeat).margin(2e-13));
    REQUIRE(d.lastExternalEnergy == Catch::Approx(16).margin(2e-13));
    // Reducing the nonlinear work allowance must roll back the earlier heating
    // substep.
    n = ThermalNetwork(c);
    n.addNode(0, 1);
    n.addNode(0, 1, true);
    n.addRadiationLink(0, 1, 1);
    n.applyPower(0, 64);
    c.maxSubsteps = 2;
    n.setConfig(c);
    const auto before = n;
    REQUIRE_THROWS_AS(n.step(.25), std::runtime_error);
    SameState(n, before);
}

TEST_CASE("Radiation avoids intermediate fourth power overflow and underflow",
          "[Thermal][radiation]") {
    for (const double scale : {1e-90, 1.0, 1e90}) {
        ThermalNetworkConfig c;
        c.maxSubstep = 1;
        ThermalNetwork n(c);
        n.addNode(2 * scale, 1 / scale);
        n.addNode(scale, 2 / scale);
        // h*kappa*scale^4=1/128, but scale^4 need not be representable.
        // Split the scaling between coefficient, duration and capacities.
        const double coefficient = std::pow(scale, -3) / 16;
        n.addRadiationLink(0, 1, coefficient);
        const double dt = .125 / scale;
        c.maxSubstep = dt;
        n.setConfig(c);
        n.step(dt);
        const double q = -15.0 / 128;
        REQUIRE(n.getNodes()[0].temperature / scale == Catch::Approx(2 + q).epsilon(3e-15));
        REQUIRE(n.getNodes()[1].temperature / scale == Catch::Approx(1 - q / 2).epsilon(3e-15));
        REQUIRE(n.getDiagnostics().totalEnergy == Catch::Approx(4).epsilon(3e-15));
        REQUIRE(n.getDiagnostics().lastSubsteps == 1);
    }
}

TEST_CASE("Radiative failures preserve queued loads topology and accounting",
          "[Thermal][radiation]") {
    auto n = Radiator(.01);
    n.step(.01);
    n.applyPower(0, -1e6);
    const auto before = n;
    REQUIRE_THROWS_AS(n.step(.1), std::runtime_error);
    SameState(n, before);
    n.clearPowers();
    n.applyPower(0, 1);
    const auto temperature = n.getNodes()[0].temperature;
    n.step(0);
    REQUIRE(n.getNodes()[0].temperature == temperature);
    REQUIRE(n.getNodes()[0].externalPower == 1);
    REQUIRE(n.getDiagnostics().lastRadiativeVisits == 0);
    n.setTemperature(0, 1e300);
    const auto extreme = n;
    REQUIRE_THROWS_AS(n.step(.01), std::runtime_error);
    SameState(n, extreme);
}

TEST_CASE("Nearly equal radiating reservoirs retain the small physical temperature difference",
          "[Thermal][radiation]") {
    for (const double temperature : {1.0, 1e80}) {
        ThermalNetworkConfig c;
        c.maxSubstep = 1;
        ThermalNetwork n(c);
        const double hotter = std::nextafter(temperature, std::numeric_limits<double>::infinity());
        const double coefficient = temperature == 1 ? 1 : 1e-240;
        n.addNode(temperature, 1 / temperature, true);
        n.addNode(hotter, 1 / temperature, true);
        n.addRadiationLink(0, 1, coefficient);
        // Independent divided difference uses normalized temperatures and the
        // binomial identity (1+d)^4-1=d*(4+d*(6+d*(4+d))). No fourth-power
        // subtraction or the implementation's low/high ratio is used.
        const long double relative =
            (static_cast<long double>(hotter) - temperature) / temperature;
        const long double factor = relative * (4 + relative * (6 + relative * (4 + relative)));
        const long double scale = temperature;
        const double expected = static_cast<double>(
            .5L * coefficient * scale * scale * scale * scale * factor);
        n.step(.5);
        REQUIRE(expected > 0);
        REQUIRE(n.getNodes()[0].reservoirHeat == Catch::Approx(-expected).epsilon(3e-15));
        REQUIRE(n.getNodes()[1].reservoirHeat == Catch::Approx(expected).epsilon(3e-15));
        REQUIRE(n.getDiagnostics().lastReservoirHeat == 0);
    }
}

TEST_CASE("Radiative links validate independent pairs and share the topology budget",
          "[Thermal][radiation]") {
    ThermalNetwork n;
    n.addNode(1, 1);
    n.addNode(2, 1);
    n.addNode(3, 1);
    REQUIRE_THROWS_AS(n.addRadiationLink(0, 3, 1), std::out_of_range);
    REQUIRE_THROWS_AS(n.addRadiationLink(0, 0, 1), std::invalid_argument);
    for (double coefficient :
         {-1.0, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()})
        REQUIRE_THROWS_AS(n.addRadiationLink(0, 1, coefficient), std::invalid_argument);
    REQUIRE(n.getRadiationLinks().empty());
    REQUIRE(n.addRadiationLink(0, 1, 0) == 0);
    REQUIRE_THROWS_AS(n.addRadiationLink(1, 0, 1), std::invalid_argument);
    REQUIRE(n.addLink(0, 1, 1) == 0); // Distinct simultaneous heat-transfer mechanisms.
    auto c = n.getConfig();
    c.maxLinks = 2;
    n.setConfig(c);
    REQUIRE_THROWS_AS(n.addRadiationLink(1, 2, 1), std::length_error);
    REQUIRE_THROWS_AS(n.addLink(1, 2, 1), std::length_error);
    c.maxLinks = 1;
    REQUIRE_THROWS_AS(n.setConfig(c), std::length_error);
    REQUIRE(n.getConfig().maxLinks == 2);
    REQUIRE(n.getRadiationLinks()[0].coefficient == 0);
    n.step(.01);
    REQUIRE(n.getDiagnostics().totalEnergy == Catch::Approx(6).margin(1e-14));
}

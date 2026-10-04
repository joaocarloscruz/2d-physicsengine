#include "catch_amalgamated.hpp"
#include "physics/physics.h"

#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

using namespace PhysicsEngine;
namespace {
ThermalNetwork twoNodes(double h) {
    ThermalNetworkConfig config;
    config.maxSubstep = h;
    ThermalNetwork network(config);
    network.addNode(400, 2);
    network.addNode(300, 3);
    network.addLink(0, 1, 4);
    return network;
}
double temperatureDifference(const ThermalNetwork& network) {
    return network.getNodes()[0].temperature - network.getNodes()[1].temperature;
}
}

TEST_CASE("ThermalNetwork pair relaxation follows the analytic exponential", "[Thermal]") {
    auto network = twoNodes(0.0005);
    const double initialEnergy = network.getDiagnostics().totalEnergy;
    const double time = 0.6;
    network.step(time);
    const double difference = 100.0 * std::exp(-4.0 * (0.5 + 1.0 / 3.0) * time);
    REQUIRE(temperatureDifference(network) == Catch::Approx(difference).margin(0.025));
    const double equilibrium = initialEnergy / 5.0;
    REQUIRE(network.getNodes()[0].temperature == Catch::Approx(equilibrium + 3.0 / 5.0 * difference).margin(0.015));
    REQUIRE(network.getNodes()[1].temperature == Catch::Approx(equilibrium - 2.0 / 5.0 * difference).margin(0.01));
    REQUIRE(network.getDiagnostics().totalEnergy == Catch::Approx(initialEnergy).margin(1e-9));
    network.step(1.4);
    REQUIRE(temperatureDifference(network) < 0.13);
    REQUIRE(network.getDiagnostics().totalEnergy == Catch::Approx(initialEnergy).margin(1e-9));
}

TEST_CASE("ThermalNetwork explicit Euler converges at first order", "[Thermal]") {
    auto coarse = twoNodes(0.02);
    auto medium = twoNodes(0.01);
    auto fine = twoNodes(0.005);
    coarse.step(0.3);
    medium.step(0.3);
    fine.step(0.3);
    const double exact = 100.0 * std::exp(-1.0);
    const double e0 = std::abs(temperatureDifference(coarse) - exact);
    const double e1 = std::abs(temperatureDifference(medium) - exact);
    const double e2 = std::abs(temperatureDifference(fine) - exact);
    REQUIRE(e0 > 1.95 * e1);
    REQUIRE(e0 < 2.1 * e1);
    REQUIRE(e1 > 1.95 * e2);
    REQUIRE(e1 < 2.1 * e2);
    REQUIRE(e2 < 0.32);
}

TEST_CASE("ThermalNetwork heterogeneous isolated graph conserves energy", "[Thermal]") {
    ThermalNetwork network;
    for (int i = 0; i < 20; ++i) network.addNode(250.0 + 5.0 * i, 1.0 + 0.25 * i);
    for (std::size_t i = 0; i + 1 < network.getNodes().size(); ++i)
        network.addLink(i, i + 1, 10.0 + i);
    network.addLink(0, 19, 30);
    const double energy = network.getDiagnostics().totalEnergy;
    auto repeat = network;
    for (int i = 0; i < 2000; ++i) {
        network.step(0.01);
        repeat.step(0.01);
    }
    REQUIRE(network.getDiagnostics().totalEnergy == Catch::Approx(energy).margin(2e-8));
    REQUIRE(network.getDiagnostics().totalExternalEnergy == 0);
    REQUIRE(network.getDiagnostics().totalReservoirHeat == 0);
    for (std::size_t i = 0; i < network.getNodes().size(); ++i)
        REQUIRE(network.getNodes()[i].temperature == repeat.getNodes()[i].temperature);
}

TEST_CASE("ThermalNetwork equilibrium including reservoirs is stationary", "[Thermal]") {
    ThermalNetwork network;
    network.addNode(300, 1);
    network.addNode(300, 2);
    network.addNode(300, 3, true);
    network.addLink(0, 1, 1);
    network.addLink(1, 2, 100);
    network.step(1.0);
    for (const auto& node : network.getNodes()) REQUIRE(node.temperature == 300);
    REQUIRE(network.getDiagnostics().lastReservoirHeat == 0);
    REQUIRE(network.getDiagnostics().totalEnergy == 1800);
}

TEST_CASE("ThermalNetwork graph bound preserves the conduction maximum principle", "[Thermal]") {
    ThermalNetworkConfig config;
    config.maxSubstep = 1.0;
    config.safetyFactor = 0.9;
    ThermalNetwork network(config);
    network.addNode(0, 1);
    for (int i = 0; i < 4; ++i) {
        network.addNode(400, 1, i == 3);
        network.addLink(0, i + 1, 100);
    }
    network.step(0.02);
    REQUIRE(network.getDiagnostics().lastSubsteps == 9);
    REQUIRE(network.getDiagnostics().minimumTemperature >= 0);
    REQUIRE(network.getDiagnostics().maximumTemperature <= 400);
    REQUIRE(network.getNodes()[0].temperature > 0);
    for (int i = 0; i < 500; ++i) {
        network.step(0.02);
        REQUIRE(network.getDiagnostics().minimumTemperature >= 0);
        REQUIRE(network.getDiagnostics().maximumTemperature <= 400);
    }
}

TEST_CASE("ThermalNetwork unit safety factor stays positive at the representable conduction bound", "[Thermal]") {
    // This exact pair formerly produced -1.3877787807814457e-17 K without power.
    constexpr double capacity = 8.117670883489172;
    constexpr double conductance = 7.0724471232455031;
    constexpr double hot = 0.10820222073046995;
    const double duration = 1.0 / (conductance / capacity);
    for (const double scale : {1.0, 1e-200, 1e-50, 1e50, 1e200}) {
        for (const int degree : {1, 3, 17}) {
            for (const bool reservoirs : {false, true}) {
                CAPTURE(scale, degree, reservoirs);
                ThermalNetworkConfig config;
                config.safetyFactor = 1;
                config.maxSubstep = 10;
                ThermalNetwork graph(config);
                graph.addNode(hot, scale * capacity);
                for (int i = 0; i < degree; ++i) {
                    graph.addNode(0, scale * capacity * (2 + 0.25 * i), reservoirs);
                    graph.addLink(0, i + 1, scale * conductance * (i + 1) / (degree * (degree + 1) / 2));
                    if (i > 0) graph.addLink(i, i + 1, scale * conductance / (2 * degree));
                }
                const double energy = graph.getDiagnostics().totalEnergy;
                for (int step = 0; step < 20; ++step) {
                    REQUIRE_NOTHROW(graph.step(duration));
                    const auto d = graph.getDiagnostics();
                    REQUIRE(d.minimumTemperature >= 0);
                    REQUIRE(d.maximumTemperature <= hot);
                    if (!reservoirs) REQUIRE(d.totalEnergy == Catch::Approx(energy).epsilon(2e-13));
                    else REQUIRE(d.totalEnergy - energy == Catch::Approx(d.totalReservoirHeat).epsilon(2e-13));
                }
            }
        }
    }
}

TEST_CASE("ThermalNetwork does not round a truly above-bound conduction step down", "[Thermal]") {
    ThermalNetworkConfig config;
    config.maxSubstep = 2;
    config.safetyFactor = 1;
    config.maxSubsteps = 1;
    ThermalNetwork graph(config);
    graph.addNode(100, 1);
    graph.addNode(0, 1, true);
    graph.addLink(0, 1, 1);
    const double tooLarge = std::nextafter(1.0, 2.0);
    REQUIRE_THROWS_AS(graph.step(tooLarge), std::runtime_error);
    REQUIRE(graph.getNodes()[0].temperature == 100);
    REQUIRE(graph.getNodes()[1].reservoirHeat == 0);
}

TEST_CASE("ThermalNetwork pair heat flow has equal and opposite energy transfers", "[Thermal]") {
    ThermalNetworkConfig config;
    config.maxSubstep = 1;
    ThermalNetwork network(config);
    network.addNode(400, 2);
    network.addNode(300, 3);
    network.addLink(0, 1, 4);
    network.step(0.1);
    REQUIRE(network.getNodes()[0].temperature == Catch::Approx(380).margin(1e-12));
    REQUIRE(network.getNodes()[1].temperature == Catch::Approx(300 + 40.0 / 3).margin(1e-12));
    REQUIRE(network.getDiagnostics().totalEnergy == Catch::Approx(1700).margin(1e-12));
}

TEST_CASE("ThermalNetwork reservoirs and external loads close the energy budget", "[Thermal]") {
    ThermalNetworkConfig config;
    config.maxSubstep = 1;
    ThermalNetwork network(config);
    network.addNode(300, 2);
    network.addNode(400, 5, true);
    network.addLink(0, 1, 4);
    network.applyPower(0, 6);
    network.applyPower(0, 4);
    network.applyPower(1, 20);
    const double before = network.getDiagnostics().totalEnergy;
    network.step(0.1);
    const auto d = network.getDiagnostics();
    REQUIRE(network.getNodes()[0].temperature == Catch::Approx(320.5).margin(1e-12));
    REQUIRE(network.getNodes()[1].temperature == 400);
    REQUIRE(d.lastExternalEnergy == Catch::Approx(3).margin(1e-12));
    REQUIRE(d.lastReservoirHeat == Catch::Approx(38).margin(1e-12));
    REQUIRE(network.getNodes()[1].reservoirHeat == Catch::Approx(38).margin(1e-12));
    REQUIRE(d.totalEnergy - before == Catch::Approx(d.totalExternalEnergy + d.totalReservoirHeat).margin(1e-12));
    REQUIRE(network.getNodes()[0].externalPower == 0);
    REQUIRE(network.getNodes()[1].externalPower == 0);
    for (int i = 0; i < 100; ++i) {
        network.applyPower(0, 10);
        network.applyPower(1, 20);
        network.step(0.1);
    }
    const auto after = network.getDiagnostics();
    REQUIRE(after.totalEnergy - before == Catch::Approx(after.totalExternalEnergy + after.totalReservoirHeat).margin(1e-9));
    REQUIRE(after.totalExternalEnergy == Catch::Approx(303).margin(1e-10));
    REQUIRE(after.totalReservoirHeat == network.getNodes()[1].reservoirHeat);
}

TEST_CASE("ThermalNetwork accounts separately for multiple fixed reservoirs", "[Thermal]") {
    ThermalNetworkConfig config;
    config.maxSubstep = 1;
    ThermalNetwork network(config);
    network.addNode(200, 1, true);
    network.addNode(400, 1, true);
    network.addLink(0, 1, 2);
    network.step(0.1);
    REQUIRE(network.getNodes()[0].temperature == 200);
    REQUIRE(network.getNodes()[1].temperature == 400);
    REQUIRE(network.getNodes()[0].reservoirHeat == -40);
    REQUIRE(network.getNodes()[1].reservoirHeat == 40);
    REQUIRE(network.getDiagnostics().totalReservoirHeat == 0);
    network.applyPower(0, 20);
    network.step(0.1);
    REQUIRE(network.getDiagnostics().lastExternalEnergy == 2);
    REQUIRE(network.getDiagnostics().lastReservoirHeat == -2);
    REQUIRE(network.getDiagnostics().totalEnergy == 600);
}

TEST_CASE("ThermalNetwork queued powers persist across failure and zero time", "[Thermal]") {
    ThermalNetworkConfig config;
    config.maxSubstep = 0.01;
    config.maxSubsteps = 7;
    ThermalNetwork network(config);
    network.addNode(300, 2);
    network.applyPower(0, 2);
    network.step(0);
    REQUIRE(network.getNodes()[0].externalPower == 2);
    network.step(0.07);
    REQUIRE(network.getDiagnostics().lastSubsteps == 7);
    REQUIRE(network.getNodes()[0].temperature == Catch::Approx(300.07).margin(1e-11));
    network.applyPower(0, 10);
    const auto before = network.getDiagnostics();
    REQUIRE_THROWS_AS(network.step(0.071), std::runtime_error);
    REQUIRE(network.getNodes()[0].externalPower == 10);
    REQUIRE(network.getDiagnostics().lastSubsteps == before.lastSubsteps);
    REQUIRE(network.getDiagnostics().totalExternalEnergy == before.totalExternalEnergy);
    network.clearPowers(0);
    REQUIRE(network.getNodes()[0].externalPower == 0);
    network.applyPower(0, 4);
    network.clearPowers();
    REQUIRE(network.getNodes()[0].externalPower == 0);
    network.step(0);
    REQUIRE(network.getDiagnostics().lastSubsteps == 0);
    REQUIRE(network.getDiagnostics().lastExternalEnergy == 0);
    REQUIRE(network.getDiagnostics().totalExternalEnergy == before.totalExternalEnergy);
}

TEST_CASE("ThermalNetwork negative power cannot cross zero Kelvin", "[Thermal]") {
    ThermalNetworkConfig config;
    config.maxSubstep = 1;
    ThermalNetwork network(config);
    network.addNode(1, 2);
    network.addNode(300, 3);
    network.applyPower(0, -4);
    network.applyPower(1, 2);
    REQUIRE_THROWS_AS(network.step(1), std::runtime_error);
    REQUIRE(network.getNodes()[0].temperature == 1);
    REQUIRE(network.getNodes()[1].temperature == 300);
    REQUIRE(network.getNodes()[0].externalPower == -4);
    REQUIRE(network.getDiagnostics().totalExternalEnergy == 0);
    network.step(0.5);
    REQUIRE(network.getNodes()[0].temperature == 0);
    REQUIRE(network.getNodes()[1].temperature == Catch::Approx(300 + 1.0 / 3).margin(1e-12));
    REQUIRE(network.getDiagnostics().lastExternalEnergy == -1);
}

TEST_CASE("ThermalNetwork validates finite parameters topology indexes and budgets", "[Thermal]") {
    const double inf = std::numeric_limits<double>::infinity();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    ThermalNetwork network;
    REQUIRE_THROWS_AS(network.addNode(-1, 1), std::invalid_argument);
    REQUIRE_THROWS_AS(network.addNode(inf, 1), std::invalid_argument);
    REQUIRE_THROWS_AS(network.addNode(nan, 1), std::invalid_argument);
    REQUIRE_THROWS_AS(network.addNode(300, 0), std::invalid_argument);
    REQUIRE_THROWS_AS(network.addNode(300, -1), std::invalid_argument);
    REQUIRE_THROWS_AS(network.addNode(300, inf), std::invalid_argument);
    REQUIRE_THROWS_AS(network.addNode(300, std::numeric_limits<double>::denorm_min()), std::invalid_argument);
    REQUIRE_THROWS_AS(network.addNode(1e308, 1e308), std::invalid_argument);
    network.addNode(300, 1);
    network.addNode(400, 2);
    REQUIRE_THROWS_AS(network.addLink(0, 2, 1), std::out_of_range);
    REQUIRE_THROWS_AS(network.addLink(0, 0, 1), std::invalid_argument);
    REQUIRE_THROWS_AS(network.addLink(0, 1, -1), std::invalid_argument);
    REQUIRE_THROWS_AS(network.addLink(0, 1, inf), std::invalid_argument);
    network.addLink(0, 1, 0);
    REQUIRE_THROWS_AS(network.addLink(1, 0, 1), std::invalid_argument);
    REQUIRE_THROWS_AS(network.setTemperature(0, -1), std::invalid_argument);
    REQUIRE_THROWS_AS(network.setTemperature(0, nan), std::invalid_argument);
    REQUIRE_THROWS_AS(network.setTemperature(2, 300), std::out_of_range);
    REQUIRE_THROWS_AS(network.setFixed(2, true), std::out_of_range);
    REQUIRE_THROWS_AS(network.applyPower(2, 1), std::out_of_range);
    REQUIRE_THROWS_AS(network.clearPowers(2), std::out_of_range);
    REQUIRE_THROWS_AS(network.applyPower(0, inf), std::invalid_argument);
    REQUIRE_THROWS_AS(network.applyPower(0, nan), std::invalid_argument);
    REQUIRE_THROWS_AS(network.step(-1), std::invalid_argument);
    REQUIRE_THROWS_AS(network.step(nan), std::invalid_argument);
    REQUIRE_THROWS_AS(network.step(inf), std::invalid_argument);
    auto config = network.getConfig();
    config.maxNodes = 2;
    config.maxLinks = 1;
    network.setConfig(config);
    REQUIRE_THROWS_AS(network.addNode(300, 1), std::length_error);
    config.maxNodes = 1;
    REQUIRE_THROWS_AS(network.setConfig(config), std::length_error);
    config.maxNodes = 3;
    network.setConfig(config);
    network.addNode(300, 1);
    REQUIRE_THROWS_AS(network.addLink(1, 2, 1), std::length_error);
    config.maxSubsteps = 0;
    REQUIRE_THROWS_AS(ThermalNetwork(config), std::invalid_argument);
    config = network.getConfig();
    config.maxSubstep = 0;
    REQUIRE_THROWS_AS(ThermalNetwork(config), std::invalid_argument);
    config = network.getConfig();
    config.safetyFactor = 1.01;
    REQUIRE_THROWS_AS(ThermalNetwork(config), std::invalid_argument);
    network.setFixed(0, true);
    network.setTemperature(0, 350);
    REQUIRE(network.getNodes()[0].temperature == 350);
    network.setFixed(0, false);
    REQUIRE_FALSE(network.getNodes()[0].fixed);
}

TEST_CASE("ThermalNetwork derived overflow and staged failures are atomic", "[Thermal]") {
    SECTION("Power accumulation") {
        ThermalNetwork network;
        network.addNode(300, 1);
        network.applyPower(0, 1e308);
        REQUIRE_THROWS_AS(network.applyPower(0, 1e308), std::runtime_error);
        REQUIRE(network.getNodes()[0].externalPower == 1e308);
        REQUIRE_THROWS_AS(network.step(20), std::runtime_error);
        REQUIRE(network.getNodes()[0].temperature == 300);
        REQUIRE(network.getNodes()[0].externalPower == 1e308);
    }
    SECTION("Conductance rate") {
        ThermalNetwork network;
        network.addNode(300, 1e-308);
        network.addNode(400, 1);
        network.addLink(0, 1, 10);
        network.applyPower(1, 1);
        REQUIRE_THROWS_AS(network.step(0.01), std::runtime_error);
        REQUIRE(network.getNodes()[0].temperature == 300);
        REQUIRE(network.getNodes()[1].externalPower == 1);
    }
    SECTION("Heat flux") {
        ThermalNetworkConfig config;
        config.maxSubstep = 10;
        ThermalNetwork network(config);
        network.addNode(0, 1, true);
        network.addNode(2, 1, true);
        network.addLink(0, 1, 1e308);
        REQUIRE_THROWS_AS(network.step(10), std::runtime_error);
        REQUIRE(network.getNodes()[0].reservoirHeat == 0);
        REQUIRE(network.getDiagnostics().totalReservoirHeat == 0);
    }
    SECTION("Later substep fails after earlier staged heating") {
        ThermalNetworkConfig config;
        config.maxSubstep = 0.1;
        ThermalNetwork network(config);
        network.addNode(1, 1);
        network.addNode(100, 1);
        network.applyPower(0, -4);
        network.applyPower(1, 5);
        REQUIRE_THROWS_AS(network.step(0.5), std::runtime_error);
        REQUIRE(network.getNodes()[0].temperature == 1);
        REQUIRE(network.getNodes()[1].temperature == 100);
        REQUIRE(network.getNodes()[1].externalPower == 5);
        REQUIRE(network.getDiagnostics().lastExternalEnergy == 0);
    }
    SECTION("Failed cooling preserves prior reservoir and power accounting") {
        ThermalNetworkConfig config;
        config.maxSubstep = 0.1;
        ThermalNetwork network(config);
        network.addNode(400, 1, true);
        network.addNode(300, 1);
        network.addLink(0, 1, 1);
        network.step(0.1);
        const auto before = network.getDiagnostics();
        const double reservoirBefore = network.getNodes()[0].reservoirHeat;
        network.applyPower(0, 10);
        network.applyPower(1, -1000);
        REQUIRE_THROWS_AS(network.step(0.5), std::runtime_error);
        REQUIRE(network.getNodes()[1].temperature == 310);
        REQUIRE(network.getNodes()[0].reservoirHeat == reservoirBefore);
        REQUIRE(network.getNodes()[0].externalPower == 10);
        REQUIRE(network.getNodes()[1].externalPower == -1000);
        const auto after = network.getDiagnostics();
        REQUIRE(after.totalReservoirHeat == before.totalReservoirHeat);
        REQUIRE(after.totalExternalEnergy == before.totalExternalEnergy);
        REQUIRE(after.lastReservoirHeat == before.lastReservoirHeat);
        REQUIRE(after.lastSubsteps == before.lastSubsteps);
    }
    SECTION("Aggregate energy") {
        ThermalNetwork network;
        network.addNode(1e308, 1);
        network.addNode(1e308, 1);
        REQUIRE_THROWS_AS(network.getDiagnostics(), std::runtime_error);
        network.applyPower(0, 1);
        REQUIRE_THROWS_AS(network.step(0.01), std::runtime_error);
        REQUIRE(network.getNodes()[0].externalPower == 1);
    }
}

TEST_CASE("ThermalNetwork finite heat transfers survive intermediate overflow or underflow", "[Thermal]") {
    for (const auto& parameters : {
        std::array<double, 3>{0.01, 1e308, 2},
        std::array<double, 3>{1e-200, 1e-200, 1e200},
        std::array<double, 3>{1e200, 1e200, 1e-200}}) {
        const double duration = parameters[0];
        const double conductance = parameters[1];
        const double temperature = parameters[2];
        ThermalNetworkConfig config;
        config.maxSubstep = duration;
        ThermalNetwork network(config);
        network.addNode(0, 1, true);
        network.addNode(temperature, temperature > 1 ? 1.0 / temperature : 1.0, true);
        network.addLink(0, 1, conductance);
        REQUIRE_NOTHROW(network.step(duration));
        const double expected = duration == 0.01 ? 2e306 : duration;
        REQUIRE(network.getNodes()[0].reservoirHeat == Catch::Approx(-expected).epsilon(2e-15));
        REQUIRE(network.getNodes()[1].reservoirHeat == Catch::Approx(expected).epsilon(2e-15));
        REQUIRE(network.getDiagnostics().lastReservoirHeat == 0);
        REQUIRE(network.getNodes()[1].temperature == temperature);
    }
}

TEST_CASE("ThermalNetwork empty and disconnected graphs are well defined", "[Thermal]") {
    ThermalNetwork empty;
    empty.step(1);
    REQUIRE(empty.getDiagnostics().lastSubsteps == 0);
    REQUIRE(empty.getDiagnostics().totalEnergy == 0);
    ThermalNetwork network;
    network.addNode(0, 1);
    network.addNode(300, 1);
    network.addLink(0, 1, 0);
    network.step(0.1);
    REQUIRE(network.getNodes()[0].temperature == 0);
    REQUIRE(network.getNodes()[1].temperature == 300);
    network.applyPower(0, 10);
    network.step(0.1);
    REQUIRE(network.getNodes()[0].temperature == Catch::Approx(1).margin(1e-12));
    REQUIRE(network.getDiagnostics().totalExternalEnergy == Catch::Approx(1).margin(1e-12));
}

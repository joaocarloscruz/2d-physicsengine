#include "physics/physics.h"
#include <cmath>
#include <iomanip>
#include <iostream>

int main() {
    using namespace PhysicsEngine;
    // Gray body facing a large black enclosure at 0 K; lumped, fixed geometry.
    // sigma is the rounded SI Stefan-Boltzmann constant (NIST CODATA).
    constexpr double sigma = 5.670374419e-8;
    constexpr double coefficient = .8 * sigma * .02; // emissivity * sigma * area, W/K^4.
    constexpr double initialTemperature = 500, capacity = 20, duration = 200;
    const double exact =
        initialTemperature /
        std::cbrt(1 + 3 * coefficient * std::pow(initialTemperature, 3) * duration / capacity);
    double previousError = 0;
    std::cout << std::setprecision(12);
    for (double h : {.25, .125, .0625}) {
        ThermalNetworkConfig config;
        config.maxSubstep = h;
        ThermalNetwork network(config);
        network.addNode(initialTemperature, capacity);
        network.addNode(0, 1, true);
        network.addRadiationLink(0, 1, coefficient);
        network.step(duration);
        const auto d = network.getDiagnostics();
        const double temperature = network.getNodes()[0].temperature;
        const double error = std::abs(temperature - exact);
        const double residual =
            d.totalEnergy - capacity * initialTemperature - d.totalReservoirHeat;
        std::cout << "max_substep_s=" << h << " temperature_K=" << temperature
                  << " analytical_K=" << exact << " error_K=" << error
                  << " refinement_ratio=" << (previousError ? previousError / error : 0)
                  << " energy_residual_J=" << residual << " substeps=" << d.lastSubsteps
                  << " visits=" << d.lastRadiativeVisits << '\n';
        if (!std::isfinite(error) || error > .15 || std::abs(residual) > 1e-8 ||
            (previousError && (previousError / error < 1.95 || previousError / error > 2.05)))
            return 1;
        previousError = error;
    }
    return 0;
}

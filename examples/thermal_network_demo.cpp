#include "physics/physics.h"
#include <cmath>
#include <iomanip>
#include <iostream>

int main() {
    using namespace PhysicsEngine;
    ThermalNetwork graph;
    graph.addNode(280, 100);
    graph.addNode(350, 100, true);
    graph.addLink(0, 1, 2);
    const double initialEnergy = graph.getDiagnostics().totalEnergy;
    for (int step = 0; step < 1200; ++step) {
        graph.applyPower(0, 5);
        graph.step(0.1);
    }
    const auto d = graph.getDiagnostics();
    const double residual = d.totalEnergy - initialEnergy - d.totalExternalEnergy - d.totalReservoirHeat;
    std::cout << std::setprecision(12)
              << "temperature_K=" << graph.getNodes()[0].temperature
              << " external_energy_J=" << d.totalExternalEnergy
              << " reservoir_heat_J=" << d.totalReservoirHeat
              << " energy_residual_J=" << residual
              << " last_substeps=" << d.lastSubsteps << '\n';
    return std::isfinite(residual) && std::abs(residual) < 1e-7 &&
           graph.getNodes()[1].temperature == 350 ? 0 : 1;
}

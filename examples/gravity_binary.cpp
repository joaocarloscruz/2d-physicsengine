#include "physics/physics.h"
#include <cmath>
#include <iomanip>
#include <iostream>

int main() {
    using namespace PhysicsEngine;
    NBodyGravityConfig config;
    config.maxSubstep = 0.002;
    NBodyGravity orbit(config); // G=1 in reduced units.
    orbit.addParticle({-0.5, 0}, {0, -std::sqrt(0.5)});
    orbit.addParticle({0.5, 0}, {0, std::sqrt(0.5)});
    const double period = 2 * std::acos(-1.0) / std::sqrt(2.0);
    orbit.step(period);
    const auto d = orbit.getDiagnostics();
    const auto p = orbit.getParticles()[0].position;
    const double positionError = std::hypot(p.x + 0.5, p.y);
    std::cout << std::setprecision(12)
              << "period=" << period << " position_error=" << positionError
              << " total_energy=" << d.totalEnergy << " angular_momentum=" << d.angularMomentum
              << " substeps=" << d.lastSubsteps << " pair_work=" << d.lastPairWork << '\n';
    return positionError < 1e-5 && std::abs(d.totalEnergy + 0.5) < 1e-8 ? 0 : 1;
}

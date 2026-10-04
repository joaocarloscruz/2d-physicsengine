#include "physics/physics.h"
#include <cmath>
#include <iomanip>
#include <iostream>

int main() {
    using namespace PhysicsEngine;
    SoftBody rope;
    rope.addParticle({0.0f, 2.0f}, {}, 1.0, true);
    for (int i = 1; i <= 10; ++i) {
        rope.addParticle({0.0f, 2.0f - 0.2f * i}, {}, 0.1);
        rope.addSpring(i - 1, i, 0.2, 200.0, 1.0);
    }
    rope.setUniformAcceleration({0.0f, -9.81f});
    rope.applyImpulse(10, {0.05f, 0.0f});
    for (int frame = 0; frame < 1200; ++frame) rope.step(1.0 / 120.0);
    const auto metrics = rope.getDiagnostics();
    const auto tip = rope.getParticles().back().position;
    std::cout << std::setprecision(9)
              << "particles=" << rope.getParticles().size()
              << " springs=" << rope.getSprings().size()
              << " tip=(" << tip.x << ',' << tip.y << ')'
              << " kinetic=" << metrics.kineticEnergy
              << " elastic=" << metrics.elasticEnergy
              << " max_strain=" << metrics.maxStrain
              << " last_substeps=" << metrics.lastSubsteps << '\n';
    return std::isfinite(metrics.kineticEnergy) && std::isfinite(metrics.elasticEnergy) &&
           rope.getParticles().front().position == Vector2(0, 2) ? 0 : 1;
}

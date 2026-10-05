#include <physics/physics.h>
#include <cmath>
#include <iostream>

int main() {
    using namespace PhysicsEngine;
    PeriodicIncompressibleGridConfig c; c.geometry={24,24,2*std::acos(-1.)/24,2*std::acos(-1.)/24};
    c.kinematicViscosity=.1; PeriodicIncompressibleGrid flow(c); auto v=flow.velocities();
    for(std::size_t j=0;j<24;++j) for(std::size_t i=0;i<24;++i) {
        v.xFaces[i+24*j]=std::sin(i*c.geometry.spacingX)*std::cos((j+.5)*c.geometry.spacingY);
        v.yFaces[i+24*j]=-std::cos((i+.5)*c.geometry.spacingX)*std::sin(j*c.geometry.spacingY);
    }
    flow.setVelocities(v); std::size_t visits=0;
    for(int k=0;k<20;++k) {
        const auto d=flow.step(.005); visits+=d.cellVisits;
        if(std::abs(d.storageEnergyError)>d.roundoffEnergyAllowance) return 1;
    }
    const auto d=flow.lastStep();
    std::cout << "face-center Taylor-Green t=" << flow.time() << " kinetic_energy=" << d.finalKineticEnergy
              << " continuum_energy=" << std::acos(-1.)*std::acos(-1.)*std::exp(-.4*flow.time())
              << " visits=" << visits << " donor_loss_last_step=" << d.donorDissipation
              << " viscous_loss_last_step=" << d.viscousDissipation << '\n';
    return 0;
}

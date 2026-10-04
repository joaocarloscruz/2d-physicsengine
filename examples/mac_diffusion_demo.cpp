#include <physics/physics.h>
#include <cmath>
#include <iomanip>
#include <iostream>
int main() {
    try {
        using namespace PhysicsEngine;
        constexpr double pi=3.14159265358979323846;
        PeriodicMacGridConfig c; c.columns=24; c.rows=18; c.spacingX=.1; c.spacingY=.17;
        PeriodicMacGrid grid(c); auto v=grid.velocities();
        for(std::size_t j=0;j<c.rows;++j) for(std::size_t i=0;i<c.columns;++i) {
            const auto k=i+c.columns*j;
            v.xFaces[k]=.3+std::sin(4*pi*(j+.5)/c.rows);
            v.yFaces[k]=-.2+.4*std::cos(2*pi*(i+.5)/c.columns);
        }
        grid.setVelocities(v);
        MacDiffusionConfig options; options.kinematicViscosity=.07; options.timeStep=.2; options.density=1000;
        const auto d=grid.diffuse(options);
        std::cout<<std::setprecision(17)<<"{\"columns\":24,\"rows\":18,\"nu\":0.07,\"dt\":0.2,\"density\":1000"
            <<",\"iterations\":"<<d.iterations<<",\"cell_visits\":"<<d.cellVisits
            <<",\"final_residual_rms\":"<<d.finalResidualRms<<",\"target_residual_rms\":"<<d.targetResidualRms
            <<",\"mean_x\":"<<d.finalMeanX<<",\"mean_y\":"<<d.finalMeanY
            <<",\"energy_before\":"<<d.initialKineticEnergy<<",\"energy_after\":"<<d.finalKineticEnergy
            <<",\"gradient_dissipation\":"<<d.gradientDissipation<<",\"increment_energy\":"<<d.incrementKineticEnergy
            <<",\"residual_work\":"<<d.residualWork<<",\"energy_residual_bound\":"<<d.residualEnergyBound
            <<",\"storage_energy_error\":"<<d.storageEnergyError<<"}\n";
        return d.finalResidualRms<=d.targetResidualRms?0:1;
    } catch(const std::exception& error) { std::cerr<<error.what()<<'\n'; return 1; }
}

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
        const double ax=2*std::sin(pi/c.columns)/c.spacingX,ay=2*std::sin(2*pi/c.rows)/c.spacingY;
        const double tx=2*std::sin(3*pi/c.columns)/c.spacingX,ty=2*std::sin(pi/c.rows)/c.spacingY;
        for(std::size_t j=0;j<c.rows;++j) for(std::size_t i=0;i<c.columns;++i) {
            const auto k=i+c.columns*j;
            v.xFaces[k]=.3+ax*std::sin(2*pi*(static_cast<double>(i)/c.columns+2*(j+.5)/c.rows))
                +.2*ty*std::sin(2*pi*(3*static_cast<double>(i)/c.columns+(j+.5)/c.rows));
            v.yFaces[k]=-.2+ay*std::sin(2*pi*((i+.5)/c.columns+2*static_cast<double>(j)/c.rows))
                -.2*tx*std::sin(2*pi*(3*(i+.5)/c.columns+static_cast<double>(j)/c.rows));
        }
        grid.setVelocities(v); MacProjectionConfig projection; projection.density=1000; projection.timeStep=.01;
        const auto d=grid.project(projection);
        std::cout<<std::setprecision(17)<<"{\"columns\":24,\"rows\":18,\"dx\":0.1,\"dy\":0.17,\"density\":1000,\"dt\":0.01"
            <<",\"iterations\":"<<d.iterations<<",\"cell_visits\":"<<d.cellVisits
            <<",\"initial_divergence_rms\":"<<d.initialDivergenceRms<<",\"final_divergence_rms\":"<<d.finalDivergenceRms
            <<",\"target_divergence_rms\":"<<d.targetDivergenceRms<<",\"pressure_mean\":"<<d.pressureMean
            <<",\"mean_x\":"<<d.finalMeanX<<",\"mean_y\":"<<d.finalMeanY
            <<",\"energy_before\":"<<d.initialKineticEnergy<<",\"energy_after\":"<<d.finalKineticEnergy
            <<",\"energy_residual_bound\":"<<d.residualEnergyBound<<",\"storage_energy_error\":"<<d.storageEnergyError<<"}\n";
        return d.finalDivergenceRms<=d.targetDivergenceRms?0:1;
    } catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}

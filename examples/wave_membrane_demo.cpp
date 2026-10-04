#include <physics/physics.h>
#include <cmath>
#include <iostream>

int main() {
    using namespace PhysicsEngine;
    constexpr std::size_t width=33,height=25;
    constexpr double pi=3.14159265358979323846;
    WaveMembraneConfig config; config.tension=12; config.surfaceDensity=3;
    WaveMembrane membrane(width,height,1.0/(width-1),0.75/(height-1),config);
    std::vector<double> displacement(width*height),velocity(width*height);
    for(std::size_t y=1;y<height-1;++y) for(std::size_t x=1;x<width-1;++x)
        displacement[y*width+x]=0.001*std::sin(pi*x/(width-1))*std::sin(pi*y/(height-1));
    membrane.setState(displacement,velocity);
    const double initialEnergy=membrane.getDiagnostics().totalEnergy;
    for(int i=0;i<100;++i) membrane.step(0.01);
    const auto d=membrane.getDiagnostics();
    std::cout<<"time="<<d.time<<" energy="<<d.totalEnergy<<" initial_energy="<<initialEnergy
             <<" substeps="<<d.lastSubsteps<<" cell_work="<<d.lastCellWork<<'\n';
    return std::isfinite(d.totalEnergy) && d.totalEnergy>initialEnergy*0.99 && d.totalEnergy<=initialEnergy*1.001 ? 0 : 1;
}

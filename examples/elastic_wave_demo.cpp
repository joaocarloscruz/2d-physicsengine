#include "physics/core/elastic_wave_grid.h"
#include <cmath>
#include <iomanip>
#include <iostream>

int main() {
    try {
        PhysicsEngine::ElasticWaveGridConfig c;
        c.columns = 32;
        c.rows = 24;
        c.spacingX = .0625;
        c.spacingY = .125;
        c.density = 2;
        c.lambda = 3;
        c.shearModulus = 2;
        c.maxSubstep = .01;
        PhysicsEngine::ElasticWaveGrid grid(c);
        auto state = grid.getState();
        constexpr double pi = 3.14159265358979323846;
        const double x = 2 * std::sin(pi / c.columns) / c.spacingX,
                     y = 2 * std::sin(pi / c.rows) / c.spacingY, norm = std::hypot(x, y);
        for (std::size_t j = 0; j < c.rows; ++j)
            for (std::size_t i = 0; i < c.columns; ++i) {
                const auto k = i + c.columns * j;
                state.vx[k] =
                    x / norm * std::cos(2 * pi * (double(i) / c.columns + (j + .5) / c.rows));
                state.vy[k] =
                    y / norm * std::cos(2 * pi * ((i + .5) / c.columns + double(j) / c.rows));
            }
        grid.setState(state);
        const double initial = grid.getModifiedEnergy(.01);
        for (int i = 0; i < 100; ++i)
            grid.step(.01);
        const auto d = grid.getDiagnostics();
        if (std::abs(d.modifiedEnergy - initial) > 1e-11 * initial || d.maxAbsCompatibility > 1e-10)
            return 1;
        std::cout << std::setprecision(17)
                  << "{\"model\":\"periodic_plane_strain\",\"time\":" << d.time
                  << ",\"cp\":" << grid.getCompressionalSpeed()
                  << ",\"cs\":" << grid.getShearSpeed() << ",\"physicalEnergy\":" << d.totalEnergy
                  << ",\"modifiedEnergy\":" << d.modifiedEnergy
                  << ",\"initialModifiedEnergy\":" << initial
                  << ",\"compatibilityRms\":" << d.compatibilityRms
                  << ",\"lastCellVisits\":" << d.lastCellVisits << "}\n";
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}

#include "physics/physics.h"
#include <cmath>
#include <iomanip>
#include <iostream>

int main() {
    using namespace PhysicsEngine;
    constexpr double pi = 3.1415926535897932384626433832795;
    std::cout << std::setprecision(17);
    for (std::size_t columns : {16, 32, 64, 128}) {
        ElectrostaticGridConfig c{columns, columns / 2, 1.0 / columns, 2.0 / columns, 2};
        PeriodicElectrostaticGrid grid(c);
        std::vector<double> charge(c.columns * c.rows);
        for (std::size_t j = 0; j < c.rows; ++j)
            for (std::size_t i = 0; i < c.columns; ++i) {
                const double phase = 2 * pi * ((i + .5) * c.spacingX + (j + .5) * c.spacingY) + .31;
                charge[i + c.columns * j] = c.permittivity * 8 * pi * pi * .7 * std::cos(phase);
            }
        const auto d = grid.solve(charge);
        const auto s = grid.getSnapshot();
        double potentialError = 0, fieldError = 0;
        for (std::size_t j = 0; j < c.rows; ++j)
            for (std::size_t i = 0; i < c.columns; ++i) {
                const auto k = i + c.columns * j;
                const double phi =
                    .7 * std::cos(2 * pi * ((i + .5) * c.spacingX + (j + .5) * c.spacingY) + .31);
                const double ex =
                    1.4 * pi * std::sin(2 * pi * (i * c.spacingX + (j + .5) * c.spacingY) + .31);
                const double ey =
                    1.4 * pi * std::sin(2 * pi * ((i + .5) * c.spacingX + j * c.spacingY) + .31);
                potentialError += std::pow(s.potential[k] - phi, 2);
                fieldError +=
                    std::pow(s.field.xFaces[k] - ex, 2) + std::pow(s.field.yFaces[k] - ey, 2);
            }
        std::cout << "{\"columns\":" << c.columns << ",\"rows\":" << c.rows
                  << ",\"potentialRmsError\":" << std::sqrt(potentialError / charge.size())
                  << ",\"fieldRmsError\":" << std::sqrt(fieldError / (2 * charge.size()))
                  << ",\"iterations\":" << d.iterations << ",\"cellVisits\":" << d.cellVisits
                  << ",\"originalChargeMean\":" << d.originalChargeMean
                  << ",\"neutralityMeanAllowance\":" << d.neutralityMeanAllowance
                  << ",\"maximumSourceCorrection\":" << d.maximumSourceCorrection
                  << ",\"originalGaussRms\":" << d.originalGaussRms
                  << ",\"effectiveGaussRms\":" << d.finalGaussRms
                  << ",\"targetGaussRms\":" << d.targetGaussRms << ",\"curlRms\":" << d.curlRms
                  << ",\"fieldEnergy\":" << d.fieldEnergy << ",\"sourceEnergy\":" << d.sourceEnergy
                  << ",\"residualEnergyCorrection\":" << d.residualEnergyCorrection
                  << ",\"energyIdentityError\":" << d.energyIdentityError << "}\n";
    }
}

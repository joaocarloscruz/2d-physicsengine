#include <cmath>
#include <iomanip>
#include <iostream>
#include <physics/physics.h>
int main() {
    try {
        PhysicsEngine::MaxwellGridConfig c;
        c.columns = 32;
        c.rows = 24;
        c.spacingX = 2 * 3.14159265358979323846 / c.columns;
        c.spacingY = 2 * 3.14159265358979323846 / c.rows;
        c.permittivity = 2;
        c.permeability = 1.5;
        c.maxSubstep = .02;
        PhysicsEngine::MaxwellGrid grid(c);
        auto s = grid.getState();
        for (std::size_t j = 0; j < c.rows; ++j)
            for (std::size_t i = 0; i < c.columns; ++i)
                s.ez[i + c.columns * j] = std::cos(i * c.spacingX + 2 * j * c.spacingY + .31);
        grid.setState(s);
        const double initial = grid.getDiagnostics().totalEnergy,
                     reference = grid.getModifiedEnergy(.02);
        double joule = 0, represented = 0, wave = 0, modified = 0;
        PhysicsEngine::MaxwellOhmicStepDiagnostics report;
        for (int k = 0; k < 100; ++k) {
            report = grid.stepOhmic(.02, .8);
            joule += report.exactJouleEnergy;
            represented += report.representedElectricEnergyLoss;
            wave += report.wavePhysicalEnergyChange;
            modified += report.modifiedEnergyDissipation;
        }
        const auto d = grid.getDiagnostics();
        const double balance = d.totalEnergy - initial + joule - wave;
        if (std::abs(balance) > 1e-11 * initial ||
            std::abs(d.modifiedEnergy + modified - reference) > 1e-11 * reference ||
            d.maxAbsMagneticDivergence > 1e-12)
            return 1;
        std::cout << std::setprecision(17)
                  << "{\"model\":\"homogeneous_periodic_tmz_ohmic\",\"sigma\":0.8,\"time\":"
                  << d.time << ",\"initialPhysicalEnergy\":" << initial
                  << ",\"finalPhysicalEnergy\":" << d.totalEnergy
                  << ",\"exactDecaySubflowJoule\":" << joule
                  << ",\"representedElectricLoss\":" << represented
                  << ",\"wavePhysicalEnergyChange\":" << wave
                  << ",\"modifiedEnergyDissipation\":" << modified
                  << ",\"physicalBalanceResidual\":" << balance
                  << ",\"maxDivH\":" << d.maxAbsMagneticDivergence
                  << ",\"lastCellVisits\":" << report.cellVisits << "}\n";
        return 0;
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}

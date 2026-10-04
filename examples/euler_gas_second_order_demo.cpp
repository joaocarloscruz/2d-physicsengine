#include <cmath>
#include <iomanip>
#include <iostream>
#include <physics/physics.h>
int main() {
    try {
        using namespace PhysicsEngine;
        constexpr double pi = 3.14159265358979323846, u = .7, v = -.2, duration = .15;
        std::cout << std::setprecision(17);
        for (const std::size_t n : {32, 64})
            for (bool high : {false, true}) {
                EulerGasGridConfig c{n, n / 2, 1.0 / n, 2.0 / n, 1.4};
                PeriodicEulerGasGrid grid(c);
                auto s = grid.state();
                const double factor = std::sin(pi * c.spacingX) / (pi * c.spacingX) *
                                      std::sin(2 * pi * c.spacingY) / (2 * pi * c.spacingY);
                for (std::size_t j = 0; j < c.rows; ++j)
                    for (std::size_t i = 0; i < n; ++i) {
                        const auto k = i + n * j;
                        s.density[k] =
                            1 + .2 * factor *
                                    std::cos(2 * pi *
                                             ((i + .5) * c.spacingX + 2 * (j + .5) * c.spacingY));
                        s.momentumX[k] = s.density[k] * u;
                        s.momentumY[k] = s.density[k] * v;
                        s.totalEnergy[k] = 1 / (c.gamma - 1) + .5 * s.density[k] * (u * u + v * v);
                    }
                grid.setState(s);
                const auto d =
                    high ? grid.stepSecondOrder(duration) : EulerGasSecondOrderDiagnostics{};
                const auto base = high ? EulerGasDiagnostics(d) : grid.step(duration);
                const auto actual = grid.state();
                double square = 0;
                for (std::size_t j = 0; j < c.rows; ++j)
                    for (std::size_t i = 0; i < n; ++i) {
                        const double exact =
                            1 + .2 * factor *
                                    std::cos(2 * pi *
                                             ((i + .5) * c.spacingX + 2 * (j + .5) * c.spacingY -
                                              (u + 2 * v) * duration));
                        square += std::pow(actual.density[i + n * j] - exact, 2);
                    }
                std::cout << "{\"second_order\":" << (high ? "true" : "false")
                          << ",\"columns\":" << n << ",\"rows\":" << n / 2
                          << ",\"rejected_attempts\":" << d.rejectedAttempts
                          << ",\"reconstruction_trials\":" << d.reconstructionTrials
                          << ",\"minimum_slope_scale\":" << d.minimumSlopeScale
                          << ",\"duration\":" << duration << ",\"substeps\":" << base.substeps
                          << ",\"cell_visits\":" << base.cellVisits
                          << ",\"maximum_cfl\":" << base.maximumCfl
                          << ",\"minimum_density\":" << base.final.minimumDensity
                          << ",\"minimum_pressure\":" << base.final.minimumPressure
                          << ",\"mass_defect\":" << base.massDefect
                          << ",\"momentum_x_defect\":" << base.momentumXDefect
                          << ",\"momentum_y_defect\":" << base.momentumYDefect
                          << ",\"energy_before\":" << base.initial.totalEnergy
                          << ",\"energy_after\":" << base.final.totalEnergy
                          << ",\"total_energy_defect\":" << base.totalEnergyDefect
                          << ",\"total_energy_allowance\":" << base.totalEnergyRoundoffAllowance
                          << ",\"continuous_density_rms_error\":"
                          << std::sqrt(square / actual.density.size()) << "}\n";
            }
        return 0;
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}

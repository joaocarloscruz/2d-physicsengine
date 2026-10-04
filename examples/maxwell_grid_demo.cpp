#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <physics/physics.h>

int main() {
    using namespace PhysicsEngine;
    constexpr double pi = 3.14159265358979323846, h = .02;
    MaxwellGridConfig c;
    c.columns = 32;
    c.rows = 24;
    c.spacingX = 2 * pi / c.columns;
    c.spacingY = 2 * pi / c.rows;
    c.maxSubstep = h;
    MaxwellGrid grid(c);
    auto fields = grid.getState();
    for (std::size_t j = 0; j < c.rows; ++j)
        for (std::size_t i = 0; i < c.columns; ++i)
            fields.ez[i + c.columns * j] = std::cos(i * c.spacingX + 2 * j * c.spacingY + .31);
    grid.setState(fields);
    const double initial = grid.getModifiedEnergy(h);
    double minimum = grid.getDiagnostics().totalEnergy, maximum = minimum;
    for (int n = 0; n < 1000; ++n) {
        grid.step(h);
        const auto d = grid.getDiagnostics();
        minimum = std::min(minimum, d.totalEnergy);
        maximum = std::max(maximum, d.totalEnergy);
    }
    const auto d = grid.getDiagnostics();
    std::cout << std::setprecision(16) << "Periodic homogeneous TMz; reduced units eps=mu=1\n"
              << "time=" << d.time << " h=" << d.lastSubstep
              << " last cell visits=" << d.lastCellVisits << '\n'
              << "physical energy range=" << minimum << ", " << maximum << '\n'
              << "modified energy=" << d.modifiedEnergy << " initial=" << initial << '\n'
              << "magnetic divergence RMS=" << d.magneticDivergenceRms << '\n';
    return std::isfinite(d.totalEnergy) && std::abs(d.modifiedEnergy - initial) < 1e-10 * initial &&
                   d.magneticDivergenceRms < 1e-12 && maximum - minimum > 1e-5 * initial
               ? 0
               : 1;
}

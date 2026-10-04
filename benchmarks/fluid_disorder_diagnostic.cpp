#include "fluid_disorder_operator.h"
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

using namespace PhysicsEngine;
using namespace FluidDisorder;
namespace {
const char *Name(Family f) {
    ValidateFamily(f);
    return f == Family::Poly6Spiky ? "legacy" : (f == Family::CubicSpline ? "cubic" : "wendland-c2");
}
double Separation(const std::vector<FluidParticle> &p) {
    double value = 1e100;
    for (std::size_t i = 0; i < p.size(); ++i)
        for (std::size_t j = 0; j < i; ++j)
            value = std::min(value, std::hypot(double(p[i].position.x) - p[j].position.x,
                                               double(p[i].position.y) - p[j].position.y));
    return value;
}
double Kinetic(const std::vector<FluidParticle> &p) {
    double energy = 0;
    for (const auto &v : p)
        energy += .5 * v.mass *
                  (double(v.velocity.x) * v.velocity.x + double(v.velocity.y) * v.velocity.y);
    return energy;
}
void Run(std::ostream &out, const char *label, Family family, bool clamp, int steps, int side = 21,
         float dx = .1f, float ratio = 2, float massScale = OriginalMassScale) {
    const float h = dx * ratio, dt = .2f / steps;
    auto particles = Block(side, dx, h, massScale);
    const auto disturbed = particles, nominal = Block(side, dx, h, massScale, .02f, false);
    WcsphConfig c;
    c.externalAcceleration = {};
    c.speedOfSound = 15;
    c.kernelFamily = family;
    c.maximumTimeStep = dt;
    c.clampNegativePressure = clamp;
    WcsphSolver solver(h, c);
    solver.prepare(particles);
    double rhoMin = 1e100, rhoMax = 0, pMin = 1e100, pMax = -1e100, bulkMin = 1e100, bulkMax = 0;
    double bulkAcceleration = 0, surfaceAcceleration = 0, initialRms = 0, totalMass = 0;
    std::size_t bulkCount = 0;
    const int margin = int(std::ceil(ratio)) + 1;
    for (std::size_t i = 0; i < particles.size(); ++i) {
        const auto &p = particles[i];
        const int x = int(i % side), y = int(i / side);
        const double acceleration = std::hypot(double(p.force.x), p.force.y) / p.mass;
        rhoMin = std::min(rhoMin, double(p.density) / p.restDensity);
        rhoMax = std::max(rhoMax, double(p.density) / p.restDensity);
        pMin = std::min(pMin, double(p.pressure));
        pMax = std::max(pMax, double(p.pressure));
        if (x >= margin && y >= margin && x < side - margin && y < side - margin) {
            bulkMin = std::min(bulkMin, double(p.density) / p.restDensity);
            bulkMax = std::max(bulkMax, double(p.density) / p.restDensity);
            bulkAcceleration = std::max(bulkAcceleration, acceleration);
            ++bulkCount;
        } else
            surfaceAcceleration = std::max(surfaceAcceleration, acceleration);
        initialRms += std::pow(double(p.position.x) - nominal[i].position.x, 2) +
                      std::pow(double(p.position.y) - nominal[i].position.y, 2);
        totalMass += p.mass;
    }
    if (bulkCount == 0)
        throw std::runtime_error("Disorder run has no interior");
    auto system = FromParticles(particles, family);
    system.clamp = clamp;
    const double initialInternal = Build(system).internalEnergy;
    double energyMin = initialInternal, energyMax = initialInternal,
           minSeparation = Separation(particles);
    double peakSpeed = 0;
    std::uint64_t substeps = 0;
    for (int step = 0; step < steps; ++step) {
        solver.step(particles, dt);
        substeps += solver.getLastStatistics().substepCount;
        for (const auto &p : particles) {
            if (!std::isfinite(p.position.x) || !std::isfinite(p.position.y) ||
                !std::isfinite(p.velocity.x) || !std::isfinite(p.velocity.y))
                throw std::runtime_error("Nonfinite disorder trajectory");
            peakSpeed = std::max(peakSpeed, std::hypot(double(p.velocity.x), p.velocity.y));
        }
        if ((step + 1) % (steps / 8) == 0) {
            auto current = FromParticles(particles, family);
            current.clamp = clamp;
            const double energy = Kinetic(particles) + Build(current).internalEnergy;
            energyMin = std::min(energyMin, energy);
            energyMax = std::max(energyMax, energy);
            minSeparation = std::min(minSeparation, Separation(particles));
        }
    }
    double finalRms = 0, motionRms = 0, angularMomentum = 0;
    D2 momentum;
    for (std::size_t i = 0; i < particles.size(); ++i) {
        const auto &p = particles[i];
        if (p.mass != disturbed[i].mass || p.restDensity != disturbed[i].restDensity)
            throw std::runtime_error("Disorder solver changed caller mass/rho0");
        finalRms += std::pow(double(p.position.x) - nominal[i].position.x, 2) +
                    std::pow(double(p.position.y) - nominal[i].position.y, 2);
        motionRms += std::pow(double(p.position.x) - disturbed[i].position.x, 2) +
                     std::pow(double(p.position.y) - disturbed[i].position.y, 2);
        const D2 pv{double(p.mass) * p.velocity.x, double(p.mass) * p.velocity.y};
        momentum = momentum + pv;
        angularMomentum += double(p.position.x) * pv.y - double(p.position.y) * pv.x;
    }
    auto finalSystem = FromParticles(particles, family);
    finalSystem.clamp = clamp;
    const double finalInternal = Build(finalSystem).internalEnergy;
    const double finalKinetic = Kinetic(particles);
    if (label == std::string("original-caller-mass") && clamp && family != Family::WendlandC2)
        for (std::size_t i = 0; i < particles.size(); ++i)
            if (!(particles[i].position == disturbed[i].position) ||
                !(particles[i].velocity == Vector2{}))
                throw std::runtime_error("Stationary original-input control changed");
    if (!std::isfinite(finalRms + motionRms + finalInternal + finalKinetic + momentum.norm() +
                       angularMomentum))
        throw std::runtime_error("Invalid disorder report");
    out << "{\"label\":\"" << label << "\",\"family\":\"" << Name(family)
        << "\",\"clampNegativePressure\":" << (clamp ? "true" : "false") << ",\"side\":" << side
        << ",\"particleCount\":" << particles.size() << ",\"spacing\":" << dx
        << ",\"support\":" << h << ",\"hOverDx\":" << ratio << ",\"callerMassScale\":" << massScale
        << ",\"totalMass\":" << totalMass << ",\"outerSteps\":" << steps << ",\"outerDt\":" << dt
        << ",\"requestedDuration\":0.2,\"summedOuterDuration\":" << double(dt) * steps
        << ",\"actualSubsteps\":" << substeps << ",\"initialDensityRatioMin\":" << rhoMin
        << ",\"initialDensityRatioMax\":" << rhoMax << ",\"initialBulkDensityRatioMin\":" << bulkMin
        << ",\"initialBulkDensityRatioMax\":" << bulkMax << ",\"initialPressureMin\":" << pMin
        << ",\"initialPressureMax\":" << pMax
        << ",\"initialBulkMaxAcceleration\":" << bulkAcceleration
        << ",\"initialSurfaceMaxAcceleration\":" << surfaceAcceleration
        << ",\"initialRmsToLattice\":" << std::sqrt(initialRms / particles.size())
        << ",\"finalRmsToLattice\":" << std::sqrt(finalRms / particles.size())
        << ",\"rmsMotionFromDisturbedState\":" << std::sqrt(motionRms / particles.size())
        << ",\"peakSpeed\":" << peakSpeed
        << ",\"finalMinSeparationOverDx\":" << Separation(particles) / dx
        << ",\"sampledMinSeparationOverDx\":" << minSeparation / dx
        << ",\"initialEosInternalEnergy\":" << initialInternal
        << ",\"finalEosInternalEnergy\":" << finalInternal
        << ",\"finalKineticEnergy\":" << finalKinetic << ",\"sampledTotalEnergyMin\":" << energyMin
        << ",\"sampledTotalEnergyMax\":" << energyMax
        << ",\"finalTotalMomentumMagnitude\":" << momentum.norm()
        << ",\"finalAngularMomentum\":" << angularMomentum
        << ",\"originalRmsHealingTargetMet\":" << (finalRms < .81 * initialRms ? "true" : "false")
        << ",\"callerMassAndRestDensityPreserved\":true}";
}
void Work(std::ostream &out, Family family) {
    auto p = Block(9, .1f, .2f, 1);
    for (auto &v : p) {
        v.position = v.position * .96f;
        v.viscosity = 0;
        v.velocity = {float(.3 * v.position.x + .1 * std::sin(4 * v.position.y)),
                      float(-.2 * v.position.y + .07 * std::cos(3 * v.position.x))};
    }
    const auto s = FromParticles(p, family);
    const auto op = Build(s);
    out << "{\"family\":\"" << Name(family) << "\",\"mechanicalPressureWork\":" << op.mechanicalWork
        << ",\"trueSummationEosEnergyRate\":" << op.trueEnergyRate
        << ",\"pressureMapEnergyRate\":" << op.pressureMapEnergyRate
        << ",\"truePressureWorkResidual\":" << op.mechanicalWork + op.trueEnergyRate
        << ",\"formalPressureMapResidual\":" << op.mechanicalWork + op.pressureMapEnergyRate
        << ",\"energyDerivativeErrors\":[";
    bool first = true;
    for (double dt : {.001, .0005, .00025}) {
        if (!first)
            out << ',';
        first = false;
        out << std::abs(EnergyDifference(s, dt) - op.trueEnergyRate);
    }
    out << "]}";
}
} // namespace
int main(int argc, char **argv) {
    try {
        bool quick = false, includeWendland = false;
        std::string path = "fluid-disorder.json";
        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            if (arg == "--quick")
                quick = true;
            else if (arg == "--include-wendland")
                includeWendland = true;
            else if (arg == "--output" && i + 1 < argc)
                path = argv[++i];
            else
                throw std::invalid_argument(
                    "Usage: fluid_disorder_diagnostic [--quick] [--include-wendland] [--output report.json]");
        }
        std::vector<Family> families{Family::Poly6Spiky, Family::CubicSpline};
        if (includeWendland) families.push_back(Family::WendlandC2);
        std::ofstream out(path);
        if (!out)
            throw std::runtime_error("Cannot open disorder report");
        out << std::setprecision(17)
            << "{\"scope\":\"diagnostic only; no production "
               "correction\",\"fixtureBaseCommit\":\"355fe2b\",\"quick\":"
            << (quick ? "true" : "false")
            << ",\"trajectoryEnergyAndSeparationSamples\":\"initial and eight equally spaced "
               "outer-step endpoints\",\n\"bulkRows\":[\n";
        bool first = true;
        for (auto family : families)
            for (double ratio : {2., 2.5, 4., 8.})
                for (double dx : {.1, .05, .025}) {
                    const auto row = LatticeRow(dx, dx * ratio, .02, family);
                    const auto pressureRow = LatticeRow(dx, dx * ratio, .02, family, false);
                    const double densityRatio = row.shifted * OriginalMassScale;
                    const double signedPressure = Pressure(densityRatio, false);
                    const D2 bulkAcceleration =
                        pressureRow.gradientSum * (-2 * OriginalMassScale * signedPressure /
                                                   (1000 * densityRatio * densityRatio));
                    if (!first)
                        out << ",\n";
                    first = false;
                    out << "{\"family\":\"" << Name(family) << "\",\"spacing\":" << dx
                        << ",\"hOverDx\":" << ratio
                        << ",\"unshiftedNominalDensityRatio\":" << row.unshifted
                        << ",\"shiftedNominalDensityRatio\":" << row.shifted
                        << ",\"familyNormalizedShiftedDensityRatio\":"
                        << row.shifted / row.unshifted << ",\"originalMassShiftedDensityRatio\":"
                        << row.shifted * OriginalMassScale
                        << ",\"linearDensitySymbolNorm\":" << row.linearSymbol.norm()
                        << ",\"signedPressureBulkAcceleration\":" << bulkAcceleration.norm()
                        << ",\"signedPressureAccelerationAlongShift\":"
                        << (bulkAcceleration.x + bulkAcceleration.y) / std::sqrt(2.) << "}";
                }
        out << "],\n\"sameInputTrajectories\":[\n";
        first = true;
        for (auto family : families)
            for (bool clamp : {true, false})
                for (int steps :
                     quick ? std::vector<int>{96, 192} : std::vector<int>{48, 96, 192, 384}) {
                    if (!first)
                        out << ",\n";
                    first = false;
                    Run(out, "original-caller-mass", family, clamp, steps);
                }
        out << "],\n\"separateControls\":[\n";
        first = true;
        if (!quick)
            for (auto family : families) {
                for (bool clamp : {true, false}) {
                    if (!first)
                        out << ",\n";
                    first = false;
                    Run(out, "input-family-calibration-control-not-a-solver-fix", family, clamp,
                        192, 21, .1f, 2, SphKernels2D::SquareLatticeMassScale(.1f, .2f, family));
                    out << ",\n";
                    Run(out, "fixed-cell-footprint-spatial-control", family, clamp, 192, 42, .05f);
                }
                out << ",\n";
                Run(out, "original-mass-increased-neighbor-control", family, true, 192, 21, .1f, 4);
            }
        out << "],\n\"pressureWorkControls\":[\n";
        Work(out, Family::Poly6Spiky);
        out << ",\n";
        Work(out, Family::CubicSpline);
        if (includeWendland) { out << ",\n"; Work(out, Family::WendlandC2); }
        out << "]}\n";
        if (!out)
            throw std::runtime_error("Cannot write disorder report");
        std::cout << "Disorder report written: " << path
                  << "; original physical targets remain unchanged\n";
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}

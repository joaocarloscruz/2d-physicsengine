#include "fluid_consistency_metrics.h"
#include "physics/core/fluids/wcsph_solver.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>
#include <utility>

using namespace PhysicsEngine;
namespace {
// One explicit family choice applies to every solver control in this process.
SphKernelFamily selectedFamily = SphKernelFamily::Poly6Spiky;
void RequireFiniteState(const std::vector<FluidParticle>& particles) {
    for (const auto& particle : particles) {
        if (!std::isfinite(particle.position.x) || !std::isfinite(particle.position.y) ||
            !std::isfinite(particle.velocity.x) || !std::isfinite(particle.velocity.y) ||
            !std::isfinite(particle.force.x) || !std::isfinite(particle.force.y) ||
            !std::isfinite(particle.density) || particle.density <= 0 ||
            !std::isfinite(particle.pressure))
            throw std::runtime_error("Diagnostic solver produced a nonfinite or invalid state.");
    }
}
std::vector<FluidParticle> MakeBlock(int side, float spacing, float h, float massScale = 1,
    Vector2 translation = {}) {
    FluidParticleProperties properties;
    properties.restDensity = 1000;
    properties.mass = properties.restDensity * spacing * spacing * massScale;
    properties.smoothingLength = h;
    properties.viscosity = 0.05f;
    std::vector<FluidParticle> particles;
    particles.reserve(static_cast<std::size_t>(side) * side);
    for (int y = 0; y < side; ++y)
        for (int x = 0; x < side; ++x)
            particles.emplace_back(translation + Vector2{x * spacing, y * spacing}, Vector2{}, properties);
    return particles;
}
double BulkRatio(const std::vector<FluidParticle>& particles, int side, float spacing, float h) {
    const int margin = static_cast<int>(std::ceil(h / spacing)) + 1;
    if (2 * margin >= side) throw std::invalid_argument("Block has no complete-support bulk sample.");
    double sum = 0;
    std::size_t count = 0;
    for (int y = margin; y < side - margin; ++y)
        for (int x = margin; x < side - margin; ++x) {
            const auto& particle = particles[static_cast<std::size_t>(y) * side + x];
            sum += particle.density / static_cast<double>(particle.restDensity);
            ++count;
        }
    return sum / count;
}
void WriteMoments(std::ostream& out, float spacing, float h) {
    const auto m = FluidConsistency::MeasureKernelMoments(spacing, h);
    const double scale = SphKernels2D::SquareLatticeMassScale(spacing, h);
    out << "{\"spacing\":" << spacing << ",\"h\":" << h << ",\"hOverDx\":" << h / spacing
        << ",\"supportSamples\":" << m.supportSamples << ",\"poly6ZerothMoment\":" << m.density
        << ",\"spikyZerothMoment\":" << m.pressureWeight
        << ",\"poly6DerivativeFirstMomentXX\":" << m.densityGradientXX
        << ",\"spikyGradientFirstMomentXX\":" << m.pressureGradientXX
        << ",\"spikyGradientFirstMomentYY\":" << m.pressureGradientYY
        << ",\"spikyGradientFirstMomentXY\":" << m.pressureGradientXY
        << ",\"massScale\":" << scale
        << ",\"massCalibratedSpikyGradientFirstMomentXX\":" << scale * m.pressureGradientXX
        << ",\"candidateCubicZerothMoment\":" << m.candidateCubicDensity
        << ",\"candidateCubicGradientFirstMomentXX\":" << m.candidateCubicGradientXX
        << ",\"selectedDensityZerothMoment\":" << (selectedFamily == SphKernelFamily::CubicSpline ? m.cubicDensity : m.density)
        << ",\"selectedGradientFirstMomentXX\":" << (selectedFamily == SphKernelFamily::CubicSpline ? m.cubicGradientXX : m.pressureGradientXX)
        << ",\"selectedFamilyMassScale\":" << SphKernels2D::SquareLatticeMassScale(spacing,h,selectedFamily) << '}';
}
void WriteDensity(std::ostream& out, float spacing, float h) {
    const int side = 2 * static_cast<int>(std::ceil(h / spacing)) + 9;
    auto original = MakeBlock(side, spacing, h);
    auto translated = MakeBlock(side, spacing, h, 1, {0.037f, -0.061f});
    WcsphConfig config;
    config.externalAcceleration = {};
    config.kernelFamily = selectedFamily;
    WcsphSolver solver(h, config);
    solver.prepare(original);
    solver.prepare(translated);
    RequireFiniteState(original);
    RequireFiniteState(translated);
    const double originalRatio = BulkRatio(original, side, spacing, h);
    const double translatedRatio = BulkRatio(translated, side, spacing, h);
    out << "{\"spacing\":" << spacing << ",\"h\":" << h << ",\"side\":" << side
        << ",\"originalBulkDensityRatio\":" << originalRatio
        << ",\"translatedBulkDensityRatio\":" << translatedRatio
        << ",\"translationDifference\":" << translatedRatio - originalRatio << '}';
}
void WriteRest(std::ostream& out, const char* label, int side, float spacing, float h, int steps,
    float massScale = 1, WcsphDensityMode mode = WcsphDensityMode::Summation) {
    auto particles = MakeBlock(side, spacing, h, massScale);
    const auto initial = particles;
    constexpr float duration = 0.1f;
    const float dt = duration / steps;
    WcsphConfig config;
    config.externalAcceleration = {};
    config.kernelFamily = selectedFamily;
    config.speedOfSound = 15;
    config.equationOfStateExponent = 7;
    config.clampNegativePressure = true;
    config.maximumTimeStep = dt;
    config.densityMode = mode;
    WcsphSolver solver(h, config);
    solver.prepare(particles);
    RequireFiniteState(particles);
    const double bulkRatio = BulkRatio(particles, side, spacing, h);
    double initialAcceleration = 0, initialPressure = 0, peakSpeed = 0, finalSpeed = 0, displacement = 0;
    for (const auto& particle : particles) {
        initialAcceleration = std::max(initialAcceleration,
            std::hypot(static_cast<double>(particle.force.x), particle.force.y) / particle.mass);
        initialPressure = std::max(initialPressure, static_cast<double>(particle.pressure));
    }
    for (int step = 0; step < steps; ++step) {
        solver.step(particles, dt);
        RequireFiniteState(particles);
        for (const auto& particle : particles)
            peakSpeed = std::max(peakSpeed, std::hypot(static_cast<double>(particle.velocity.x), particle.velocity.y));
    }
    double momentumX = 0, momentumY = 0, totalMass = 0;
    for (std::size_t i = 0; i < particles.size(); ++i) {
        const auto& particle = particles[i];
        if (particle.mass != initial[i].mass || particle.restDensity != initial[i].restDensity)
            throw std::runtime_error("Diagnostic solver changed caller mass or rest density.");
        finalSpeed = std::max(finalSpeed, std::hypot(static_cast<double>(particle.velocity.x), particle.velocity.y));
        displacement = std::max(displacement,
            std::hypot(static_cast<double>(particle.position.x) - initial[i].position.x,
                static_cast<double>(particle.position.y) - initial[i].position.y));
        momentumX += particle.mass * static_cast<double>(particle.velocity.x);
        momentumY += particle.mass * static_cast<double>(particle.velocity.y);
        totalMass += particle.mass;
    }
    out << "{\"label\":\"" << label << "\",\"side\":" << side
        << ",\"particleCount\":" << particles.size() << ",\"spacing\":" << spacing
        << ",\"h\":" << h << ",\"outerDt\":" << dt << ",\"duration\":" << duration
        << ",\"densityMode\":\"" << (mode == WcsphDensityMode::Summation ? "summation" : "continuity")
        << "\",\"massScale\":" << massScale << ",\"totalMass\":" << totalMass
        << ",\"initialBulkDensityRatio\":" << bulkRatio << ",\"initialMaximumPressure\":" << initialPressure
        << ",\"initialMaximumAcceleration\":" << initialAcceleration
        << ",\"peakSpeed\":" << peakSpeed << ",\"finalMaximumSpeed\":" << finalSpeed
        << ",\"finalMaximumDisplacement\":" << displacement
        << ",\"totalMomentumMagnitude\":" << std::hypot(momentumX, momentumY)
        << ",\"callerMassAndRestDensityPreserved\":true}";
}

void WriteCharacterization(std::ostream& out, float ratio, bool perturb, bool clamp) {
    constexpr int side=21;
    constexpr float dx=0.1f, dt=1.0f/960;
    const float h=ratio*dx;
    auto particles=MakeBlock(side,dx,h);
    const auto lattice=particles;
    if(perturb) for(int y=0;y<side;++y) for(int x=0;x<side;++x) {
        auto& p=particles[static_cast<std::size_t>(y)*side+x];
        p.position.x += (x%2 ? -1 : 1)*0.1f*dx;
        p.position.y += (y%2 ? -1 : 1)*0.1f*dx;
    }
    const auto measure=[&]() {
        double rms=0,minimum=1e100;
        for(std::size_t i=0;i<particles.size();++i) {
            const double x=static_cast<double>(particles[i].position.x)-lattice[i].position.x;
            const double y=static_cast<double>(particles[i].position.y)-lattice[i].position.y;
            rms += x*x+y*y;
            for(std::size_t j=0;j<i;++j) minimum=std::min(minimum,std::hypot(
                static_cast<double>(particles[i].position.x)-particles[j].position.x,
                static_cast<double>(particles[i].position.y)-particles[j].position.y));
        }
        return std::pair<double,double>{std::sqrt(rms/particles.size()),minimum/dx};
    };
    const auto initial=measure();
    WcsphConfig config; config.kernelFamily=selectedFamily; config.externalAcceleration={};
    config.speedOfSound=15; config.maximumTimeStep=dt; config.clampNegativePressure=clamp;
    WcsphSolver solver(h,config);
    double peakSpeed=0;
    for(int step=0;step<48;++step) {
        solver.step(particles,dt); RequireFiniteState(particles);
        for(const auto& p:particles) peakSpeed=std::max(peakSpeed,std::hypot(static_cast<double>(p.velocity.x),p.velocity.y));
    }
    const auto final=measure();
    out << "{\"perturbed\":" << (perturb?"true":"false") << ",\"clampNegativePressure\":" << (clamp?"true":"false")
        << ",\"hOverDx\":" << ratio << ",\"spacing\":" << dx
        << ",\"duration\":0.05,\"outerDt\":" << dt
        << ",\"initialPositionRms\":" << initial.first << ",\"finalPositionRms\":" << final.first
        << ",\"initialMinimumSeparationOverDx\":" << initial.second
        << ",\"finalMinimumSeparationOverDx\":" << final.second << ",\"peakSpeed\":" << peakSpeed << '}';
}

void WriteWallCharacterization(std::ostream& out) {
    FluidBoundarySettings settings; settings.particleRadius=0.05f;
    FluidConvexPolygonContainer tank({{-1,0},{1,0},{1,2},{-1,2}},settings);
    FluidBoundarySamplingSettings sampling; sampling.spacing=0.1f; sampling.supportRadius=0.2f;
    const auto walls=SampleFluidContainerBoundary(tank,sampling);
    FluidParticleProperties properties; properties.mass=10; properties.smoothingLength=0.2f; properties.viscosity=0.05f;
    std::vector<FluidParticle> particles;
    for(int y=0;y<15;++y) for(int x=0;x<19;++x)
        particles.emplace_back(Vector2{-0.9f+x*0.1f,0.1f+y*0.1f},Vector2{},properties);
    WcsphConfig config; config.kernelFamily=selectedFamily; config.speedOfSound=40; config.maximumTimeStep=0.001f;
    WcsphSolver solver(0.2f,config); double peak=0;
    for(int step=0;step<100;++step) {
        solver.step(particles,0.001f,tank,walls,[](float){}); RequireFiniteState(particles);
        for(const auto& p:particles) peak=std::max(peak,std::hypot(static_cast<double>(p.velocity.x),p.velocity.y));
    }
    double error=0;
    for(int y=2;y<13;++y) for(int x=2;x<17;++x) {
        const auto& p=particles[static_cast<std::size_t>(y)*19+x];
        error=std::max(error,std::abs(p.density/static_cast<double>(p.restDensity)-1));
    }
    out << "{\"initialization\":\"nominal summation, not EOS hydrostatic\",\"duration\":0.1,\"outerDt\":0.001,"
        << "\"speedOfSound\":40,\"particleCount\":285,\"peakSpeed\":" << peak
        << ",\"finalBulkDensityError\":" << error << '}';
}
}

int main(int argc, char** argv) {
    try {
        bool quick = false;
        std::string outputPath;
        for (int i = 1; i < argc; ++i) {
            const std::string argument = argv[i];
            if (argument == "--quick") quick = true;
            else if (argument == "--family" && i + 1 < argc) {
                const std::string family = argv[++i];
                if (family == "cubic") selectedFamily = SphKernelFamily::CubicSpline;
                else if (family == "legacy") selectedFamily = SphKernelFamily::Poly6Spiky;
                else throw std::invalid_argument("Kernel family must be legacy or cubic.");
            }
            else if (argument == "--output" && i + 1 < argc) outputPath = argv[++i];
            else throw std::invalid_argument("Usage: fluid_consistency_diagnostic [--quick] [--family legacy|cubic] [--output report.json]");
        }
        std::ofstream file;
        if (!outputPath.empty()) {
            file.open(outputPath);
            if (!file) throw std::runtime_error("Cannot open diagnostic output.");
        }
        auto& out = file.is_open() ? static_cast<std::ostream&>(file) : std::cout;
        out << std::setprecision(12);
        out << "{\n\"schemaVersion\":1,\"quick\":" << (quick ? "true" : "false")
            << ",\"model\":\"" << (selectedFamily == SphKernelFamily::CubicSpline ? "WCSPH matched cubic density / pressure gradient" : "WCSPH poly6 density / spiky pressure gradient") << "\","
            << "\"continuumKernelNormalizationUnchanged\":true,"
            << "\"densityCheckTranslation\":[0.037,-0.061],"
            << "\"restSettings\":{\"speedOfSound\":15,\"equationOfStateExponent\":7,"
            << "\"restDensity\":1000,\"viscosity\":0.05,\"externalAcceleration\":[0,0],"
            << "\"clampNegativePressure\":true},\n\"kernelMoments\":[";
        bool first = true;
        for (float ratio : {1.25f, 1.5f, 2.0f, 2.5f, 3.0f, 4.0f, 6.0f, 8.0f, 12.0f}) {
            if (!first) out << ',';
            first = false;
            WriteMoments(out, 0.1f, ratio * 0.1f);
        }
        out << "],\n\"fixedRatioRefinement\":[";
        first = true;
        for (float spacing : {0.1f, 0.05f, 0.025f}) {
            if (!first) out << ',';
            first = false;
            WriteMoments(out, spacing, 2 * spacing);
        }
        out << "],\n\"densityTranslationChecks\":[";
        first = true;
        for (float spacing : {0.1f, 0.05f, 0.025f}) {
            if (!first) out << ',';
            first = false;
            WriteDensity(out, spacing, 0.2f);
        }
        out << "],\n\"restCases\":[";
        WriteRest(out, "canonical-nominal-dt240", 21, 0.1f, 0.2f, 24);
        out << ',';
        WriteRest(out, "canonical-mass-calibrated", 21, 0.1f, 0.2f, 48,
            SphKernels2D::SquareLatticeMassScale(0.1f, 0.2f, selectedFamily));
        out << ',';
        WriteRest(out, "canonical-initialized-continuity", 21, 0.1f, 0.2f, 48,
            1, WcsphDensityMode::Continuity);
        if (!quick) {
            out << ',';
            WriteRest(out, "canonical-nominal-dt480", 21, 0.1f, 0.2f, 48);
            out << ',';
            WriteRest(out, "canonical-nominal-dt960", 21, 0.1f, 0.2f, 96);
            // Cell-center grids share a fixed 2x2 domain and nominal total mass.
            // Translation is irrelevant in zero gravity, so positions start at
            // zero while the represented cell footprint remains side*dx=2.
            for (int side : {20, 40, 80}) {
                const float spacing = 2.0f / side;
                out << ',';
                WriteRest(out, "fixed-ratio-domain2", side, spacing, 2 * spacing, 96);
                out << ',';
                WriteRest(out, "fixed-h-domain2", side, spacing, 0.2f, 96);
            }
        }
        out << "]";
        if (!quick) {
            out << ",\n\"perturbationAndTensileControls\":[";
            bool firstControl=true;
            for(float ratio:{2.0f,4.0f,8.0f}) for(bool clamp:{true,false}) for(bool perturb:{false,true}) {
                if(!firstControl) out << ',';
                firstControl=false;
                WriteCharacterization(out,ratio,perturb,clamp);
            }
            out << "],\n\"sampledWallControl\":";
            WriteWallCharacterization(out);
        }
        out << "\n}\n";
        out.flush();
        if (!out) throw std::runtime_error("Failed to write diagnostic output.");
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}

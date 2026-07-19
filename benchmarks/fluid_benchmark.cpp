#include "physics/core/fluids/wcsph_solver.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace PhysicsEngine;

namespace {

struct RenderVertex {
    float x;
    float y;
    std::uint32_t color;
};

struct Measurement {
    std::size_t particleCount;
    double simulationMedianMs;
    double simulationP95Ms;
    double renderPreparationMedianMs;
    double renderPreparationP95Ms;
    std::size_t candidatePairs;
    std::size_t neighborPairs;
    std::uint32_t substeps;
};

double Percentile(std::vector<double> values, double percentile) {
    std::sort(values.begin(), values.end());
    const std::size_t index = static_cast<std::size_t>(std::ceil(
        percentile * static_cast<double>(values.size())
    )) - 1;
    return values[std::min(index, values.size() - 1)];
}

std::vector<FluidParticle> MakeParticles(std::size_t count) {
    constexpr float spacing = 0.1f;
    FluidParticleProperties properties;
    properties.mass = properties.restDensity * spacing * spacing;
    properties.smoothingLength = spacing * 2.0f;
    properties.viscosity = 0.05f;
    const std::size_t columns = static_cast<std::size_t>(std::ceil(
        std::sqrt(static_cast<double>(count))
    ));
    std::vector<FluidParticle> particles;
    particles.reserve(count);
    for (std::size_t index = 0; index < count; ++index) {
        const std::size_t row = index / columns;
        const std::size_t column = index % columns;
        particles.emplace_back(
            Vector2(
                static_cast<float>(column) * spacing,
                static_cast<float>(row) * spacing
            ),
            Vector2(),
            properties
        );
    }
    return particles;
}

void PrepareRenderBuffer(
    const std::vector<FluidParticle>& particles,
    std::vector<RenderVertex>& vertices
) {
    constexpr float radius = 0.04f;
    constexpr std::uint32_t color = 0x3A86FFFFu;
    vertices.clear();
    for (const FluidParticle& particle : particles) {
        const float left = particle.position.x - radius;
        const float right = particle.position.x + radius;
        const float top = particle.position.y - radius;
        const float bottom = particle.position.y + radius;
        vertices.push_back({left, top, color});
        vertices.push_back({right, top, color});
        vertices.push_back({right, bottom, color});
        vertices.push_back({left, top, color});
        vertices.push_back({right, bottom, color});
        vertices.push_back({left, bottom, color});
    }
}

Measurement RunMeasurement(std::size_t count, int warmups, int samples) {
    const std::vector<FluidParticle> baseline = MakeParticles(count);
    WcsphConfig config;
    config.externalAcceleration = Vector2();
    config.speedOfSound = 20.0f;
    config.maximumTimeStep = 1.0f / 60.0f;
    WcsphSolver solver(0.2f, config);
    constexpr float frameTime = 1.0f / 120.0f;

    for (int warmup = 0; warmup < warmups; ++warmup) {
        std::vector<FluidParticle> particles = baseline;
        solver.step(particles, frameTime);
    }

    std::vector<double> simulationTimes;
    std::vector<double> renderTimes;
    simulationTimes.reserve(samples);
    renderTimes.reserve(samples);
    std::vector<RenderVertex> vertices;
    vertices.reserve(count * 6);
    volatile float checksum = 0.0f;
    for (int sample = 0; sample < samples; ++sample) {
        std::vector<FluidParticle> particles = baseline;
        const auto simulationStart = std::chrono::steady_clock::now();
        solver.step(particles, frameTime);
        const auto simulationEnd = std::chrono::steady_clock::now();
        PrepareRenderBuffer(particles, vertices);
        const auto renderEnd = std::chrono::steady_clock::now();
        checksum = checksum + vertices.back().x;
        simulationTimes.push_back(std::chrono::duration<double, std::milli>(
            simulationEnd - simulationStart
        ).count());
        renderTimes.push_back(std::chrono::duration<double, std::milli>(
            renderEnd - simulationEnd
        ).count());
    }
    if (!std::isfinite(checksum)) {
        throw std::runtime_error("Fluid benchmark checksum became non-finite.");
    }
    const auto& statistics = solver.getLastStatistics();
    return Measurement{
        count,
        Percentile(simulationTimes, 0.5),
        Percentile(simulationTimes, 0.95),
        Percentile(renderTimes, 0.5),
        Percentile(renderTimes, 0.95),
        statistics.neighbors.candidatePairCount,
        statistics.neighbors.neighborPairCount,
        statistics.substepCount
    };
}

std::vector<std::size_t> ParseCounts(const std::string& value) {
    std::vector<std::size_t> counts;
    std::stringstream stream(value);
    std::string item;
    while (std::getline(stream, item, ',')) {
        const std::size_t count = static_cast<std::size_t>(std::stoull(item));
        if (count == 0) {
            throw std::invalid_argument("Particle counts must be positive.");
        }
        counts.push_back(count);
    }
    if (counts.empty()) {
        throw std::invalid_argument("At least one particle count is required.");
    }
    return counts;
}

std::string ToJson(const std::vector<Measurement>& measurements) {
    std::ostringstream output;
    output << std::fixed << std::setprecision(4);
    output << "{\n  \"frameTimeSeconds\": 0.0083333,\n";
    output << "  \"renderMetric\": \"CPU batched-vertex preparation; excludes GPU draw/present\",\n";
    output << "  \"measurements\": [\n";
    for (std::size_t index = 0; index < measurements.size(); ++index) {
        const Measurement& value = measurements[index];
        output << "    {\"particles\": " << value.particleCount
               << ", \"simulationMedianMs\": " << value.simulationMedianMs
               << ", \"simulationP95Ms\": " << value.simulationP95Ms
               << ", \"renderPreparationMedianMs\": "
               << value.renderPreparationMedianMs
               << ", \"renderPreparationP95Ms\": "
               << value.renderPreparationP95Ms
               << ", \"candidatePairs\": " << value.candidatePairs
               << ", \"neighborPairs\": " << value.neighborPairs
               << ", \"substeps\": " << value.substeps << "}";
        if (index + 1 != measurements.size()) {
            output << ',';
        }
        output << '\n';
    }
    output << "  ]\n}\n";
    return output.str();
}

} // namespace

int main(int argc, char** argv) {
    try {
        std::vector<std::size_t> counts = {1000, 3000, 10000};
        int warmups = 3;
        int samples = 10;
        std::string outputPath;
        for (int index = 1; index < argc; ++index) {
            const std::string argument = argv[index];
            if (argument == "--particles" && index + 1 < argc) {
                counts = ParseCounts(argv[++index]);
            } else if (argument == "--warmup" && index + 1 < argc) {
                warmups = std::stoi(argv[++index]);
            } else if (argument == "--samples" && index + 1 < argc) {
                samples = std::stoi(argv[++index]);
            } else if (argument == "--output" && index + 1 < argc) {
                outputPath = argv[++index];
            } else {
                throw std::invalid_argument("Unknown or incomplete benchmark argument.");
            }
        }
        if (warmups < 0 || samples <= 0) {
            throw std::invalid_argument(
                "Warmups must be non-negative and samples must be positive."
            );
        }
        std::vector<Measurement> measurements;
        for (std::size_t count : counts) {
            measurements.push_back(RunMeasurement(count, warmups, samples));
        }
        const std::string json = ToJson(measurements);
        std::cout << json;
        if (!outputPath.empty()) {
            std::ofstream output(outputPath);
            if (!output) {
                throw std::runtime_error("Could not open benchmark output path.");
            }
            output << json;
        }
        return 0;
    } catch (const std::exception& exception) {
        std::cerr << "Fluid benchmark failed: " << exception.what() << '\n';
        return 1;
    }
}

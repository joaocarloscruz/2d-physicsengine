#include "physics/core/collisions/collision_resolver.h"
#include "physics/core/collisions/narrow_phase/collision_circle_circle.h"
#include "physics/core/forces/gravity.h"
#include "physics/core/rigidbody.h"
#include "physics/core/shape.h"
#include "physics/core/world.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

using namespace PhysicsEngine;

namespace {

struct Measurement {
    std::string name;
    double expected;
    double actual;
    double absoluteError;
    double tolerance;
    bool passed;
};

struct ScalarRun {
    double position;
    double velocity;
    std::int64_t nanoseconds;
};

Measurement Measure(
    const std::string& name,
    double expected,
    double actual,
    double tolerance
) {
    const double error = std::abs(actual - expected);
    return Measurement{name, expected, actual, error, tolerance, error <= tolerance};
}

std::pair<double, double> SimulateFreeFall(float timeStep, int steps) {
    World world;
    Circle shape(0.5f);
    Material material{1.0f, 0.0f};
    auto body = std::make_shared<RigidBody>(
        &shape,
        material,
        Vector2(2.0f, 10.0f)
    );
    body->SetVelocity(Vector2(3.0f, 4.0f));
    world.addBody(body);
    world.addUniversalForce(std::make_unique<Gravity>(Vector2(0.0f, -9.81f)));

    for (int i = 0; i < steps; ++i) {
        world.step(timeStep);
    }
    return {body->GetPosition().y, body->GetVelocity().y};
}

double SimulateHarmonicOscillator(float timeStep, int steps) {
    Circle shape(0.5f);
    Material material{1.0f, 0.0f};
    RigidBody body(&shape, material, Vector2(1.0f, 0.0f));
    body.SetMass(1.0f);

    for (int i = 0; i < steps; ++i) {
        body.ApplyForce(Vector2(-body.GetPosition().x, 0.0f));
        body.Integrate(timeStep);
    }
    return body.GetPosition().x;
}

template<typename Scalar>
ScalarRun RunScalarIntegration(std::size_t steps) {
    const Scalar timeStep = static_cast<Scalar>(0.0001);
    const Scalar acceleration = static_cast<Scalar>(-9.81);
    Scalar position = static_cast<Scalar>(10.0);
    Scalar velocity = static_cast<Scalar>(4.0);

    const auto started = std::chrono::steady_clock::now();
    for (std::size_t i = 0; i < steps; ++i) {
        position = position + velocity * timeStep
            + acceleration * timeStep * timeStep * static_cast<Scalar>(0.5);
        velocity = velocity + acceleration * timeStep;
    }
    const auto elapsed = std::chrono::steady_clock::now() - started;

    return ScalarRun{
        static_cast<double>(position),
        static_cast<double>(velocity),
        std::chrono::duration_cast<std::chrono::nanoseconds>(elapsed).count()
    };
}

void AddAnalyticalTrajectoryMeasurements(std::vector<Measurement>& measurements) {
    constexpr float timeStep = 1.0f / 120.0f;
    constexpr int steps = 1200;
    constexpr double elapsed = 10.0;
    constexpr double initialPosition = 10.0;
    constexpr double initialVelocity = 4.0;
    constexpr double acceleration = -9.81;
    const auto [position, velocity] = SimulateFreeFall(timeStep, steps);

    measurements.push_back(Measure(
        "free_fall_position_10s",
        initialPosition + initialVelocity * elapsed
            + 0.5 * acceleration * elapsed * elapsed,
        position,
        0.01
    ));
    measurements.push_back(Measure(
        "free_fall_velocity_10s",
        initialVelocity + acceleration * elapsed,
        velocity,
        0.002
    ));
}

void AddConservationMeasurements(std::vector<Measurement>& measurements) {
    Circle shape(1.0f);
    Material material{1.0f, 1.0f, 0.0f, 0.0f};
    RigidBody first(&shape, material, Vector2(0.0f, 0.0f));
    RigidBody second(&shape, material, Vector2(1.5f, 0.0f));
    first.SetMass(2.0f);
    second.SetMass(5.0f);
    first.SetVelocity(Vector2(4.0f, 0.0f));
    second.SetVelocity(Vector2(-1.0f, 0.0f));

    const double momentumBefore = 2.0 * 4.0 + 5.0 * -1.0;
    const double energyBefore = 0.5 * 2.0 * 16.0 + 0.5 * 5.0;
    CollisionResolver::Resolve(CollisionCircleCircle(&first, &second));
    const double momentumAfter = 2.0 * first.GetVelocity().x
        + 5.0 * second.GetVelocity().x;
    const double energyAfter = 0.5 * 2.0 * first.GetVelocity().magnitudeSquared()
        + 0.5 * 5.0 * second.GetVelocity().magnitudeSquared();

    measurements.push_back(Measure(
        "elastic_collision_momentum",
        momentumBefore,
        momentumAfter,
        1e-4
    ));
    measurements.push_back(Measure(
        "elastic_collision_energy",
        energyBefore,
        energyAfter,
        1e-3
    ));
}

void AddConvergenceMeasurements(std::vector<Measurement>& measurements) {
    constexpr double elapsed = 2.0;
    constexpr double expected = 10.0 + 4.0 * elapsed - 0.5 * 9.81 * elapsed * elapsed;
    const double coarse = SimulateFreeFall(1.0f / 30.0f, 60).first;
    const double medium = SimulateFreeFall(1.0f / 60.0f, 120).first;
    const double fine = SimulateFreeFall(1.0f / 240.0f, 480).first;
    const double coarseError = std::abs(coarse - expected);
    const double mediumError = std::abs(medium - expected);
    const double fineError = std::abs(fine - expected);

    measurements.push_back(Measure(
        "free_fall_dt_1_over_30_error",
        0.0,
        coarseError,
        0.001
    ));
    measurements.push_back(Measure(
        "free_fall_dt_1_over_60_error",
        0.0,
        mediumError,
        0.001
    ));
    measurements.push_back(Measure(
        "free_fall_dt_1_over_240_error",
        0.0,
        fineError,
        0.001
    ));
    measurements.push_back(Measure(
        "free_fall_roundoff_fine_minus_coarse_error",
        0.0,
        std::max(0.0, fineError - coarseError),
        0.001
    ));

    constexpr double oscillatorElapsed = 10.0;
    const double oscillatorExpected = std::cos(oscillatorElapsed);
    const double oscillatorCoarseError = std::abs(
        SimulateHarmonicOscillator(0.1f, 100) - oscillatorExpected
    );
    const double oscillatorMediumError = std::abs(
        SimulateHarmonicOscillator(0.05f, 200) - oscillatorExpected
    );
    const double oscillatorFineError = std::abs(
        SimulateHarmonicOscillator(0.025f, 400) - oscillatorExpected
    );
    measurements.push_back(Measure(
        "oscillator_dt_0_1_error",
        0.0,
        oscillatorCoarseError,
        0.25
    ));
    measurements.push_back(Measure(
        "oscillator_dt_0_05_error",
        0.0,
        oscillatorMediumError,
        0.125
    ));
    measurements.push_back(Measure(
        "oscillator_dt_0_025_error",
        0.0,
        oscillatorFineError,
        0.0625
    ));
    measurements.push_back(Measure(
        "oscillator_fine_to_coarse_error_ratio",
        0.0,
        oscillatorFineError / oscillatorCoarseError,
        0.5
    ));
}

void AddDriftMeasurement(
    std::vector<Measurement>& measurements,
    std::size_t steps
) {
    Circle shape(0.5f);
    Material material{1.0f, 0.0f};
    RigidBody body(&shape, material, Vector2(1.0f, -2.0f));
    body.SetVelocity(Vector2(0.125f, -0.25f));
    constexpr float timeStep = 1.0f / 1000.0f;

    for (std::size_t i = 0; i < steps; ++i) {
        body.Integrate(timeStep);
    }

    const double expected = 1.0 + 0.125 * static_cast<double>(timeStep) * steps;
    const double tolerance = steps <= 100000 ? 0.05 : 1.5;
    measurements.push_back(Measure(
        "free_motion_long_duration_position",
        expected,
        body.GetPosition().x,
        tolerance
    ));
}

void WriteJson(
    std::ostream& output,
    const std::string& mode,
    const std::vector<Measurement>& measurements,
    const ScalarRun& floatRun,
    const ScalarRun& doubleRun,
    bool passed
) {
    output << std::setprecision(17);
    output << "{\n";
    output << "  \"mode\": \"" << mode << "\",\n";
    output << "  \"engineScalar\": \"float\",\n";
    output << "  \"passed\": " << (passed ? "true" : "false") << ",\n";
    output << "  \"measurements\": [\n";
    for (std::size_t i = 0; i < measurements.size(); ++i) {
        const Measurement& value = measurements[i];
        output << "    {\"name\": \"" << value.name
            << "\", \"expected\": " << value.expected
            << ", \"actual\": " << value.actual
            << ", \"absoluteError\": " << value.absoluteError
            << ", \"tolerance\": " << value.tolerance
            << ", \"passed\": " << (value.passed ? "true" : "false")
            << "}" << (i + 1 == measurements.size() ? "\n" : ",\n");
    }
    output << "  ],\n";
    output << "  \"precisionComparison\": {\n";
    output << "    \"floatPosition\": " << floatRun.position << ",\n";
    output << "    \"doublePosition\": " << doubleRun.position << ",\n";
    output << "    \"absolutePositionDifference\": "
        << std::abs(floatRun.position - doubleRun.position) << ",\n";
    output << "    \"floatNanoseconds\": " << floatRun.nanoseconds << ",\n";
    output << "    \"doubleNanoseconds\": " << doubleRun.nanoseconds << ",\n";
    output << "    \"floatToDoubleTimeRatio\": "
        << (doubleRun.nanoseconds == 0
            ? 0.0
            : static_cast<double>(floatRun.nanoseconds) / doubleRun.nanoseconds)
        << "\n";
    output << "  }\n";
    output << "}\n";
}

} // namespace

int main(int argc, char** argv) {
    std::string mode = "fast";
    std::string outputPath;
    for (int i = 1; i < argc; ++i) {
        const std::string argument = argv[i];
        if (argument == "--mode" && i + 1 < argc) {
            mode = argv[++i];
        } else if (argument == "--output" && i + 1 < argc) {
            outputPath = argv[++i];
        } else {
            std::cerr << "Unknown or incomplete argument: " << argument << '\n';
            return 2;
        }
    }
    if (mode != "fast" && mode != "extended") {
        std::cerr << "Mode must be 'fast' or 'extended'.\n";
        return 2;
    }

    std::vector<Measurement> measurements;
    AddAnalyticalTrajectoryMeasurements(measurements);
    AddConservationMeasurements(measurements);
    AddConvergenceMeasurements(measurements);
    AddDriftMeasurement(measurements, mode == "fast" ? 100000 : 1000000);

    const std::size_t scalarSteps = mode == "fast" ? 1000000 : 10000000;
    const ScalarRun floatRun = RunScalarIntegration<float>(scalarSteps);
    const ScalarRun doubleRun = RunScalarIntegration<double>(scalarSteps);
    const double scalarRelativeDifference = std::abs(
        floatRun.position - doubleRun.position
    ) / std::max(1.0, std::abs(doubleRun.position));
    measurements.push_back(Measure(
        "float_vs_double_relative_position_difference",
        0.0,
        scalarRelativeDifference,
        mode == "fast" ? 0.005 : 0.05
    ));

    bool passed = true;
    for (const Measurement& measurement : measurements) {
        passed = passed && measurement.passed;
    }

    WriteJson(std::cout, mode, measurements, floatRun, doubleRun, passed);
    if (!outputPath.empty()) {
        std::ofstream output(outputPath);
        if (!output) {
            throw std::runtime_error("Could not open numerical validation output file.");
        }
        WriteJson(output, mode, measurements, floatRun, doubleRun, passed);
    }
    return passed ? 0 : 1;
}

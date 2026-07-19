#include "catch_amalgamated.hpp"

#include "physics/core/fluids/sph_kernels.h"

#include <cmath>
#include <limits>

using namespace PhysicsEngine;

namespace {

constexpr float Pi = 3.14159265358979323846f;

template<typename Kernel>
float IntegrateRadially(float supportRadius, Kernel kernel) {
    constexpr int Samples = 20000;
    const float width = supportRadius / static_cast<float>(Samples);
    double integral = 0.0;
    for (int sample = 0; sample < Samples; ++sample) {
        const float radius = (static_cast<float>(sample) + 0.5f) * width;
        integral += static_cast<double>(kernel(radius))
            * 2.0 * static_cast<double>(Pi) * radius * width;
    }
    return static_cast<float>(integral);
}

} // namespace

TEST_CASE("2D SPH scalar kernels are numerically normalized", "[fluid][sph][kernel][normalization]") {
    for (float smoothingLength : {0.25f, 0.7f, 2.0f}) {
        const float densityIntegral = IntegrateRadially(
            smoothingLength,
            [smoothingLength](float radius) {
                return SphKernels2D::DensityWeight(
                    Vector2(radius, 0.0f),
                    smoothingLength
                );
            }
        );
        const float pressureIntegral = IntegrateRadially(
            smoothingLength,
            [smoothingLength](float radius) {
                return SphKernels2D::PressureWeight(
                    Vector2(radius, 0.0f),
                    smoothingLength
                );
            }
        );

        REQUIRE(densityIntegral == Catch::Approx(1.0f).margin(0.0002f));
        REQUIRE(pressureIntegral == Catch::Approx(1.0f).margin(0.0002f));
    }
}

TEST_CASE("2D SPH kernels enforce compact support", "[fluid][sph][kernel][support]") {
    constexpr float smoothingLength = 0.8f;
    for (float radius : {smoothingLength, 1.0f, 100.0f}) {
        const Vector2 displacement(radius, 0.0f);
        REQUIRE(SphKernels2D::DensityWeight(displacement, smoothingLength) == 0.0f);
        REQUIRE(SphKernels2D::PressureWeight(displacement, smoothingLength) == 0.0f);
        REQUIRE(SphKernels2D::PressureGradient(displacement, smoothingLength) == Vector2());
        REQUIRE(SphKernels2D::ViscosityLaplacian(displacement, smoothingLength) == 0.0f);
    }
}

TEST_CASE("2D SPH kernels have the required radial symmetry", "[fluid][sph][kernel][symmetry]") {
    const Vector2 displacement(0.23f, -0.31f);
    const Vector2 reversed = displacement * -1.0f;
    constexpr float smoothingLength = 0.9f;

    REQUIRE(
        SphKernels2D::DensityWeight(displacement, smoothingLength)
        == Catch::Approx(SphKernels2D::DensityWeight(reversed, smoothingLength))
    );
    REQUIRE(
        SphKernels2D::PressureWeight(displacement, smoothingLength)
        == Catch::Approx(SphKernels2D::PressureWeight(reversed, smoothingLength))
    );
    const Vector2 gradient = SphKernels2D::PressureGradient(
        displacement,
        smoothingLength
    );
    const Vector2 reversedGradient = SphKernels2D::PressureGradient(
        reversed,
        smoothingLength
    );
    REQUIRE(gradient.x == Catch::Approx(-reversedGradient.x));
    REQUIRE(gradient.y == Catch::Approx(-reversedGradient.y));
    REQUIRE(
        SphKernels2D::ViscosityLaplacian(displacement, smoothingLength)
        == Catch::Approx(SphKernels2D::ViscosityLaplacian(reversed, smoothingLength))
    );
}

TEST_CASE("Pressure gradient is the derivative of the normalized spiky weight", "[fluid][sph][kernel][gradient]") {
    constexpr float smoothingLength = 1.1f;
    constexpr float radius = 0.4f;
    constexpr float epsilon = 0.0001f;
    const float numericalDerivative = (
        SphKernels2D::PressureWeight(
            Vector2(radius + epsilon, 0.0f),
            smoothingLength
        )
        - SphKernels2D::PressureWeight(
            Vector2(radius - epsilon, 0.0f),
            smoothingLength
        )
    ) / (2.0f * epsilon);

    REQUIRE(
        SphKernels2D::PressureGradient(
            Vector2(radius, 0.0f),
            smoothingLength
        ).x == Catch::Approx(numericalDerivative).epsilon(0.001f)
    );
}

TEST_CASE("SPH kernels remain finite for coincident particles", "[fluid][sph][kernel][degenerate]") {
    constexpr float smoothingLength = 0.5f;
    const Vector2 zero;

    REQUIRE(std::isfinite(SphKernels2D::DensityWeight(zero, smoothingLength)));
    REQUIRE(std::isfinite(SphKernels2D::PressureWeight(zero, smoothingLength)));
    const Vector2 gradient = SphKernels2D::PressureGradient(zero, smoothingLength);
    REQUIRE(gradient == Vector2());
    REQUIRE(std::isfinite(SphKernels2D::ViscosityLaplacian(zero, smoothingLength)));
    REQUIRE(SphKernels2D::ViscosityLaplacian(zero, smoothingLength) > 0.0f);
}

TEST_CASE("SPH kernels reject invalid numerical parameters", "[fluid][sph][kernel][validation]") {
    REQUIRE_THROWS_AS(
        SphKernels2D::DensityWeight(Vector2(), 0.0f),
        std::invalid_argument
    );
    REQUIRE_THROWS_AS(
        SphKernels2D::PressureGradient(
            Vector2(std::numeric_limits<float>::quiet_NaN(), 0.0f),
            1.0f
        ),
        std::invalid_argument
    );
}

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

TEST_CASE("Square lattice mass scale corrects discrete kernel quadrature", "[fluid][sph][kernel][lattice]") {
    constexpr float spacing = 0.1f;
    for (const float ratio : {1.5f, 2.0f, 2.5f}) {
        const float smoothingLength = ratio * spacing;
        double densityRatio = 0.0;
        const int extent = static_cast<int>(std::ceil(ratio));
        for (int y = -extent; y <= extent; ++y) {
            for (int x = -extent; x <= extent; ++x) {
                densityRatio += spacing * spacing
                    * SphKernels2D::DensityWeight(
                        Vector2(x * spacing, y * spacing),
                        smoothingLength
                    );
            }
        }
        const float scale = SphKernels2D::SquareLatticeMassScale(
            spacing, smoothingLength
        );
        REQUIRE(static_cast<float>(densityRatio) * scale
            == Catch::Approx(1.0f).margin(2e-6f));
    }
    REQUIRE_THROWS_AS(
        SphKernels2D::SquareLatticeMassScale(0.0f, 0.2f),
        std::invalid_argument
    );
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

TEST_CASE("Scalar SPH weights retain their scale law for large finite support", "[fluid][sph][kernel][scale]") {
    constexpr double pi = 3.14159265358979323846;
    const float h = 1e20f;
    const Vector2 displacement(h * 0.5f, 0);
    const double area = static_cast<double>(h) * h;
    REQUIRE(SphKernels2D::DensityWeight(displacement, h) * area ==
        Catch::Approx(4 / pi * std::pow(0.75, 3)).margin(2e-5));
    REQUIRE(SphKernels2D::PressureWeight(displacement, h) * area ==
        Catch::Approx(10 / pi * std::pow(0.5, 3)).margin(2e-5));
    REQUIRE(SphKernels2D::DensityWeight({h, 0}, h) == 0);
}

TEST_CASE("Pressure gradients check final components and retain tiny directions", "[fluid][sph][kernel][scale]") {
    constexpr double pi = 3.14159265358979323846;
    const float h = 1e-10f;
    for (const Vector2 displacement : {Vector2(h * 0.3f, h * 0.4f), Vector2(1e-30f, 0)}) {
        const double radius = std::hypot(static_cast<double>(displacement.x), displacement.y);
        const double radialDerivative = -30 / pi * std::pow(1 - radius / h, 2) /
            (static_cast<double>(h) * h * h);
        const auto gradient = SphKernels2D::PressureGradient(displacement, h);
        REQUIRE(gradient.x == Catch::Approx(radialDerivative * displacement.x / radius));
        REQUIRE(gradient.y == Catch::Approx(radialDerivative * displacement.y / radius));
    }
}

TEST_CASE("Lattice calibration depends on spacing ratio rather than physical scale", "[fluid][sph][kernel][scale]") {
    const float expected = SphKernels2D::SquareLatticeMassScale(1, 2);
    for (const float spacing : {1e-30f, 1e-15f, 1.0f, 1e15f, 1e30f}) {
        REQUIRE(SphKernels2D::SquareLatticeMassScale(spacing, spacing * 2) == Catch::Approx(expected));
    }
}

TEST_CASE("SPH scale handling still rejects unrepresentable final values", "[fluid][sph][kernel][scale]") {
    const float h = 1e-20f;
    REQUIRE_THROWS_AS(SphKernels2D::DensityWeight({}, h), std::overflow_error);
    REQUIRE_THROWS_AS(SphKernels2D::PressureWeight({}, h), std::overflow_error);
    REQUIRE_THROWS_AS(SphKernels2D::PressureGradient({h / 2, 0}, h), std::overflow_error);
    REQUIRE_THROWS_AS(SphKernels2D::ViscosityLaplacian({}, h), std::overflow_error);
}

TEST_CASE("Cubic family matches independently normalized analytic weight and derivative", "[fluid][sph][kernel][cubic]") {
    constexpr auto family = SphKernelFamily::CubicSpline;
    constexpr double pi = 3.14159265358979323846;
    for (float h : {0.2f, 1.0f, 2.0f}) {
        const double c = 40 / (7*pi*static_cast<double>(h)*h);
        const float integral = IntegrateRadially(h, [h](float r) {
            return SphKernels2D::DensityWeight({r, 0}, h, SphKernelFamily::CubicSpline);
        });
        REQUIRE(integral == Catch::Approx(1).margin(0.0002));
        for (float ratio : {0.0f, 0.2f, 0.49f, 0.5f, 0.7f, 0.99f, 1.0f, 1.1f}) {
            const float r = ratio*h;
            const double u = 2*static_cast<double>(r)/h;
            const double expected = u < 1 ? c*(1-1.5*u*u+0.75*u*u*u) :
                (u < 2 ? c*0.25*std::pow(2-u,3) : 0);
            const double derivative = u < 1 ? c*(-3*u+2.25*u*u)*2/h :
                (u < 2 ? c*(-0.75)*std::pow(2-u,2)*2/h : 0);
            const float weight = SphKernels2D::DensityWeight({r,0},h,family);
            REQUIRE(weight == Catch::Approx(expected).margin(1e-7));
            REQUIRE(SphKernels2D::PressureWeight({r,0},h,family) == weight);
            REQUIRE(SphKernels2D::PressureGradient({r,0},h,family).x ==
                Catch::Approx(derivative).margin(1e-7));
            if (ratio > 0 && ratio < 1) {
                const float delta = h*0.0002f;
                const double numerical = (SphKernels2D::DensityWeight({r+delta,0},h,family)
                    - SphKernels2D::DensityWeight({r-delta,0},h,family))/(2*delta);
                REQUIRE(numerical == Catch::Approx(derivative).epsilon(0.001));
            }
        }
        for (float branch : {0.5f, 1.0f}) {
            const auto left = SphKernels2D::PressureGradient({h*(branch-1e-5f),0},h,family);
            const auto right = SphKernels2D::PressureGradient({h*(branch+1e-5f),0},h,family);
            REQUIRE(left.x/c*h == Catch::Approx(right.x/c*h).margin(0.0002));
            REQUIRE(SphKernels2D::DensityWeight({h*(branch-1e-5f),0},h,family)/c ==
                Catch::Approx(SphKernels2D::DensityWeight({h*(branch+1e-5f),0},h,family)/c).margin(0.0001));
        }
    }
}

TEST_CASE("Cubic parity scale laws calibration and final representability are checked", "[fluid][sph][kernel][cubic]") {
    constexpr auto family = SphKernelFamily::CubicSpline;
    const Vector2 d{0.3f, -0.4f};
    REQUIRE(SphKernels2D::DensityWeight(d,1,family) == SphKernels2D::DensityWeight(d*-1,1,family));
    REQUIRE(SphKernels2D::PressureGradient(d,1,family) == SphKernels2D::PressureGradient(d*-1,1,family)*-1);
    const double baseWeight=SphKernels2D::DensityWeight(d,1,family);
    const auto baseGradient=SphKernels2D::PressureGradient(d,1,family);
    for(float scale : {1e-10f,0.25f,2.0f,1e10f}) {
        const double area=static_cast<double>(scale)*scale;
        const auto gradient=SphKernels2D::PressureGradient(d*scale,scale,family);
        REQUIRE(SphKernels2D::DensityWeight(d*scale,scale,family)*area == Catch::Approx(baseWeight));
        REQUIRE(gradient.x*area*scale == Catch::Approx(baseGradient.x));
        REQUIRE(gradient.y*area*scale == Catch::Approx(baseGradient.y));
    }
    const double pi=3.14159265358979323846, diag=2-std::sqrt(2.0);
    const double sum=10/(7*pi)*(2+diag*diag*diag);
    REQUIRE(SphKernels2D::SquareLatticeMassScale(1,2,family) == Catch::Approx(1/sum));
    for(float dx : {1e-30f,1e-15f,1.0f,1e15f,1e30f})
        REQUIRE(SphKernels2D::SquareLatticeMassScale(dx,2*dx,family) == Catch::Approx(1/sum));
    REQUIRE(SphKernels2D::PressureGradient({},1e-20f,family) == Vector2{});
    REQUIRE_THROWS_AS(SphKernels2D::DensityWeight({},1e-20f,family),std::overflow_error);
    REQUIRE_THROWS_AS(SphKernels2D::PressureGradient({0.5e-20f,0},1e-20f,family),std::overflow_error);
    const auto tinyDirection=SphKernels2D::PressureGradient({1e-30f,0},1e-10f,family);
    REQUIRE(tinyDirection.x == Catch::Approx(-480/(7*pi)*1e10).epsilon(1e-6));
    REQUIRE(tinyDirection.y == 0);
    const auto invalid=static_cast<SphKernelFamily>(99);
    REQUIRE_THROWS_AS(SphKernels2D::DensityWeight({2,0},1,invalid),std::invalid_argument);
    REQUIRE_THROWS_AS(SphKernels2D::PressureGradient({},1,invalid),std::invalid_argument);
    REQUIRE_THROWS_AS(SphKernels2D::SquareLatticeMassScale(1,2,invalid),std::invalid_argument);
    REQUIRE_THROWS_AS(SphKernels2D::PressureWeight({},0,family),std::invalid_argument);
}

TEST_CASE("Explicit legacy family preserves all two-argument kernel results", "[fluid][sph][kernel][cubic]") {
    for(Vector2 d : {Vector2{},Vector2{0.2f,0.3f},Vector2{1,0}}) {
        REQUIRE(SphKernels2D::DensityWeight(d,1,SphKernelFamily::Poly6Spiky) == SphKernels2D::DensityWeight(d,1));
        REQUIRE(SphKernels2D::PressureWeight(d,1,SphKernelFamily::Poly6Spiky) == SphKernels2D::PressureWeight(d,1));
        REQUIRE(SphKernels2D::PressureGradient(d,1,SphKernelFamily::Poly6Spiky) == SphKernels2D::PressureGradient(d,1));
    }
    REQUIRE(SphKernels2D::SquareLatticeMassScale(1,2,SphKernelFamily::Poly6Spiky) == SphKernels2D::SquareLatticeMassScale(1,2));
}

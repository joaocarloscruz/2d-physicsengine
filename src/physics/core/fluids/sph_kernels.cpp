#include "physics/core/fluids/sph_kernels.h"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace PhysicsEngine {
namespace {

constexpr double Pi = 3.141592653589793238462643383279502884;

void ValidateArguments(
    const Vector2& displacement,
    float smoothingLength
) {
    if (!std::isfinite(smoothingLength) || smoothingLength <= 0.0f) {
        throw std::invalid_argument(
            "SPH smoothing length must be positive and finite."
        );
    }
    if (!std::isfinite(displacement.x) || !std::isfinite(displacement.y)) {
        throw std::invalid_argument("SPH displacement must be finite.");
    }
}

float CheckedFloat(double value) {
    if (!std::isfinite(value)
        || std::abs(value) > std::numeric_limits<float>::max()) {
        throw std::overflow_error("SPH kernel result exceeds float range.");
    }
    return static_cast<float>(value);
}

} // namespace

float SphKernels2D::DensityWeight(
    const Vector2& displacement,
    float smoothingLength
) {
    ValidateArguments(displacement, smoothingLength);
    const double h = smoothingLength;
    const double radiusSquared = displacement.magnitudeSquared();
    const double hSquared = h * h;
    if (radiusSquared >= hSquared) {
        return 0.0f;
    }
    const double difference = hSquared - radiusSquared;
    const double normalization = 4.0 / (Pi * std::pow(h, 8.0));
    return CheckedFloat(normalization * difference * difference * difference);
}

float SphKernels2D::PressureWeight(
    const Vector2& displacement,
    float smoothingLength
) {
    ValidateArguments(displacement, smoothingLength);
    const double h = smoothingLength;
    const double radius = std::sqrt(
        static_cast<double>(displacement.magnitudeSquared())
    );
    if (radius >= h) {
        return 0.0f;
    }
    const double difference = h - radius;
    const double normalization = 10.0 / (Pi * std::pow(h, 5.0));
    return CheckedFloat(normalization * difference * difference * difference);
}

Vector2 SphKernels2D::PressureGradient(
    const Vector2& displacement,
    float smoothingLength
) {
    ValidateArguments(displacement, smoothingLength);
    const double radiusSquared = displacement.magnitudeSquared();
    if (radiusSquared <= 0.0) {
        return Vector2();
    }
    const double h = smoothingLength;
    const double radius = std::sqrt(radiusSquared);
    if (radius >= h) {
        return Vector2();
    }
    const double difference = h - radius;
    const double radialDerivative = -30.0
        / (Pi * std::pow(h, 5.0)) * difference * difference;
    const float scale = CheckedFloat(radialDerivative / radius);
    return displacement * scale;
}

float SphKernels2D::ViscosityLaplacian(
    const Vector2& displacement,
    float smoothingLength
) {
    ValidateArguments(displacement, smoothingLength);
    const double h = smoothingLength;
    const double radius = std::sqrt(
        static_cast<double>(displacement.magnitudeSquared())
    );
    if (radius >= h) {
        return 0.0f;
    }
    return CheckedFloat(
        40.0 / (Pi * std::pow(h, 5.0)) * (h - radius)
    );
}

}

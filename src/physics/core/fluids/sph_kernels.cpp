#include "physics/core/fluids/sph_kernels.h"

#include "../checked_grid.h"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

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

// Dimensionless cubic shape and derivative with respect to q=r/h.
std::pair<double, double> CubicShape(double q) {
    const double u = 2 * q;
    if (u >= 2) return {0, 0};
    if (u < 1) return {1 - 1.5*u*u + 0.75*u*u*u, -6*u + 4.5*u*u};
    const double remainder = 2-u;
    return {0.25*remainder*remainder*remainder, -1.5*remainder*remainder};
}

} // namespace

void SphKernels2D::ValidateFamily(SphKernelFamily family) {
    if (family != SphKernelFamily::Poly6Spiky && family != SphKernelFamily::CubicSpline)
        throw std::invalid_argument("SPH kernel family is not recognized.");
}

float SphKernels2D::SquareLatticeMassScale(
    float spacing,
    float smoothingLength
) {
    return SquareLatticeMassScale(spacing, smoothingLength, SphKernelFamily::Poly6Spiky);
}

float SphKernels2D::SquareLatticeMassScale(
    float spacing,
    float smoothingLength,
    SphKernelFamily family
) {
    ValidateFamily(family);
    if (!std::isfinite(spacing) || spacing <= 0.0f
        || !std::isfinite(smoothingLength) || smoothingLength <= 0.0f) {
        throw std::invalid_argument(
            "SPH lattice spacing and smoothing length must be positive and finite."
        );
    }
    const int extent = CheckedGrid::Extent(smoothingLength, spacing);
    std::uint64_t remainingSamples = CheckedGrid::MaximumSamples;
    const std::int64_t limit = extent;
    CheckedGrid::Charge(CheckedGrid::Window{-limit, limit, -limit, limit}, remainingSamples);
    const double ratio = static_cast<double>(spacing) / smoothingLength;
    double discreteDensityRatio = 0.0;
    for (std::int64_t y = -static_cast<std::int64_t>(extent); y <= extent; ++y) {
        for (std::int64_t x = -static_cast<std::int64_t>(extent); x <= extent; ++x) {
            const double qx = x * ratio, qy = y * ratio;
            const double qSquared = qx * qx + qy * qy;
            if (qSquared < 1.0) {
                const double difference = 1.0 - qSquared;
                if (family == SphKernelFamily::Poly6Spiky)
                    discreteDensityRatio += ratio * ratio * (4.0 / Pi) *
                        difference * difference * difference;
                else
                    discreteDensityRatio += ratio * ratio * (40.0 / (7*Pi)) *
                        CubicShape(std::sqrt(qSquared)).first;
            }
        }
    }
    const float result = CheckedFloat(1.0 / discreteDensityRatio);
    if (result <= 0) throw std::overflow_error("SPH lattice calibration underflows float range.");
    return result;
}

float SphKernels2D::DensityWeight(
    const Vector2& displacement,
    float smoothingLength
) {
    ValidateArguments(displacement, smoothingLength);
    const double h = smoothingLength;
    const double radiusSquared = static_cast<double>(displacement.x) * displacement.x
        + static_cast<double>(displacement.y) * displacement.y;
    const double hSquared = h * h;
    if (radiusSquared >= hSquared) {
        return 0.0f;
    }
    const double difference = 1.0 - radiusSquared / hSquared;
    const double normalization = 4.0 / (Pi * hSquared);
    return CheckedFloat(normalization * difference * difference * difference);
}

float SphKernels2D::PressureWeight(
    const Vector2& displacement,
    float smoothingLength
) {
    ValidateArguments(displacement, smoothingLength);
    const double h = smoothingLength;
    const double radius = std::hypot(static_cast<double>(displacement.x), displacement.y);
    if (radius >= h) {
        return 0.0f;
    }
    const double difference = 1.0 - radius / h;
    const double normalization = 10.0 / (Pi * h * h);
    return CheckedFloat(normalization * difference * difference * difference);
}

Vector2 SphKernels2D::PressureGradient(
    const Vector2& displacement,
    float smoothingLength
) {
    ValidateArguments(displacement, smoothingLength);
    const double radius = std::hypot(static_cast<double>(displacement.x), displacement.y);
    if (radius == 0.0) {
        return Vector2();
    }
    const double h = smoothingLength;
    if (radius >= h) {
        return Vector2();
    }
    const double difference = 1.0 - radius / h;
    const double radialDerivative = -30.0
        / (Pi * h * h * h) * difference * difference;
    return {CheckedFloat(radialDerivative * (displacement.x / radius)),
            CheckedFloat(radialDerivative * (displacement.y / radius))};
}


float SphKernels2D::DensityWeight(const Vector2& displacement, float smoothingLength,
    SphKernelFamily family) {
    ValidateFamily(family);
    if (family == SphKernelFamily::Poly6Spiky) return DensityWeight(displacement, smoothingLength);
    ValidateArguments(displacement, smoothingLength);
    const double h = smoothingLength;
    const double radius = std::hypot(static_cast<double>(displacement.x), displacement.y);
    if (radius >= h) return 0;
    return CheckedFloat((40.0/(7*Pi)) * CubicShape(radius/h).first / (h*h));
}

float SphKernels2D::PressureWeight(const Vector2& displacement, float smoothingLength,
    SphKernelFamily family) {
    ValidateFamily(family);
    if (family == SphKernelFamily::Poly6Spiky) return PressureWeight(displacement, smoothingLength);
    return DensityWeight(displacement, smoothingLength, family);
}

Vector2 SphKernels2D::PressureGradient(const Vector2& displacement, float smoothingLength,
    SphKernelFamily family) {
    ValidateFamily(family);
    if (family == SphKernelFamily::Poly6Spiky) return PressureGradient(displacement, smoothingLength);
    ValidateArguments(displacement, smoothingLength);
    const double h = smoothingLength;
    const double radius = std::hypot(static_cast<double>(displacement.x), displacement.y);
    if (radius == 0 || radius >= h) return {};
    const double derivative = (40.0/(7*Pi)) * CubicShape(radius/h).second / (h*h*h);
    return {CheckedFloat(derivative * (displacement.x/radius)),
        CheckedFloat(derivative * (displacement.y/radius))};
}

float SphKernels2D::ViscosityLaplacian(
    const Vector2& displacement,
    float smoothingLength
) {
    ValidateArguments(displacement, smoothingLength);
    const double h = smoothingLength;
    const double radius = std::hypot(static_cast<double>(displacement.x), displacement.y);
    if (radius >= h) {
        return 0.0f;
    }
    return CheckedFloat(
        40.0 / (Pi * h * h * h * h) * (1.0 - radius / h)
    );
}

}

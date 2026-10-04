#pragma once

#include "physics/core/fluids/sph_kernels.h"
#include <cmath>
#include <cstddef>
#include <stdexcept>

namespace FluidConsistency {
struct KernelMoments {
    double density = 0;
    double pressureWeight = 0;
    double pressureGradientXX = 0;
    double pressureGradientYY = 0;
    double pressureGradientXY = 0;
    double densityGradientXX = 0;
    double pressureGradientSumX = 0;
    double pressureGradientSumY = 0;
    double candidateCubicDensity = 0;
    double candidateCubicGradientXX = 0;
    std::size_t supportSamples = 0;
};

struct CandidateKernelValue {
    double weight;
    double radialDerivative;
};
// Diagnostic-only normalized 2D cubic B-spline, with full support radius h
// (the conventional smoothing scale is h/2). No production kernel is changed.
inline CandidateKernelValue CubicSplineCandidate(double radius, double h) {
    if (!std::isfinite(radius) || radius < 0 || !std::isfinite(h) || h <= 0)
        throw std::invalid_argument("Candidate kernel arguments must be finite and valid.");
    const double u = 2 * radius / h;
    if (u >= 2) return {0, 0};
    constexpr double pi = 3.141592653589793238462643383279502884;
    const double normalization = 40 / (7 * pi * h * h);
    if (u < 1)
        return {normalization * (1 - 1.5 * u * u + 0.75 * u * u * u),
            normalization * (-3 * u + 2.25 * u * u) * 2 / h};
    const double remainder = 2 - u;
    return {normalization * 0.25 * remainder * remainder * remainder,
        normalization * -0.75 * remainder * remainder * 2 / h};
}

// Square lattice volume is dx^2. The negative first gradient moment should be
// the identity for an exact linear scalar-gradient operator. Kernels retain
// their existing continuum normalization; this helper does not correct them.
inline KernelMoments MeasureKernelMoments(float spacing, float smoothingLength) {
    if (!std::isfinite(spacing) || spacing <= 0 ||
        !std::isfinite(smoothingLength) || smoothingLength <= 0)
        throw std::invalid_argument("Diagnostic lattice parameters must be positive and finite.");
    const double ratio = static_cast<double>(smoothingLength) / spacing;
    if (ratio > 64) throw std::invalid_argument("Diagnostic h/dx is limited to 64.");
    const int extent = static_cast<int>(std::ceil(ratio));
    const double volume = static_cast<double>(spacing) * spacing;
    const double h = smoothingLength;
    constexpr double pi = 3.141592653589793238462643383279502884;
    KernelMoments result;
    for (int y = -extent; y <= extent; ++y) {
        for (int x = -extent; x <= extent; ++x) {
            const PhysicsEngine::Vector2 displacement{x * spacing, y * spacing};
            const double radiusSquared = static_cast<double>(displacement.x) * displacement.x
                + static_cast<double>(displacement.y) * displacement.y;
            if (radiusSquared >= h * h) continue;
            ++result.supportSamples;
            result.density += volume * PhysicsEngine::SphKernels2D::DensityWeight(displacement, smoothingLength);
            result.pressureWeight += volume * PhysicsEngine::SphKernels2D::PressureWeight(displacement, smoothingLength);
            const auto gradient = PhysicsEngine::SphKernels2D::PressureGradient(displacement, smoothingLength);
            result.pressureGradientXX -= volume * displacement.x * gradient.x;
            result.pressureGradientYY -= volume * displacement.y * gradient.y;
            result.pressureGradientXY -= volume * displacement.x * gradient.y;
            result.pressureGradientSumX += volume * gradient.x;
            result.pressureGradientSumY += volume * gradient.y;
            // Analytic derivative of the actual poly6 density weight, solely
            // for comparison. No solver uses this diagnostic derivative.
            const double q = 1 - radiusSquared / (h * h);
            const double derivativeX = -24 / (pi * h * h * h * h) * q * q * displacement.x;
            result.densityGradientXX -= volume * displacement.x * derivativeX;
            const double radius = std::sqrt(radiusSquared);
            const auto cubic = CubicSplineCandidate(radius, h);
            result.candidateCubicDensity += volume * cubic.weight;
            if (radius != 0)
                result.candidateCubicGradientXX -= volume * displacement.x * displacement.x
                    / radius * cubic.radialDerivative;
        }
    }
    return result;
}
}

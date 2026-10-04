#include "physics/core/fluids/wcsph_solver.h"

#include "physics/core/fluids/sph_kernels.h"
#include "sph_viscosity.h"

#include "../checked_grid.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_map>

namespace PhysicsEngine {
namespace {

void RequireFinite(float value, const char* message) {
    if (!std::isfinite(value)) {
        throw std::runtime_error(message);
    }
}

void ValidateParticleState(const FluidParticle& particle) {
    FluidParticleProperties{particle.mass, particle.restDensity,
        particle.smoothingLength, particle.viscosity}.Validate();
    if (!std::isfinite(particle.mass) || particle.mass <= 0.0f
        || !std::isfinite(particle.restDensity)
        || particle.restDensity <= 0.0f
        || !std::isfinite(particle.smoothingLength)
        || particle.smoothingLength <= 0.0f
        || !std::isfinite(particle.viscosity)
        || particle.viscosity < 0.0f
        || !std::isfinite(particle.density)
        || particle.density <= 0.0f
        || !std::isfinite(particle.position.x)
        || !std::isfinite(particle.position.y)
        || !std::isfinite(particle.velocity.x)
        || !std::isfinite(particle.velocity.y)) {
        throw std::invalid_argument(
            "WCSPH particles must contain valid finite state."
        );
    }
}

using BoundaryCell = std::pair<int, int>;

struct BoundaryCellHash {
    std::size_t operator()(const BoundaryCell& key) const {
        const std::size_t xHash = std::hash<int>{}(key.first);
        const std::size_t yHash = std::hash<int>{}(key.second);
        return xHash ^ (yHash + 0x9e3779b9u + (xHash << 6) + (xHash >> 2));
    }
};

BoundaryCell GetBoundaryCell(const Vector2& position, float cellSize) {
    return {
        CheckedGrid::Coordinate(position.x, cellSize),
        CheckedGrid::Coordinate(position.y, cellSize)
    };
}

void ValidateBoundaryParticle(const FluidBoundaryParticle& particle) {
    if (!std::isfinite(particle.position.x)
        || !std::isfinite(particle.position.y)
        || !std::isfinite(particle.velocity.x)
        || !std::isfinite(particle.velocity.y)
        || !std::isfinite(particle.volume) || particle.volume <= 0.0f
        || !std::isfinite(particle.acceleration.x)
        || !std::isfinite(particle.acceleration.y)
        || !std::isfinite(particle.pressureScale) || particle.pressureScale < 0.0f) {
        throw std::invalid_argument(
            "WCSPH boundary particles must contain valid finite state."
        );
    }
}

} // namespace

void WcsphConfig::Validate() const {
    SphKernels2D::ValidateFamily(kernelFamily);
    if (!std::isfinite(externalAcceleration.x)
        || !std::isfinite(externalAcceleration.y)) {
        throw std::invalid_argument(
            "WCSPH external acceleration must be finite."
        );
    }
    if (!std::isfinite(speedOfSound) || speedOfSound <= 0.0f) {
        throw std::invalid_argument(
            "WCSPH speed of sound must be positive and finite."
        );
    }
    if (!std::isfinite(equationOfStateExponent)
        || equationOfStateExponent <= 1.0f) {
        throw std::invalid_argument(
            "WCSPH equation-of-state exponent must be finite and greater than one."
        );
    }
    if (!std::isfinite(cflFactor) || cflFactor <= 0.0f || cflFactor > 1.0f) {
        throw std::invalid_argument(
            "WCSPH CFL factor must be finite and between zero and one."
        );
    }
    if (!std::isfinite(maximumTimeStep) || maximumTimeStep <= 0.0f) {
        throw std::invalid_argument(
            "WCSPH maximum timestep must be positive and finite."
        );
    }
    if (maximumSubsteps <= 0) {
        throw std::invalid_argument(
            "WCSPH maximum substeps must be positive."
        );
    }
    if (densityMode != WcsphDensityMode::Summation
        && densityMode != WcsphDensityMode::Continuity) {
        throw std::invalid_argument(
            "WCSPH density mode is not recognized."
        );
    }
    if (!std::isfinite(densityDiffusion)
        || densityDiffusion < 0.0f || densityDiffusion > 1.0f) {
        throw std::invalid_argument(
            "WCSPH density diffusion must be finite and between zero and one."
        );
    }
}

WcsphSolver::WcsphSolver(
    float referenceSmoothingLength,
    const WcsphConfig& solverConfig
) : config(solverConfig),
    grid(referenceSmoothingLength) {
    config.Validate();
}

void WcsphSolver::prepare(std::vector<FluidParticle>& particles) {
    lastStatistics.substepCount = 0;
    lastStatistics.boundaryCorrectionCount = 0;
    lastStatistics.maximumBoundaryPenetration = 0.0f;
    prepareState(particles, nullptr);
}

void WcsphSolver::prepare(
    std::vector<FluidParticle>& particles,
    const std::vector<FluidBoundaryParticle>& boundaryParticles
) {
    lastStatistics.substepCount = 0;
    lastStatistics.boundaryCorrectionCount = 0;
    lastStatistics.maximumBoundaryPenetration = 0.0f;
    prepareState(particles, &boundaryParticles);
}

void WcsphSolver::prepareState(
    std::vector<FluidParticle>& particles,
    const std::vector<FluidBoundaryParticle>* boundaryParticles
) {
    struct FluidBoundaryPair {
        std::size_t fluid;
        std::size_t boundary;
    };
    std::vector<FluidBoundaryPair> boundaryPairs;
    // Validate coordinates and work before publishing a rebuilt neighbor grid or
    // changing particle state. Boundary inputs are checked even for an empty fluid.
    const float cellSize = grid.getCellSize();
    float interactionRadius = 0.0f;
    for (const auto& particle : particles) {
        ValidateParticleState(particle);
        GetBoundaryCell(particle.position, cellSize);
        interactionRadius = std::max(interactionRadius, particle.smoothingLength);
    }
    if (boundaryParticles) {
        for (const auto& particle : *boundaryParticles) {
            ValidateBoundaryParticle(particle);
            GetBoundaryCell(particle.position, cellSize);
        }
    }
    if (!particles.empty()) {
        const int extent = CheckedGrid::Extent(interactionRadius, cellSize);
        std::uint64_t remainingVisits = CheckedGrid::MaximumGridVisits;
        for (const auto& particle : particles) {
            CheckedGrid::Charge(CheckedGrid::Around(
                GetBoundaryCell(particle.position, cellSize), extent
            ), remainingVisits);
        }
    }
    lastStatistics.boundaryParticleCount = boundaryParticles
        ? boundaryParticles->size()
        : 0;
    lastStatistics.boundaryCandidateCount = 0;
    if (particles.empty()) {
        diagnostics = {};
        lastStatistics.neighbors = FluidNeighborStatistics{};
        lastStatistics.minimumDensity = 0.0f;
        lastStatistics.maximumDensity = 0.0f;
        lastStatistics.maximumSpeed = 0.0f;
        lastStatistics.stableTimeStep = config.maximumTimeStep;
        return;
    }

    for (FluidParticle& particle : particles) {
        particle.inverseMass = 1.0f / particle.mass;
    }
    grid.rebuild(particles);
    const auto pairs = grid.findNeighborPairs(particles, interactionRadius);

    if (config.densityMode == WcsphDensityMode::Summation) {
        for (FluidParticle& particle : particles) {
            particle.density = particle.mass * SphKernels2D::DensityWeight(
                Vector2(),
                particle.smoothingLength,
                config.kernelFamily
            );
        }
    }
    if (boundaryParticles && !boundaryParticles->empty()) {
        std::unordered_map<
            BoundaryCell,
            std::vector<std::size_t>,
            BoundaryCellHash
        > boundaryCells;
        boundaryCells.reserve(boundaryParticles->size());
        for (std::size_t index = 0; index < boundaryParticles->size(); ++index) {
            const FluidBoundaryParticle& boundaryParticle = (*boundaryParticles)[index];
            boundaryCells[GetBoundaryCell(boundaryParticle.position, cellSize)]
                .push_back(index);
        }
        std::uint64_t remainingVisits = CheckedGrid::MaximumGridVisits;
        for (std::size_t particleIndex = 0;
             particleIndex < particles.size();
             ++particleIndex) {
            FluidParticle& particle = particles[particleIndex];
            const int cellRange = CheckedGrid::Extent(particle.smoothingLength, cellSize);
            const auto window = CheckedGrid::Around(
                GetBoundaryCell(particle.position, cellSize), cellRange
            );
            CheckedGrid::Charge(window, remainingVisits);
            for (std::int64_t x = window.minX; x <= window.maxX; ++x) {
                for (std::int64_t y = window.minY; y <= window.maxY; ++y) {
                    const auto cell = boundaryCells.find({static_cast<int>(x), static_cast<int>(y)});
                    if (cell == boundaryCells.end()) {
                        continue;
                    }
                    for (std::size_t boundaryIndex : cell->second) {
                        ++lastStatistics.boundaryCandidateCount;
                        const FluidBoundaryParticle& boundaryParticle =
                            (*boundaryParticles)[boundaryIndex];
                        const Vector2 displacement = particle.position
                            - boundaryParticle.position;
                        const float weight = SphKernels2D::DensityWeight(
                            displacement,
                            particle.smoothingLength,
                            config.kernelFamily
                        );
                        if (config.densityMode == WcsphDensityMode::Summation) {
                            particle.density += particle.restDensity
                                * boundaryParticle.volume * weight;
                        }
                        if (weight > 0.0f) {
                            boundaryPairs.push_back({
                                particleIndex,
                                boundaryIndex
                            });
                        }
                    }
                }
            }
        }
    }
    if (config.densityMode == WcsphDensityMode::Summation) {
        for (const auto& pair : pairs) {
            FluidParticle& first = particles[pair.first];
            FluidParticle& second = particles[pair.second];
            const Vector2 displacement = first.position - second.position;
            first.density += second.mass * SphKernels2D::DensityWeight(
                displacement,
                first.smoothingLength,
                config.kernelFamily
            );
            second.density += first.mass * SphKernels2D::DensityWeight(
                displacement,
                second.smoothingLength,
                config.kernelFamily
            );
        }
    }

    const auto updateThermodynamicState = [this, &particles]() {
        lastStatistics.minimumDensity = std::numeric_limits<float>::max();
        lastStatistics.maximumDensity = 0.0f;
        lastStatistics.maximumSpeed = 0.0f;
        for (FluidParticle& particle : particles) {
            RequireFinite(particle.density, "WCSPH density became non-finite.");
            const float densityRatio = particle.density / particle.restDensity;
            const double pressureScale = static_cast<double>(particle.restDensity)
                * config.speedOfSound * config.speedOfSound
                / config.equationOfStateExponent;
            double pressure = pressureScale * (
                std::pow(
                    static_cast<double>(densityRatio),
                    static_cast<double>(config.equationOfStateExponent)
                ) - 1.0
            );
            if (config.clampNegativePressure) {
                pressure = std::max(pressure, 0.0);
            }
            if (!std::isfinite(pressure)
                || std::abs(pressure) > std::numeric_limits<float>::max()) {
                throw std::runtime_error("WCSPH pressure exceeded float range.");
            }
            particle.pressure = static_cast<float>(pressure);
            particle.volume = particle.mass / particle.density;
            particle.force = config.externalAcceleration * particle.mass;
            lastStatistics.minimumDensity = std::min(
                lastStatistics.minimumDensity,
                particle.density
            );
            lastStatistics.maximumDensity = std::max(
                lastStatistics.maximumDensity,
                particle.density
            );
            lastStatistics.maximumSpeed = std::max(
                lastStatistics.maximumSpeed,
                particle.velocity.magnitude()
            );
        }
    };

    std::vector<double> wallPressureNumerator;
    std::vector<double> wallPressureDenominator;
    const auto extrapolateWallPressure = [
        this,
        &particles,
        boundaryParticles,
        &boundaryPairs,
        &wallPressureNumerator,
        &wallPressureDenominator
    ]() {
        wallPressureNumerator.assign(boundaryParticles->size(), 0.0);
        wallPressureDenominator.assign(boundaryParticles->size(), 0.0);
        for (const FluidBoundaryPair& pair : boundaryPairs) {
            const FluidParticle& particle = particles[pair.fluid];
            const FluidBoundaryParticle& boundaryParticle =
                (*boundaryParticles)[pair.boundary];
            if (boundaryParticle.pressureScale <= 0.0f) {
                continue;
            }
            const Vector2 displacement = particle.position
                - boundaryParticle.position;
            const float distance = displacement.magnitude();
            if (distance <= 0.0f) {
                continue;
            }
            const Vector2 direction = displacement / distance;
            const float normalExternalAcceleration = std::max(
                (config.externalAcceleration - boundaryParticle.acceleration)
                    .dot(direction * -1.0f),
                0.0f
            );
            const float weight = SphKernels2D::DensityWeight(
                displacement,
                particle.smoothingLength,
                config.kernelFamily
            );
            const double extrapolatedPressure = particle.pressure
                + particle.density * distance * normalExternalAcceleration;
            wallPressureNumerator[pair.boundary] += extrapolatedPressure * weight;
            wallPressureDenominator[pair.boundary] += weight;
        }
    };

    updateThermodynamicState();
    if (boundaryParticles && !boundaryPairs.empty()) {
        extrapolateWallPressure();
    }

    for (FluidParticle& particle : particles) {
        particle.densityRate = 0.0f;
    }

    std::vector<float> wallDensityRates(particles.size(), 0.0f);
    std::vector<double> viscosityRows(particles.size(), 0.0);
    for (const auto& pair : pairs) {
        FluidParticle& first = particles[pair.first];
        FluidParticle& second = particles[pair.second];
        const Vector2 displacement = first.position - second.position;
        const float smoothingLength = static_cast<float>(0.5 * (
            static_cast<double>(first.smoothingLength) + second.smoothingLength
        ));
        const Vector2 gradient = SphKernels2D::PressureGradient(
            displacement,
            smoothingLength,
            config.kernelFamily
        );
        const double pressureTerm = first.pressure
                / (static_cast<double>(first.density) * first.density)
            + second.pressure / (static_cast<double>(second.density) * second.density);
        const double pressureScale = -static_cast<double>(first.mass) * second.mass * pressureTerm;
        const Vector2 pressureForce = SphViscosity::CheckedVector(
            gradient.x * pressureScale, gradient.y * pressureScale
        );

        const float laplacian = SphKernels2D::ViscosityLaplacian(displacement, smoothingLength);
        const double viscosityScale = SphViscosity::Coupling(first, second, laplacian);
        viscosityRows[pair.first] += viscosityScale / first.mass;
        viscosityRows[pair.second] += viscosityScale / second.mass;
        const Vector2 pairForce = SphViscosity::Add(pressureForce,
            SphViscosity::PairForce(first, second, viscosityScale));
        first.force = SphViscosity::Add(first.force, pairForce);
        second.force = SphViscosity::Add(second.force, pairForce, -1.0);
        if (config.densityMode == WcsphDensityMode::Continuity) {
            const float compressionRate = (
                first.velocity - second.velocity
            ).dot(gradient);
            first.densityRate += second.mass * compressionRate;
            second.densityRate += first.mass * compressionRate;
            if (config.densityDiffusion > 0.0f) {
                const float distanceSquared = displacement.magnitudeSquared();
                const float regularization = 0.01f
                    * smoothingLength * smoothingLength;
                const float averageDensity = 0.5f
                    * (first.density + second.density);
                const float hydrostaticDensityDifference = averageDensity
                    * config.externalAcceleration.dot(displacement)
                    / (config.speedOfSound * config.speedOfSound);
                const float dynamicDensityDifference = first.density
                    - second.density - hydrostaticDensityDifference;
                const float diffusion = 2.0f * config.densityDiffusion
                    * smoothingLength * config.speedOfSound
                    * dynamicDensityDifference
                    * displacement.dot(gradient)
                    / (distanceSquared + regularization);
                first.densityRate += second.mass / second.density * diffusion;
                second.densityRate -= first.mass / first.density * diffusion;
            }
        }
    }

    if (boundaryParticles) {
        for (const FluidBoundaryPair& pair : boundaryPairs) {
            FluidParticle& particle = particles[pair.fluid];
            const FluidBoundaryParticle& boundaryParticle =
                (*boundaryParticles)[pair.boundary];
            const Vector2 displacement = particle.position
                - boundaryParticle.position;
            const float distance = displacement.magnitude();
            if (distance <= 0.0f) {
                continue;
            }
            const double denominator = wallPressureDenominator[pair.boundary];
            const float wallPressure = denominator > 0.0
                ? static_cast<float>(
                    wallPressureNumerator[pair.boundary] / denominator
                )
                : particle.pressure;
            const Vector2 pressureGradient = SphKernels2D::PressureGradient(
                displacement,
                particle.smoothingLength,
                config.kernelFamily
            );
            if (boundaryParticle.pressureScale > 0.0f) {
                const float pressureScale = -particle.volume
                    * boundaryParticle.volume
                    * (particle.pressure + wallPressure)
                    * boundaryParticle.pressureScale;
                particle.force = particle.force
                    + pressureGradient * pressureScale;
            }
            if (boundaryParticle.pressureScale > 0.0f) {
                const Vector2 direction = displacement / distance;
                const Vector2 relativeVelocity = particle.velocity
                    - boundaryParticle.velocity;
                const Vector2 normalRelativeVelocity = direction
                    * relativeVelocity.dot(direction);
                const float wallRate = particle.density * boundaryParticle.volume * 2.0f
                    * normalRelativeVelocity.dot(pressureGradient);
                wallDensityRates[pair.fluid] += wallRate;
                if (config.densityMode == WcsphDensityMode::Continuity)
                    particle.densityRate += wallRate;
            }
        }
    }

    lastStatistics.neighbors = grid.getLastStatistics();
    diagnostics = MeasureFluidDiagnostics(particles, pairs, config.kernelFamily, wallDensityRates);
    lastStatistics.stableTimeStep = SphViscosity::TimeStep(std::min(
        static_cast<double>(getStableTimeStep(particles)), SphViscosity::RowLimit(viscosityRows)
    ));
}

float WcsphSolver::getStableTimeStep(
    const std::vector<FluidParticle>& particles
) const {
    if (particles.empty()) {
        return config.maximumTimeStep;
    }
    float minimumSmoothingLength = std::numeric_limits<float>::max();
    double maximumSpeed = 0.0;
    double viscosityLimit = std::numeric_limits<double>::infinity();
    double maximumAcceleration = 0.0;
    for (const FluidParticle& particle : particles) {
        ValidateParticleState(particle);
        RequireFinite(particle.force.x, "WCSPH timestep requires finite particle force.");
        RequireFinite(particle.force.y, "WCSPH timestep requires finite particle force.");
        minimumSmoothingLength = std::min(
            minimumSmoothingLength,
            particle.smoothingLength
        );
        maximumSpeed = std::max(
            maximumSpeed,
            std::hypot(static_cast<double>(particle.velocity.x), particle.velocity.y)
        );
        viscosityLimit = std::min(viscosityLimit, SphViscosity::ContinuumLimit(particle));
        maximumAcceleration = std::max(
            maximumAcceleration,
            std::hypot(static_cast<double>(particle.force.x), particle.force.y) / particle.mass
        );
    }
    const double acousticLimit = static_cast<double>(config.cflFactor) * minimumSmoothingLength
        / (config.speedOfSound + maximumSpeed);
    double stableTimeStep = std::min(static_cast<double>(config.maximumTimeStep), acousticLimit);
    stableTimeStep = std::min(stableTimeStep, viscosityLimit);
    if (maximumAcceleration > 0.0) {
        const double forceLimit = config.cflFactor * std::sqrt(minimumSmoothingLength / maximumAcceleration);
        stableTimeStep = std::min(stableTimeStep, forceLimit);
    }
    return SphViscosity::TimeStep(stableTimeStep);
}

void WcsphSolver::integrate(
    std::vector<FluidParticle>& particles,
    float deltaTime
) {
    for (FluidParticle& particle : particles) {
        particle.velocity = SphViscosity::AdvanceVelocity(particle, deltaTime);
        particle.position = SphViscosity::Add(particle.position, particle.velocity, deltaTime);
        if (config.densityMode == WcsphDensityMode::Continuity) {
            particle.density += particle.densityRate * deltaTime;
        }
        if (!std::isfinite(particle.position.x)
            || !std::isfinite(particle.position.y)
            || !std::isfinite(particle.velocity.x)
            || !std::isfinite(particle.velocity.y)
            || !std::isfinite(particle.density)
            || particle.density <= 0.0f) {
            throw std::runtime_error("WCSPH integration became non-finite.");
        }
    }
}

void WcsphSolver::step(
    std::vector<FluidParticle>& particles,
    float deltaTime
) {
    stepInternal(particles, deltaTime, nullptr, nullptr, nullptr);
}

void WcsphSolver::step(
    std::vector<FluidParticle>& particles,
    float deltaTime,
    const IFluidContainer& boundary
) {
    stepInternal(particles, deltaTime, &boundary, nullptr, nullptr);
}

void WcsphSolver::step(
    std::vector<FluidParticle>& particles,
    float deltaTime,
    const SubstepCallback& afterSubstep
) {
    stepInternal(particles, deltaTime, nullptr, nullptr, &afterSubstep);
}

void WcsphSolver::step(
    std::vector<FluidParticle>& particles,
    float deltaTime,
    const std::vector<FluidBoundaryParticle>& boundaryParticles,
    const SubstepCallback& afterSubstep
) {
    stepInternal(
        particles,
        deltaTime,
        nullptr,
        &boundaryParticles,
        &afterSubstep
    );
}

void WcsphSolver::step(
    std::vector<FluidParticle>& particles,
    float deltaTime,
    const IFluidContainer& boundary,
    const SubstepCallback& afterSubstep
) {
    stepInternal(particles, deltaTime, &boundary, nullptr, &afterSubstep);
}

void WcsphSolver::step(
    std::vector<FluidParticle>& particles,
    float deltaTime,
    const IFluidContainer& boundary,
    const std::vector<FluidBoundaryParticle>& boundaryParticles,
    const SubstepCallback& afterSubstep
) {
    stepInternal(
        particles,
        deltaTime,
        &boundary,
        &boundaryParticles,
        &afterSubstep
    );
}

void WcsphSolver::stepInternal(
    std::vector<FluidParticle>& particles,
    float deltaTime,
    const IFluidContainer* boundary,
    const std::vector<FluidBoundaryParticle>* boundaryParticles,
    const SubstepCallback* afterSubstep
) {
    if (!std::isfinite(deltaTime) || deltaTime < 0.0f) {
        throw std::invalid_argument(
            "WCSPH delta time must be finite and non-negative."
        );
    }
    if (deltaTime == 0.0f) {
        if (boundaryParticles) {
            prepare(particles, *boundaryParticles);
        } else {
            prepare(particles);
        }
        return;
    }

    lastStatistics.boundaryCorrectionCount = 0;
    lastStatistics.maximumBoundaryPenetration = 0.0f;
    float remaining = deltaTime;
    std::uint32_t substeps = 0;
    while (remaining > 0.0f) {
        if (substeps >= static_cast<std::uint32_t>(config.maximumSubsteps)) {
            throw std::runtime_error(
                "WCSPH exceeded the configured maximum substeps."
            );
        }
        prepareState(particles, boundaryParticles);
        const float substep = std::min(
            remaining,
            lastStatistics.stableTimeStep
        );
        integrate(particles, substep);
        if (boundary) {
            const FluidBoundaryStatistics boundaryStatistics =
                EnforceFluidBoundary(*boundary, particles);
            lastStatistics.boundaryCorrectionCount +=
                boundaryStatistics.correctedParticleCount;
            lastStatistics.maximumBoundaryPenetration = std::max(
                lastStatistics.maximumBoundaryPenetration,
                boundaryStatistics.maximumPenetration
            );
        }
        if (afterSubstep && *afterSubstep) {
            (*afterSubstep)(substep);
        }
        remaining -= substep;
        if (remaining < deltaTime * 1e-6f) {
            remaining = 0.0f;
        }
        ++substeps;
    }
    prepareState(particles, boundaryParticles);
    lastStatistics.substepCount = substeps;
    diagnostics.substeps = substeps;
}

const WcsphConfig& WcsphSolver::getConfig() const {
    return config;
}

const WcsphStatistics& WcsphSolver::getLastStatistics() const {
    return lastStatistics;
}

}

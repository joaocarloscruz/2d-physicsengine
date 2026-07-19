#include "physics/core/fluids/wcsph_solver.h"

#include "physics/core/fluids/sph_kernels.h"

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
    if (!std::isfinite(particle.mass) || particle.mass <= 0.0f
        || !std::isfinite(particle.restDensity)
        || particle.restDensity <= 0.0f
        || !std::isfinite(particle.smoothingLength)
        || particle.smoothingLength <= 0.0f
        || !std::isfinite(particle.viscosity)
        || particle.viscosity < 0.0f
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
        static_cast<int>(std::floor(position.x / cellSize)),
        static_cast<int>(std::floor(position.y / cellSize))
    };
}

} // namespace

void WcsphConfig::Validate() const {
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
    lastStatistics.boundaryParticleCount = boundaryParticles
        ? boundaryParticles->size()
        : 0;
    lastStatistics.boundaryCandidateCount = 0;
    if (particles.empty()) {
        lastStatistics.neighbors = FluidNeighborStatistics{};
        lastStatistics.minimumDensity = 0.0f;
        lastStatistics.maximumDensity = 0.0f;
        lastStatistics.maximumSpeed = 0.0f;
        lastStatistics.stableTimeStep = config.maximumTimeStep;
        return;
    }

    float interactionRadius = 0.0f;
    for (const FluidParticle& particle : particles) {
        ValidateParticleState(particle);
        interactionRadius = std::max(
            interactionRadius,
            particle.smoothingLength
        );
    }
    grid.rebuild(particles);
    const auto pairs = grid.findNeighborPairs(particles, interactionRadius);

    for (FluidParticle& particle : particles) {
        particle.density = particle.mass * SphKernels2D::DensityWeight(
            Vector2(),
            particle.smoothingLength
        );
    }
    if (boundaryParticles && !boundaryParticles->empty()) {
        const float cellSize = grid.getCellSize();
        std::unordered_map<
            BoundaryCell,
            std::vector<std::size_t>,
            BoundaryCellHash
        > boundaryCells;
        boundaryCells.reserve(boundaryParticles->size());
        for (std::size_t index = 0; index < boundaryParticles->size(); ++index) {
            const FluidBoundaryParticle& boundaryParticle = (*boundaryParticles)[index];
            if (!std::isfinite(boundaryParticle.position.x)
                || !std::isfinite(boundaryParticle.position.y)
                || !std::isfinite(boundaryParticle.velocity.x)
                || !std::isfinite(boundaryParticle.velocity.y)
                || !std::isfinite(boundaryParticle.volume)
                || boundaryParticle.volume <= 0.0f) {
                throw std::invalid_argument(
                    "WCSPH boundary particles must contain valid finite state."
                );
            }
            boundaryCells[GetBoundaryCell(boundaryParticle.position, cellSize)]
                .push_back(index);
        }
        for (FluidParticle& particle : particles) {
            const int cellRange = static_cast<int>(std::ceil(
                particle.smoothingLength / cellSize
            ));
            const BoundaryCell origin = GetBoundaryCell(particle.position, cellSize);
            for (int x = origin.first - cellRange;
                 x <= origin.first + cellRange;
                 ++x) {
                for (int y = origin.second - cellRange;
                     y <= origin.second + cellRange;
                     ++y) {
                    const auto cell = boundaryCells.find({x, y});
                    if (cell == boundaryCells.end()) {
                        continue;
                    }
                    for (std::size_t boundaryIndex : cell->second) {
                        ++lastStatistics.boundaryCandidateCount;
                        const FluidBoundaryParticle& boundaryParticle =
                            (*boundaryParticles)[boundaryIndex];
                        particle.density += particle.restDensity
                            * boundaryParticle.volume
                            * SphKernels2D::DensityWeight(
                                particle.position - boundaryParticle.position,
                                particle.smoothingLength
                            );
                    }
                }
            }
        }
    }
    for (const auto& pair : pairs) {
        FluidParticle& first = particles[pair.first];
        FluidParticle& second = particles[pair.second];
        const Vector2 displacement = first.position - second.position;
        first.density += second.mass * SphKernels2D::DensityWeight(
            displacement,
            first.smoothingLength
        );
        second.density += first.mass * SphKernels2D::DensityWeight(
            displacement,
            second.smoothingLength
        );
    }

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

    for (const auto& pair : pairs) {
        FluidParticle& first = particles[pair.first];
        FluidParticle& second = particles[pair.second];
        const Vector2 displacement = first.position - second.position;
        const float smoothingLength = 0.5f * (
            first.smoothingLength + second.smoothingLength
        );
        const Vector2 gradient = SphKernels2D::PressureGradient(
            displacement,
            smoothingLength
        );
        const float pressureTerm = first.pressure
                / (first.density * first.density)
            + second.pressure / (second.density * second.density);
        const Vector2 pressureForce = gradient * (
            -first.mass * second.mass * pressureTerm
        );

        const float viscosity = 0.5f * (
            first.viscosity + second.viscosity
        );
        const float laplacian = SphKernels2D::ViscosityLaplacian(
            displacement,
            smoothingLength
        );
        const float viscosityScale = viscosity * first.mass * second.mass
            / (first.density * second.density) * laplacian;
        const Vector2 viscosityForce = (
            second.velocity - first.velocity
        ) * viscosityScale;
        const Vector2 pairForce = pressureForce + viscosityForce;
        if (!std::isfinite(pairForce.x) || !std::isfinite(pairForce.y)) {
            throw std::runtime_error("WCSPH pair force became non-finite.");
        }
        first.force = first.force + pairForce;
        second.force = second.force - pairForce;
    }

    lastStatistics.neighbors = grid.getLastStatistics();
    lastStatistics.stableTimeStep = getStableTimeStep(particles);
}

float WcsphSolver::getStableTimeStep(
    const std::vector<FluidParticle>& particles
) const {
    if (particles.empty()) {
        return config.maximumTimeStep;
    }
    float minimumSmoothingLength = std::numeric_limits<float>::max();
    float maximumSpeed = 0.0f;
    float maximumViscosity = 0.0f;
    float maximumAcceleration = 0.0f;
    for (const FluidParticle& particle : particles) {
        ValidateParticleState(particle);
        minimumSmoothingLength = std::min(
            minimumSmoothingLength,
            particle.smoothingLength
        );
        maximumSpeed = std::max(
            maximumSpeed,
            particle.velocity.magnitude()
        );
        maximumViscosity = std::max(
            maximumViscosity,
            particle.viscosity
        );
        maximumAcceleration = std::max(
            maximumAcceleration,
            particle.force.magnitude() * particle.inverseMass
        );
    }
    const float acousticLimit = config.cflFactor * minimumSmoothingLength
        / (config.speedOfSound + maximumSpeed);
    float stableTimeStep = std::min(config.maximumTimeStep, acousticLimit);
    if (maximumViscosity > 0.0f) {
        const float viscosityLimit = 0.125f
            * minimumSmoothingLength * minimumSmoothingLength
            / maximumViscosity;
        stableTimeStep = std::min(stableTimeStep, viscosityLimit);
    }
    if (maximumAcceleration > 0.0f) {
        const float forceLimit = config.cflFactor * std::sqrt(
            minimumSmoothingLength / maximumAcceleration
        );
        stableTimeStep = std::min(stableTimeStep, forceLimit);
    }
    if (!std::isfinite(stableTimeStep) || stableTimeStep <= 0.0f) {
        throw std::runtime_error("WCSPH stable timestep is not positive and finite.");
    }
    return stableTimeStep;
}

void WcsphSolver::integrate(
    std::vector<FluidParticle>& particles,
    float deltaTime
) {
    for (FluidParticle& particle : particles) {
        const Vector2 acceleration = particle.force * particle.inverseMass;
        particle.velocity = particle.velocity + acceleration * deltaTime;
        particle.position = particle.position + particle.velocity * deltaTime;
        if (!std::isfinite(particle.position.x)
            || !std::isfinite(particle.position.y)
            || !std::isfinite(particle.velocity.x)
            || !std::isfinite(particle.velocity.y)) {
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
}

const WcsphConfig& WcsphSolver::getConfig() const {
    return config;
}

const WcsphStatistics& WcsphSolver::getLastStatistics() const {
    return lastStatistics;
}

}

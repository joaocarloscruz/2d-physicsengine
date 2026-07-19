#include "physics/core/fluids/coupled_fluid_simulation.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <unordered_set>

namespace PhysicsEngine {
namespace {

void ValidateBodies(const std::vector<RigidBody*>& bodies) {
    std::unordered_set<const RigidBody*> uniqueBodies;
    for (const RigidBody* body : bodies) {
        if (body == nullptr) {
            throw std::invalid_argument(
                "Coupled fluid simulation requires valid rigid bodies."
            );
        }
        if (!uniqueBodies.insert(body).second) {
            throw std::invalid_argument(
                "Coupled fluid simulation cannot advance a rigid body twice."
            );
        }
    }
}

void AccumulateCouplingStatistics(
    FluidRigidCouplingStatistics& total,
    const FluidRigidCouplingStatistics& substep
) {
    total.interactionCount += substep.interactionCount;
    total.contactCorrectionCount += substep.contactCorrectionCount;
    total.pressureImpulseMagnitude += substep.pressureImpulseMagnitude;
    total.viscosityImpulseMagnitude += substep.viscosityImpulseMagnitude;
    total.maximumPenetration = std::max(
        total.maximumPenetration,
        substep.maximumPenetration
    );
}

std::vector<FluidBoundaryParticle> BuildBoundaryParticles(
    const IFluidContainer* boundary,
    const std::vector<RigidBody*>& bodies,
    const FluidBoundarySamplingSettings& settings
) {
    std::vector<FluidBoundaryParticle> particles;
    if (boundary) {
        particles = SampleFluidContainerBoundary(*boundary, settings);
    }
    std::vector<FluidBoundaryParticle> rigidParticles =
        SampleRigidBodyBoundaries(bodies, settings);
    particles.insert(
        particles.end(),
        rigidParticles.begin(),
        rigidParticles.end()
    );
    return particles;
}

} // namespace

CoupledFluidSimulation::CoupledFluidSimulation(
    float referenceSmoothingLength,
    const WcsphConfig& fluidConfig,
    const FluidRigidCouplingSettings& couplingSettings
) : fluidSolver(referenceSmoothingLength, fluidConfig),
    coupler(couplingSettings) {
    boundarySampling.spacing = referenceSmoothingLength * 0.5f;
    boundarySampling.supportRadius = referenceSmoothingLength;
    boundarySampling.Validate();
}

void CoupledFluidSimulation::resetStatistics() {
    lastStatistics = CoupledFluidStatistics{};
}

void CoupledFluidSimulation::advanceRigidSubstep(
    std::vector<FluidParticle>& particles,
    const std::vector<RigidBody*>& bodies,
    float deltaTime
) {
    const FluidRigidCouplingStatistics couplingStatistics = coupler.couple(
        particles,
        bodies,
        deltaTime
    );
    AccumulateCouplingStatistics(lastStatistics.coupling, couplingStatistics);

    const Vector2 acceleration = fluidSolver.getConfig().externalAcceleration;
    for (RigidBody* body : bodies) {
        if (!body->IsStatic()) {
            body->ApplyForce(acceleration * body->GetMass());
        }
        body->Integrate(deltaTime);
    }
    ++lastStatistics.rigidBodySubstepCount;
    lastStatistics.integratedTime += deltaTime;
}

void CoupledFluidSimulation::step(
    std::vector<FluidParticle>& particles,
    const std::vector<RigidBody*>& bodies,
    float deltaTime
) {
    ValidateBodies(bodies);
    resetStatistics();
    std::vector<FluidBoundaryParticle> boundaryParticles =
        BuildBoundaryParticles(nullptr, bodies, boundarySampling);
    const auto callback = [this, &particles, &bodies, &boundaryParticles](float substep) {
        advanceRigidSubstep(particles, bodies, substep);
        boundaryParticles = BuildBoundaryParticles(
            nullptr,
            bodies,
            boundarySampling
        );
    };
    fluidSolver.step(particles, deltaTime, boundaryParticles, callback);
    lastStatistics.fluid = fluidSolver.getLastStatistics();
}

void CoupledFluidSimulation::step(
    std::vector<FluidParticle>& particles,
    const std::vector<RigidBody*>& bodies,
    float deltaTime,
    const IFluidContainer& boundary
) {
    ValidateBodies(bodies);
    resetStatistics();
    std::vector<FluidBoundaryParticle> boundaryParticles =
        BuildBoundaryParticles(&boundary, bodies, boundarySampling);
    const auto callback = [this, &particles, &bodies, &boundary, &boundaryParticles](float substep) {
        advanceRigidSubstep(particles, bodies, substep);
        boundaryParticles = BuildBoundaryParticles(
            &boundary,
            bodies,
            boundarySampling
        );
    };
    fluidSolver.step(
        particles,
        deltaTime,
        boundary,
        boundaryParticles,
        callback
    );
    lastStatistics.fluid = fluidSolver.getLastStatistics();
}

void CoupledFluidSimulation::setBoundarySamplingSettings(
    const FluidBoundarySamplingSettings& settings
) {
    settings.Validate();
    boundarySampling = settings;
}

const CoupledFluidStatistics&
CoupledFluidSimulation::getLastStatistics() const {
    return lastStatistics;
}

} // namespace PhysicsEngine

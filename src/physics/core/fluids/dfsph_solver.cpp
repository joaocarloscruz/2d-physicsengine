#include "physics/core/fluids/dfsph_solver.h"
#include "physics/core/fluids/sph_kernels.h"
#include "sph_viscosity.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace PhysicsEngine {
void DfsphConfig::Validate() const {
    SphKernels2D::ValidateFamily(kernelFamily);
    if (!std::isfinite(externalAcceleration.x) || !std::isfinite(externalAcceleration.y) ||
        !std::isfinite(maximumTimeStep) || maximumTimeStep <= 0 ||
        !std::isfinite(cflFactor) || cflFactor <= 0 || cflFactor > 1 ||
        maximumSubsteps <= 0 || maximumIterations <= 0 ||
        !std::isfinite(densityTolerance) || densityTolerance <= 0 ||
        !std::isfinite(divergenceTolerance) || divergenceTolerance <= 0 ||
        !std::isfinite(relaxation) || relaxation <= 0 || relaxation > 0.5f)
        throw std::invalid_argument("Invalid DFSPH configuration.");
}
DfsphSolver::DfsphSolver(float h, const DfsphConfig& settings) : config(settings), grid(h) { config.Validate(); }

void DfsphSolver::prepare(std::vector<FluidParticle>& particles) {
    float radius = grid.getCellSize();
    for (const auto& p : particles) {
        FluidParticleProperties{p.mass, p.restDensity, p.smoothingLength, p.viscosity}.Validate();
        if (!std::isfinite(p.position.x) || !std::isfinite(p.position.y) ||
            !std::isfinite(p.velocity.x) || !std::isfinite(p.velocity.y))
            throw std::invalid_argument("DFSPH particle state must be finite.");
        radius = std::max(radius, p.smoothingLength);
    }
    grid.rebuild(particles);
    neighbors = grid.findNeighborPairs(particles, radius);
    pairs.clear(); pairs.reserve(neighbors.size());
    for (auto& p : particles) {
        p.inverseMass = 1/p.mass;
        p.density = p.mass*SphKernels2D::DensityWeight({}, p.smoothingLength, config.kernelFamily);
    }
    for (const auto& pair : neighbors) {
        auto& a = particles[pair.first]; auto& b = particles[pair.second];
        const float h = static_cast<float>(0.5*(static_cast<double>(a.smoothingLength)+b.smoothingLength));
        const Vector2 displacement = a.position-b.position;
        const float weight = SphKernels2D::DensityWeight(displacement, h, config.kernelFamily);
        a.density += b.mass*weight; b.density += a.mass*weight;
        pairs.push_back({pair.first, pair.second, SphKernels2D::PressureGradient(displacement, h, config.kernelFamily)});
    }
    std::vector<std::pair<double, double>> sum(particles.size());
    std::vector<double> squared(particles.size(), 0.0);
    diagonal.assign(particles.size(), 0);
    for (const auto& pair : pairs) {
        const auto& a = particles[pair.a]; const auto& b = particles[pair.b];
        sum[pair.a].first += static_cast<double>(pair.gradient.x)*b.mass;
        sum[pair.a].second += static_cast<double>(pair.gradient.y)*b.mass;
        sum[pair.b].first -= static_cast<double>(pair.gradient.x)*a.mass;
        sum[pair.b].second -= static_cast<double>(pair.gradient.y)*a.mass;
        const double normSquared = static_cast<double>(pair.gradient.x)*pair.gradient.x
            + static_cast<double>(pair.gradient.y)*pair.gradient.y;
        squared[pair.a] += b.mass*normSquared;
        squared[pair.b] += a.mass*normSquared;
    }
    for (std::size_t i=0; i<particles.size(); ++i) {
        auto& p = particles[i];
        if (!std::isfinite(p.density) || p.density <= 0) throw std::runtime_error("DFSPH density overflow.");
        p.volume = p.mass/p.density;
        // Exact diagonal for unequal particle masses, without the time factor.
        diagonal[i] = SphViscosity::CheckedFloat(-(sum[i].first*sum[i].first
            + sum[i].second*sum[i].second + p.mass*squared[i])
            /(static_cast<double>(p.density)*p.density));
    }
    std::vector<double> viscosityRows(particles.size(), 0.0);
    for (auto& pair : pairs) {
        const auto& a = particles[pair.a]; const auto& b = particles[pair.b];
        const float h = static_cast<float>(0.5*(static_cast<double>(a.smoothingLength)+b.smoothingLength));
        pair.viscosityCoupling = SphViscosity::Coupling(a, b,
            SphKernels2D::ViscosityLaplacian(a.position-b.position, h));
        viscosityRows[pair.a] += pair.viscosityCoupling/a.mass;
        viscosityRows[pair.b] += pair.viscosityCoupling/b.mass;
    }
    viscosityTimeLimit = SphViscosity::RowLimit(viscosityRows);
}

std::vector<float> DfsphSolver::densityRates(const std::vector<FluidParticle>& p,
    const std::vector<Vector2>& velocities) const {
    std::vector<float> result(p.size(), 0);
    for (const auto& pair : pairs) {
        const float value = (velocities[pair.a]-velocities[pair.b]).dot(pair.gradient);
        result[pair.a] += p[pair.b].mass*value;
        result[pair.b] += p[pair.a].mass*value;
    }
    return result;
}
std::vector<Vector2> DfsphSolver::pressureAcceleration(const std::vector<FluidParticle>& p,
    const std::vector<float>& pressure) const {
    std::vector<Vector2> result(p.size());
    for (const auto& pair : pairs) {
        const auto& a = p[pair.a]; const auto& b = p[pair.b];
        const Vector2 gradientPressure = pair.gradient*(pressure[pair.a]/(a.density*a.density)
            + pressure[pair.b]/(b.density*b.density));
        result[pair.a] = result[pair.a]-gradientPressure*b.mass;
        result[pair.b] = result[pair.b]+gradientPressure*a.mass;
    }
    return result;
}

void DfsphSolver::project(std::vector<FluidParticle>& p, float dt, bool density) {
    std::vector<Vector2> velocities;
    for (const auto& particle : p) velocities.push_back(particle.velocity);
    const auto rates = densityRates(p, velocities);
    std::vector<float> pressure(p.size(), 0), source(p.size());
    for (std::size_t i=0; i<p.size(); ++i)
        source[i] = (density ? (p[i].restDensity-p[i].density)/dt : 0)-rates[i];
    const float tolerance = density ? config.densityTolerance : config.divergenceTolerance;
    std::vector<Vector2> acceleration;
    float residual = 0;
    int iteration = 0;
    for (;;) {
        acceleration = pressureAcceleration(p, pressure);
        const auto response = densityRates(p, acceleration);
        residual = 0;
        for (std::size_t i=0; i<p.size(); ++i) {
            const float error = dt*response[i]-source[i];
            if (!std::isfinite(error)) throw std::runtime_error("DFSPH projection overflow.");
            residual = std::max(residual, (density ? dt : 1)*error/p[i].restDensity);
        }
        if (residual <= tolerance || iteration == config.maximumIterations) break;
        for (std::size_t i=0; i<p.size(); ++i) {
            const float diag = dt*diagonal[i];
            if (diag < -1e-12f) pressure[i] = std::max(0.0f,
                pressure[i]+config.relaxation*(source[i]-dt*response[i])/diag);
            if (!std::isfinite(pressure[i])) throw std::runtime_error("DFSPH pressure overflow.");
        }
        ++iteration;
    }
    for (std::size_t i=0; i<p.size(); ++i) {
        p[i].velocity = p[i].velocity+acceleration[i]*dt;
        p[i].pressure = pressure[i];
    }
    diagnostics.converged = diagnostics.converged && residual <= tolerance;
    if (density) {
        diagnostics.densityIterations += iteration;
        diagnostics.densityResidual = std::max(diagnostics.densityResidual, residual);
    } else {
        diagnostics.divergenceIterations += iteration;
        diagnostics.divergenceResidual = std::max(diagnostics.divergenceResidual, residual);
    }
}

float DfsphSolver::stableTimeStep(const std::vector<FluidParticle>& particles) const {
    double dt = std::min(static_cast<double>(config.maximumTimeStep), viscosityTimeLimit);
    for (const auto& p : particles) {
        const double speed = std::hypot(static_cast<double>(p.velocity.x), p.velocity.y);
        if (speed > 0.0) dt = std::min(dt, config.cflFactor*p.smoothingLength/speed);
        dt = std::min(dt, SphViscosity::ContinuumLimit(p));
        const double acceleration = std::hypot(static_cast<double>(config.externalAcceleration.x), config.externalAcceleration.y);
        if (acceleration > 0.0) dt = std::min(dt, config.cflFactor*std::sqrt(p.smoothingLength/acceleration));
    }
    return SphViscosity::TimeStep(dt);
}

void DfsphSolver::step(std::vector<FluidParticle>& particles, float deltaTime) {
    if (!std::isfinite(deltaTime) || deltaTime < 0)
        throw std::invalid_argument("DFSPH timestep must be finite and non-negative.");
    prepare(particles);
    diagnostics = {};
    float remaining = deltaTime;
    while (remaining > 0 && !particles.empty()) {
        if (diagnostics.substeps >= static_cast<std::uint32_t>(config.maximumSubsteps))
            throw std::runtime_error("DFSPH exceeded maximum substeps.");
        const float dt = std::min(remaining, stableTimeStep(particles));
        for (auto& p : particles) p.force = SphViscosity::CheckedVector(
            static_cast<double>(config.externalAcceleration.x)*p.mass,
            static_cast<double>(config.externalAcceleration.y)*p.mass);
        for (const auto& pair : pairs) {
            auto& a = particles[pair.a]; auto& b = particles[pair.b];
            const Vector2 force = SphViscosity::PairForce(a, b, pair.viscosityCoupling);
            a.force = SphViscosity::Add(a.force, force);
            b.force = SphViscosity::Add(b.force, force, -1.0);
        }
        for (auto& p : particles) p.velocity = SphViscosity::AdvanceVelocity(p, dt);
        project(particles, dt, true);
        for (auto& p : particles) p.position = SphViscosity::Add(p.position, p.velocity, dt);
        prepare(particles);
        project(particles, dt, false);
        remaining -= dt;
        if (remaining < deltaTime*1e-6f) remaining = 0;
        ++diagnostics.substeps;
    }
    const auto measured = MeasureFluidDiagnostics(particles, neighbors, config.kernelFamily);
    diagnostics.maximumDensityError = measured.maximumDensityError;
    diagnostics.maximumCompression = measured.maximumCompression;
    diagnostics.maximumAbsoluteDensityRate = measured.maximumAbsoluteDensityRate;
    diagnostics.maximumCompressionRate = measured.maximumCompressionRate;
}
}

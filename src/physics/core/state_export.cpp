#include "physics/core/state_export.h"
#include <cmath>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <stdexcept>

namespace PhysicsEngine {
namespace {
void Finite(double value) {
    if (!std::isfinite(value)) throw std::invalid_argument("Export requires finite state and time.");
}
void Finite(Vector2 value) { Finite(value.x); Finite(value.y); }
std::ostringstream Stream(double time) {
    Finite(time);
    std::ostringstream stream;
    stream.imbue(std::locale::classic());
    stream << std::setprecision(std::numeric_limits<double>::max_digits10) << std::boolalpha;
    return stream;
}
void Vector(std::ostream& out, Vector2 v) {
    Finite(v); out << "[" << v.x << "," << v.y << "]";
}
void Validate(const RigidBody& b) {
    Finite(b.position); Finite(b.velocity); Finite(b.force);
    Finite(b.orientation); Finite(b.angularVelocity); Finite(b.torque);
    Finite(b.mass); Finite(b.inertia);
}
void Validate(const FluidParticle& p) {
    Finite(p.position); Finite(p.velocity); Finite(p.force); Finite(p.mass);
    Finite(p.density); Finite(p.restDensity); Finite(p.pressure); Finite(p.smoothingLength);
}
void Statistics(std::ostream& out, const SimulationStatistics& s) {
    out << "{\"integratedBodyCount\":" << s.integratedBodyCount
        << ",\"integratedParticleCount\":" << s.integratedParticleCount
        << ",\"broadPhaseCandidateCount\":" << s.broadPhaseCandidateCount
        << ",\"narrowPhaseCandidateCount\":" << s.narrowPhaseCandidateCount
        << ",\"resolvedContactCount\":" << s.resolvedContactCount
        << ",\"solverIterationCount\":" << s.solverIterationCount
        << ",\"activeContactCount\":" << s.activeContactCount
        << ",\"fluidIterationCount\":" << s.fluidIterationCount
        << ",\"ccdImpactCount\":" << s.ccdImpactCount
        << ",\"ccdIterationLimitReached\":" << s.ccdIterationLimitReached
        << ",\"islandCount\":" << s.islandCount
        << ",\"solvedIslandCount\":" << s.solvedIslandCount
        << ",\"solvedConstraintCount\":" << s.solvedConstraintCount
        << ",\"sleepingBodyCount\":" << s.sleepingBodyCount << "}";
}
}
std::string ExportWorldJson(const World& world, double time) {
    auto out = Stream(time);
    out << "{\"schemaVersion\":1,\"time\":" << time << ",\"statistics\":";
    Statistics(out, world.getLastStepStatistics());
    out << ",\"bodies\":[";
    bool comma = false;
    for (const auto& body : world.getBodies()) {
        const auto& b = *body; Validate(b);
        if (comma) out << ","; comma = true;
        // IDs are strings so JavaScript consumers do not lose uint64 precision.
        out << "{\"id\":\"" << b.GetId() << "\",\"position\":"; Vector(out, b.position);
        out << ",\"velocity\":"; Vector(out, b.velocity);
        out << ",\"force\":"; Vector(out, b.force);
        out << ",\"orientation\":" << b.orientation << ",\"angularVelocity\":" << b.angularVelocity
            << ",\"torque\":" << b.torque << ",\"mass\":" << b.mass << ",\"inertia\":" << b.inertia
            << ",\"static\":" << b.IsStatic() << ",\"awake\":" << b.IsAwake()
            << ",\"ccd\":" << b.IsCcdEnabled() << ",\"shape\":{";
        if (b.shape->type == ShapeType::CIRCLE) {
            out << "\"type\":\"circle\",\"radius\":" << b.shape->GetRadius();
        } else {
            out << "\"type\":\"polygon\",\"vertices\":[";
            bool vertexComma = false;
            for (const auto& v : static_cast<const Polygon*>(b.shape.get())->getVertices()) {
                if (vertexComma) out << ","; vertexComma = true; Vector(out, v);
            }
            out << "]";
        }
        out << "}}";
    }
    out << "],\"particleSystems\":[";
    comma = false;
    for (const auto& system : world.getParticleSystems()) {
        if (comma) out << ","; comma = true; out << "[";
        bool particleComma = false;
        for (const auto& p : system->getParticles()) {
            if (particleComma) out << ","; particleComma = true;
            out << "{\"position\":"; Vector(out, p.position);
            out << ",\"velocity\":"; Vector(out, p.velocity); out << "}";
        }
        out << "]";
    }
    out << "]}\n"; return out.str();
}
std::string ExportWorldCsv(const World& world, double time) {
    auto out = Stream(time);
    out << "time,id,x,y,vx,vy,angle,angular_velocity,mass,inertia,static,awake,ccd,integrated_bodies,active_contacts,ccd_impacts,islands,solved_islands,sleeping_bodies\n";
    const auto& s = world.getLastStepStatistics();
    for (const auto& body : world.getBodies()) {
        const auto& b = *body; Validate(b);
        out << time << ',' << b.GetId() << ',' << b.position.x << ',' << b.position.y << ','
            << b.velocity.x << ',' << b.velocity.y << ',' << b.orientation << ',' << b.angularVelocity << ','
            << b.mass << ',' << b.inertia << ',' << b.IsStatic() << ',' << b.IsAwake() << ',' << b.IsCcdEnabled() << ','
            << s.integratedBodyCount << ',' << s.activeContactCount << ',' << s.ccdImpactCount << ','
            << s.islandCount << ',' << s.solvedIslandCount << ',' << s.sleepingBodyCount << '\n';
    }
    return out.str();
}
std::string ExportFluidCsv(const std::vector<FluidParticle>& particles, double time) {
    auto out = Stream(time);
    out << "time,index,x,y,vx,vy,mass,density,rest_density,pressure,smoothing_length\n";
    for (std::size_t i=0; i<particles.size(); ++i) {
        const auto& p = particles[i]; Validate(p);
        out << time << ',' << i << ',' << p.position.x << ',' << p.position.y << ','
            << p.velocity.x << ',' << p.velocity.y << ',' << p.mass << ',' << p.density << ','
            << p.restDensity << ',' << p.pressure << ',' << p.smoothingLength << '\n';
    }
    return out.str();
}
std::string ExportFluidJson(const std::vector<FluidParticle>& particles,
    const FluidDiagnostics& d, double time) {
    auto out = Stream(time);
    for (float value : {d.maximumDensityError, d.maximumCompression, d.maximumAbsoluteDensityRate,
        d.maximumCompressionRate, d.densityResidual, d.divergenceResidual}) Finite(value);
    out << "{\"schemaVersion\":1,\"time\":" << time << ",\"diagnostics\":{"
        << "\"maximumDensityError\":" << d.maximumDensityError
        << ",\"maximumCompression\":" << d.maximumCompression
        << ",\"maximumAbsoluteDensityRate\":" << d.maximumAbsoluteDensityRate
        << ",\"maximumCompressionRate\":" << d.maximumCompressionRate
        << ",\"densityResidual\":" << d.densityResidual
        << ",\"divergenceResidual\":" << d.divergenceResidual
        << ",\"densityIterations\":" << d.densityIterations
        << ",\"divergenceIterations\":" << d.divergenceIterations
        << ",\"substeps\":" << d.substeps << ",\"converged\":" << d.converged << "},\"particles\":[";
    for (std::size_t i=0; i<particles.size(); ++i) {
        const auto& p = particles[i]; Validate(p);
        if (i) out << ',';
        out << "{\"index\":" << i << ",\"position\":"; Vector(out, p.position);
        out << ",\"velocity\":"; Vector(out, p.velocity);
        out << ",\"mass\":" << p.mass << ",\"density\":" << p.density
            << ",\"restDensity\":" << p.restDensity << ",\"pressure\":" << p.pressure << "}";
    }
    out << "]}\n"; return out.str();
}
}

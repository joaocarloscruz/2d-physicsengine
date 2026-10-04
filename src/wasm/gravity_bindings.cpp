#include <emscripten/bind.h>
#include "physics/core/nbody_gravity.h"
#include "checked_indices.h"

namespace PhysicsEngine::Wasm {
namespace {
struct GravityConfig {
    double gravitationalStrength, softening, maxSubstep, frequencySafety;
    double maxParticles, maxSubsteps, maxPairWork;
};
NBodyGravityConfig Native(GravityConfig c) {
    return {c.gravitationalStrength,c.softening,c.maxSubstep,c.frequencySafety,
        Count(c.maxParticles),Count(c.maxSubsteps),Count(c.maxPairWork)};
}
NBodyGravity* Create(GravityConfig c) { return new NBodyGravity(Native(c)); }
GravityConfig Config(const NBodyGravity& s) {
    const auto& c=s.getConfig();
    return {c.gravitationalStrength,c.softening,c.maxSubstep,c.frequencySafety,
        double(c.maxParticles),double(c.maxSubsteps),double(c.maxPairWork)};
}
std::size_t ParticleIndex(const NBodyGravity& s,double index) { return Index(index,s.getParticles().size()); }
}
}

EMSCRIPTEN_BINDINGS(nbody_gravity) {
    using namespace emscripten;
    using namespace PhysicsEngine;
    using namespace PhysicsEngine::Wasm;
    value_object<GravityConfig>("NBodyGravityConfig")
        .field("gravitationalStrength",&GravityConfig::gravitationalStrength)
        .field("softening",&GravityConfig::softening).field("maxSubstep",&GravityConfig::maxSubstep)
        .field("frequencySafety",&GravityConfig::frequencySafety).field("maxParticles",&GravityConfig::maxParticles)
        .field("maxSubsteps",&GravityConfig::maxSubsteps).field("maxPairWork",&GravityConfig::maxPairWork);
    value_object<GravityParticle>("GravityParticle")
        .field("position",&GravityParticle::position).field("velocity",&GravityParticle::velocity)
        .field("mass",&GravityParticle::mass);
    value_object<GravityDiagnostics>("GravityDiagnostics")
        .field("totalMass",&GravityDiagnostics::totalMass).field("centerOfMass",&GravityDiagnostics::centerOfMass)
        .field("momentum",&GravityDiagnostics::momentum).field("angularMomentum",&GravityDiagnostics::angularMomentum)
        .field("kineticEnergy",&GravityDiagnostics::kineticEnergy).field("potentialEnergy",&GravityDiagnostics::potentialEnergy)
        .field("totalEnergy",&GravityDiagnostics::totalEnergy).field("lastSubsteps",&GravityDiagnostics::lastSubsteps)
        .field("lastPairWork",&GravityDiagnostics::lastPairWork);
    class_<NBodyGravity>("NBodyGravity")
        .constructor<>().constructor(&Create,allow_raw_pointers())
        .function("getConfig",&Config)
        .function("setConfig",optional_override([](NBodyGravity& s,GravityConfig c) { s.setConfig(Native(c)); }))
        .function("getParticleCount",optional_override([](const NBodyGravity& s) { return s.getParticles().size(); }))
        .function("getParticle",optional_override([](const NBodyGravity& s,double index) { return GravityParticle(s.getParticles().at(ParticleIndex(s,index))); }))
        .function("addParticle",optional_override([](NBodyGravity& s,Vector2d p) { return s.addParticle(p); }))
        .function("addParticle",optional_override([](NBodyGravity& s,Vector2d p,Vector2d v,double mass) { return s.addParticle(p,v,mass); }))
        .function("setState",optional_override([](NBodyGravity& s,double index,Vector2d p,Vector2d v) { s.setState(ParticleIndex(s,index),p,v); }))
        .function("applyImpulse",optional_override([](NBodyGravity& s,double index,Vector2d impulse) { s.applyImpulse(ParticleIndex(s,index),impulse); }))
        .function("getDiagnostics",&NBodyGravity::getDiagnostics)
        .function("step",&NBodyGravity::step);
}

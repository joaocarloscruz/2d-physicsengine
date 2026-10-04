#include <emscripten/bind.h>
#include "physics/core/softbody.h"
#include "checked_indices.h"

namespace PhysicsEngine::Wasm {
namespace {
struct SoftConfig {
    double maxSubstep, stabilityFactor, maxSubsteps, maxParticles, maxSprings;
};
SoftBodyConfig Native(SoftConfig c) {
    return {c.maxSubstep,c.stabilityFactor,Count(c.maxSubsteps),Count(c.maxParticles),Count(c.maxSprings)};
}
SoftBody* Create(SoftConfig c) { return new SoftBody(Native(c)); }
SoftConfig Config(const SoftBody& s) {
    const auto& c=s.getConfig();
    return {c.maxSubstep,c.stabilityFactor,double(c.maxSubsteps),double(c.maxParticles),double(c.maxSprings)};
}
std::size_t ParticleIndex(const SoftBody& s,double index) { return Index(index,s.getParticles().size()); }
}
}

EMSCRIPTEN_BINDINGS(softbody) {
    using namespace emscripten;
    using namespace PhysicsEngine;
    using namespace PhysicsEngine::Wasm;
    value_object<SoftConfig>("SoftBodyConfig")
        .field("maxSubstep",&SoftConfig::maxSubstep).field("stabilityFactor",&SoftConfig::stabilityFactor)
        .field("maxSubsteps",&SoftConfig::maxSubsteps).field("maxParticles",&SoftConfig::maxParticles)
        .field("maxSprings",&SoftConfig::maxSprings);
    value_object<SoftBodyForce>("SoftBodyForce")
        .field("x",&SoftBodyForce::x).field("y",&SoftBodyForce::y);
    value_object<SoftBodyParticle>("SoftBodyParticle")
        .field("position",&SoftBodyParticle::position).field("velocity",&SoftBodyParticle::velocity)
        .field("mass",&SoftBodyParticle::mass).field("fixed",&SoftBodyParticle::fixed)
        .field("force",&SoftBodyParticle::force);
    value_object<SoftBodySpring>("SoftBodySpring")
        .field("first",&SoftBodySpring::first).field("second",&SoftBodySpring::second)
        .field("restLength",&SoftBodySpring::restLength).field("stiffness",&SoftBodySpring::stiffness)
        .field("damping",&SoftBodySpring::damping);
    value_object<SoftBodyDiagnostics>("SoftBodyDiagnostics")
        .field("totalMass",&SoftBodyDiagnostics::totalMass).field("momentumX",&SoftBodyDiagnostics::momentumX)
        .field("momentumY",&SoftBodyDiagnostics::momentumY).field("kineticEnergy",&SoftBodyDiagnostics::kineticEnergy)
        .field("elasticEnergy",&SoftBodyDiagnostics::elasticEnergy).field("maxStrain",&SoftBodyDiagnostics::maxStrain)
        .field("lastSubsteps",&SoftBodyDiagnostics::lastSubsteps);
    class_<SoftBody>("SoftBody")
        .constructor<>().constructor(&Create,allow_raw_pointers())
        .function("getConfig",&Config)
        .function("setConfig",optional_override([](SoftBody& s,SoftConfig c) { s.setConfig(Native(c)); }))
        .function("getParticleCount",optional_override([](const SoftBody& s) { return s.getParticles().size(); }))
        .function("getSpringCount",optional_override([](const SoftBody& s) { return s.getSprings().size(); }))
        .function("getParticle",optional_override([](const SoftBody& s,double index) { return SoftBodyParticle(s.getParticles().at(ParticleIndex(s,index))); }))
        .function("getSpring",optional_override([](const SoftBody& s,double index) { return SoftBodySpring(s.getSprings().at(Index(index,s.getSprings().size()))); }))
        .function("getAccumulatedForce",optional_override([](const SoftBody& s,double index) { return SoftBodyForce(s.getAccumulatedForce(ParticleIndex(s,index))); }))
        .function("addParticle",optional_override([](SoftBody& s,Vector2 p) { return s.addParticle(p); }))
        .function("addParticle",optional_override([](SoftBody& s,Vector2 p,Vector2 v,double mass,bool fixed) { return s.addParticle(p,v,mass,fixed); }))
        .function("addSpring",optional_override([](SoftBody& s,double first,double second,double rest,double k) { return s.addSpring(ParticleIndex(s,first),ParticleIndex(s,second),rest,k); }))
        .function("addSpring",optional_override([](SoftBody& s,double first,double second,double rest,double k,double damping) { return s.addSpring(ParticleIndex(s,first),ParticleIndex(s,second),rest,k,damping); }))
        .function("setParticleState",optional_override([](SoftBody& s,double index,Vector2 p) { s.setParticleState(ParticleIndex(s,index),p); }))
        .function("setParticleState",optional_override([](SoftBody& s,double index,Vector2 p,Vector2 v) { s.setParticleState(ParticleIndex(s,index),p,v); }))
        .function("setFixed",optional_override([](SoftBody& s,double index,bool fixed) { s.setFixed(ParticleIndex(s,index),fixed); }))
        .function("applyImpulse",optional_override([](SoftBody& s,double index,Vector2 impulse) { s.applyImpulse(ParticleIndex(s,index),impulse); }))
        .function("applyForce",optional_override([](SoftBody& s,double index,Vector2 force) { s.applyForce(ParticleIndex(s,index),force); }))
        .function("applyForce",optional_override([](SoftBody& s,double index,double x,double y) { s.applyForce(ParticleIndex(s,index),x,y); }))
        .function("clearForces",select_overload<void()>(&SoftBody::clearForces))
        .function("clearForces",optional_override([](SoftBody& s,double index) { s.clearForces(ParticleIndex(s,index)); }))
        .function("setUniformAcceleration",&SoftBody::setUniformAcceleration)
        .function("getUniformAcceleration",optional_override([](const SoftBody& s) { return Vector2(s.getUniformAcceleration()); }))
        .function("getDiagnostics",&SoftBody::getDiagnostics)
        .function("step",&SoftBody::step);
}

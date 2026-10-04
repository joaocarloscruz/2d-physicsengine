#include <emscripten/bind.h>

#include <memory>
#include "checked_indices.h"

#include "engine.h"
#include "physics/core/charged_particle.h"
#include "physics/core/prismatic_joint.h"
#include "physics/core/material.h"
#include "physics/core/fixed_step_runner.h"
#include "physics/core/particles/particle_system.h"
#include "physics/core/rigidbody.h"
#include "physics/core/shape.h"
#include "physics/core/simulation_config.h"
#include "physics/core/simulation_statistics.h"
#include "physics/math/vector2.h"

namespace PhysicsEngine {
namespace {

// Keep integer input fields as JS doubles until the complete native call can
// validate them. Embind otherwise truncates/wraps before native validation.
struct SimulationConfigInput : SimulationConfig {
    double substepCount = 8, iterationCount = 10, ccdCount = 32;
};
int SignedCount(double value) {
    const auto count = Wasm::Count(value);
    if (count > static_cast<std::size_t>(std::numeric_limits<int>::max()))
        throw std::invalid_argument("Simulation count exceeds native integer range");
    return static_cast<int>(count);
}
void SetSimulationConfig(Engine& engine, const SimulationConfigInput& input) {
    SimulationConfig native = input;
    native.maxSubstepsPerAdvance = SignedCount(input.substepCount);
    native.solverIterations = SignedCount(input.iterationCount);
    native.maximumCcdImpacts = SignedCount(input.ccdCount);
    engine.setSimulationConfig(native);
}
SimulationConfigInput GetSimulationConfig(const Engine& engine) {
    SimulationConfigInput result;
    const auto& native = engine.getSimulationConfig();
    static_cast<SimulationConfig&>(result) = native;
    result.substepCount = native.maxSubstepsPerAdvance;
    result.iterationCount = native.solverIterations;
    result.ccdCount = native.maximumCcdImpacts;
    return result;
}

RigidBodyPtr CreateRigidBody(
    Shape* shape,
    const Material& material,
    const Vector2& position,
    bool isStatic
) {
    return std::make_shared<RigidBody>(shape, material, position, isStatic);
}

Polygon* CreateBox(float width, float height) {
    return new Polygon(Polygon::MakeBox(width, height));
}

ParticleSystemPtr CreateParticleSystem() {
    return std::make_shared<ParticleSystem>();
}

JointPtr CreateDistanceJoint(RigidBodyPtr a, RigidBodyPtr b, float length, Vector2 anchorA, Vector2 anchorB) {
    return std::make_shared<DistanceJoint>(std::move(a), std::move(b), length, anchorA, anchorB);
}
std::shared_ptr<RevoluteJoint> CreateRevoluteJoint(RigidBodyPtr a, RigidBodyPtr b, Vector2 anchorA, Vector2 anchorB) {
    return std::make_shared<RevoluteJoint>(std::move(a), std::move(b), anchorA, anchorB);
}
std::shared_ptr<PrismaticJoint> CreatePrismaticJoint(RigidBodyPtr a, RigidBodyPtr b,
    Vector2 axis, Vector2 anchorA, Vector2 anchorB) {
    return std::make_shared<PrismaticJoint>(std::move(a), std::move(b), axis, anchorA, anchorB);
}

} // namespace
} // namespace PhysicsEngine

EMSCRIPTEN_BINDINGS(physics_engine) {
    using namespace emscripten;
    using namespace PhysicsEngine;

    value_object<Vector2>("Vector2")
        .field("x", &Vector2::x)
        .field("y", &Vector2::y);

    value_object<Vector2d>("Vector2d")
        .field("x", &Vector2d::x)
        .field("y", &Vector2d::y);
    value_object<UniformElectromagneticField>("UniformElectromagneticField")
        .field("electric", &UniformElectromagneticField::electric)
        .field("magnetic", &UniformElectromagneticField::magnetic);
    class_<ChargedParticle>("ChargedParticle")
        .constructor<>()
        .constructor<Vector2d, Vector2d, double, double>()
        .function("getPosition", optional_override([](const ChargedParticle& p) { return p.getPosition(); }))
        .function("getVelocity", optional_override([](const ChargedParticle& p) { return p.getVelocity(); }))
        .function("getMass", &ChargedParticle::getMass)
        .function("getCharge", &ChargedParticle::getCharge)
        .function("getKineticEnergy", &ChargedParticle::getKineticEnergy)
        .function("setState", &ChargedParticle::setState)
        .function("step", &ChargedParticle::step);

    value_object<Material>("Material")
        .field("density", &Material::density)
        .field("restitution", &Material::restitution)
        .field("staticFriction", &Material::staticFriction)
        .field("dynamicFriction", &Material::dynamicFriction);

    value_object<SimulationConfigInput>("SimulationConfig")
        .field("fixedTimeStep", &SimulationConfig::fixedTimeStep)
        .field("maxSubstepsPerAdvance", &SimulationConfigInput::substepCount)
        .field("solverIterations", &SimulationConfigInput::iterationCount)
        .field("positionCorrectionFactor", &SimulationConfig::positionCorrectionFactor)
        .field("penetrationSlop", &SimulationConfig::penetrationSlop)
        .field("warmStartFactor", &SimulationConfig::warmStartFactor)
        .field("restitutionVelocityThreshold", &SimulationConfig::restitutionVelocityThreshold)
        .field("velocityTolerance", &SimulationConfig::velocityTolerance)
        .field("maxPositionCorrection", &SimulationConfig::maxPositionCorrection)
        .field("enableLinearVelocityLimit", &SimulationConfig::enableLinearVelocityLimit)
        .field("maxLinearSpeed", &SimulationConfig::maxLinearSpeed)
        .field("enableAngularVelocityLimit", &SimulationConfig::enableAngularVelocityLimit)
        .field("maxAngularSpeed", &SimulationConfig::maxAngularSpeed)
        .field("enableSleeping", &SimulationConfig::enableSleeping)
        .field("sleepEnergyThreshold", &SimulationConfig::sleepEnergyThreshold)
        .field("sleepTimeThreshold", &SimulationConfig::sleepTimeThreshold)
        .field("maximumCcdImpacts", &SimulationConfigInput::ccdCount);

    value_object<FixedStepResult>("FixedStepResult")
        .field("stepsPerformed", &FixedStepResult::stepsPerformed)
        .field("simulatedTime", &FixedStepResult::simulatedTime)
        .field("remainingTime", &FixedStepResult::remainingTime)
        .field("interpolationAlpha", &FixedStepResult::interpolationAlpha);

    value_object<SimulationStatistics>("SimulationStatistics")
        .field("integratedBodyCount", &SimulationStatistics::integratedBodyCount)
        .field("integratedParticleCount", &SimulationStatistics::integratedParticleCount)
        .field("broadPhaseCandidateCount", &SimulationStatistics::broadPhaseCandidateCount)
        .field("narrowPhaseCandidateCount", &SimulationStatistics::narrowPhaseCandidateCount)
        .field("resolvedContactCount", &SimulationStatistics::resolvedContactCount)
        .field("solverIterationCount", &SimulationStatistics::solverIterationCount)
        .field("activeContactCount", &SimulationStatistics::activeContactCount)
        .field("fluidIterationCount", &SimulationStatistics::fluidIterationCount)
        .field("ccdImpactCount", &SimulationStatistics::ccdImpactCount)
        .field("islandCount", &SimulationStatistics::islandCount)
        .field("solvedIslandCount", &SimulationStatistics::solvedIslandCount)
        .field("sleepingBodyCount", &SimulationStatistics::sleepingBodyCount)
        .field("solvedConstraintCount", &SimulationStatistics::solvedConstraintCount)
        .field("ccdIterationLimitReached", &SimulationStatistics::ccdIterationLimitReached);

    class_<Shape>("Shape");

    class_<Circle, base<Shape>>("Circle")
        .constructor<float>()
        .function("getArea", &Circle::GetArea)
        .function("getRadius", &Circle::GetRadius);

    class_<Polygon, base<Shape>>("Polygon")
        .class_function("makeBox", &CreateBox, allow_raw_pointers())
        .function("getArea", &Polygon::GetArea);

    class_<RigidBody>("RigidBody")
        .smart_ptr<RigidBodyPtr>("RigidBodyPtr")
        .function("applyForce", &RigidBody::ApplyForce)
        .function("applyTorque", &RigidBody::ApplyTorque)
        .function("getPosition", &RigidBody::GetPosition)
        .function("getOrientation", &RigidBody::GetOrientation)
        .function("getVelocity", &RigidBody::GetVelocity)
        .function("getAngularVelocity", &RigidBody::GetAngularVelocity)
        .function("getMass", &RigidBody::GetMass)
        .function("getId", &RigidBody::GetId)
        .function("getCollisionCategoryBits", &RigidBody::GetCollisionCategoryBits)
        .function("getCollisionMaskBits", &RigidBody::GetCollisionMaskBits)
        .function("isStatic", &RigidBody::IsStatic)
        .function("isAwake", &RigidBody::IsAwake)
        .function("wake", &RigidBody::Wake)
        .function("isCcdEnabled", &RigidBody::IsCcdEnabled)
        .function("setCcdEnabled", &RigidBody::SetCcdEnabled)
        .function("setPosition", &RigidBody::SetPosition)
        .function("setOrientation", &RigidBody::SetOrientation)
        .function("setVelocity", &RigidBody::SetVelocity)
        .function("setAngularVelocity", &RigidBody::SetAngularVelocity)
        .function("setMass", &RigidBody::SetMass)
        .function("setCollisionCategoryBits", &RigidBody::SetCollisionCategoryBits)
        .function("setCollisionMaskBits", &RigidBody::SetCollisionMaskBits);

    function("createRigidBody", &CreateRigidBody, allow_raw_pointers());
    class_<IJoint>("Joint")
        .smart_ptr<JointPtr>("JointPtr")
        .function("getAnchorA", &IJoint::getAnchorA)
        .function("getAnchorB", &IJoint::getAnchorB);
    class_<PrismaticJoint, base<IJoint>>("PrismaticJoint")
        .smart_ptr<std::shared_ptr<PrismaticJoint>>("PrismaticJointPtr")
        .function("getAxis", &PrismaticJoint::getAxis)
        .function("getLocalAxis", &PrismaticJoint::getLocalAxis)
        .function("getTranslation", &PrismaticJoint::getTranslation)
        .function("getTranslationSpeed", &PrismaticJoint::getTranslationSpeed)
        .function("getTransverseError", &PrismaticJoint::getTransverseError)
        .function("getAngle", &PrismaticJoint::getAngle)
        .function("getReferenceAngle", &PrismaticJoint::getReferenceAngle)
        .function("setMotor", &PrismaticJoint::setMotor)
        .function("isMotorEnabled", &PrismaticJoint::isMotorEnabled)
        .function("getMotorSpeed", &PrismaticJoint::getMotorSpeed)
        .function("getMaxMotorForce", &PrismaticJoint::getMaxMotorForce)
        .function("getMotorForce", &PrismaticJoint::getMotorForce)
        .function("setLimits", &PrismaticJoint::setLimits)
        .function("areLimitsEnabled", &PrismaticJoint::areLimitsEnabled)
        .function("getLowerLimit", &PrismaticJoint::getLowerLimit)
        .function("getUpperLimit", &PrismaticJoint::getUpperLimit);
    function("createPrismaticJoint", &CreatePrismaticJoint);
    class_<RevoluteJoint, base<IJoint>>("RevoluteJoint")
        .smart_ptr<std::shared_ptr<RevoluteJoint>>("RevoluteJointPtr")
        .function("setMotor", &RevoluteJoint::setMotor)
        .function("isMotorEnabled", &RevoluteJoint::isMotorEnabled)
        .function("getMotorSpeed", &RevoluteJoint::getMotorSpeed)
        .function("getMaxMotorTorque", &RevoluteJoint::getMaxMotorTorque)
        .function("getMotorTorque", &RevoluteJoint::getMotorTorque)
        .function("setLimits", &RevoluteJoint::setLimits)
        .function("areLimitsEnabled", &RevoluteJoint::areLimitsEnabled)
        .function("getLowerLimit", &RevoluteJoint::getLowerLimit)
        .function("getUpperLimit", &RevoluteJoint::getUpperLimit)
        .function("getAngle", &RevoluteJoint::getAngle);
    function("createDistanceJoint", &CreateDistanceJoint);
    function("createRevoluteJoint", &CreateRevoluteJoint);

    class_<ParticleSystem>("ParticleSystem")
        .smart_ptr<ParticleSystemPtr>("ParticleSystemPtr")
        .function("reserve", optional_override([](ParticleSystem& s, double count) { s.reserve(Wasm::Count(count)); }))
        .function("addParticle", &ParticleSystem::addParticle)
        .function("removeParticle", optional_override([](ParticleSystem& s, double index) { s.removeParticle(Wasm::Index(index, s.size())); }))
        .function("applyForce", optional_override([](ParticleSystem& s, double index, Vector2 force) { s.applyForce(Wasm::Index(index, s.size()), force); }))
        .function("clear", &ParticleSystem::clear)
        .function("step", &ParticleSystem::step)
        .function("setUniformAcceleration", &ParticleSystem::setUniformAcceleration)
        .function("getUniformAcceleration", &ParticleSystem::getUniformAcceleration)
        .function("size", &ParticleSystem::size)
        .function("empty", &ParticleSystem::empty)
        .function("getParticlePosition", optional_override([](
            const ParticleSystem& system,
            double index
        ) {
            return system.getParticles().at(Wasm::Index(index, system.size())).position;
        }))
        .function("getParticleVelocity", optional_override([](
            const ParticleSystem& system,
            double index
        ) {
            return system.getParticles().at(Wasm::Index(index, system.size())).velocity;
        }));

    function("createParticleSystem", &CreateParticleSystem);

    class_<Engine>("Engine")
        .constructor<>()
        .function("step", &Engine::step)
        .function("stepFixed", &Engine::stepFixed)
        .function("advance", &Engine::advance)
        .function("resetTiming", &Engine::resetTiming)
        .function("getAccumulatedTime", &Engine::getAccumulatedTime)
        .function("getTotalStepCount", &Engine::getTotalStepCount)
        .function("setSimulationConfig", &SetSimulationConfig)
        .function("getSimulationConfig", &GetSimulationConfig)
        .function("getLastStepStatistics", &Engine::getLastStepStatistics)
        .function("addBody", &Engine::addBody)
        .function("removeBody", &Engine::removeBody)
        .function("clearBodies", &Engine::clearBodies)
        .function("addJoint", &Engine::addJoint)
        .function("removeJoint", &Engine::removeJoint)
        .function("exportJson", &Engine::exportJson)
        .function("exportCsv", &Engine::exportCsv)
        .function("addParticleSystem", &Engine::addParticleSystem)
        .function("removeParticleSystem", &Engine::removeParticleSystem)
        .function("getMaterial", &Engine::getMaterial);
}

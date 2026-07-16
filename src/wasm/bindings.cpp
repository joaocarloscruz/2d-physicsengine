#include <emscripten/bind.h>

#include <memory>

#include "engine.h"
#include "physics/core/material.h"
#include "physics/core/particles/particle_system.h"
#include "physics/core/rigidbody.h"
#include "physics/core/shape.h"
#include "physics/math/vector2.h"

namespace PhysicsEngine {
namespace {

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

} // namespace
} // namespace PhysicsEngine

EMSCRIPTEN_BINDINGS(physics_engine) {
    using namespace emscripten;
    using namespace PhysicsEngine;

    value_object<Vector2>("Vector2")
        .field("x", &Vector2::x)
        .field("y", &Vector2::y);

    value_object<Material>("Material")
        .field("density", &Material::density)
        .field("restitution", &Material::restitution)
        .field("staticFriction", &Material::staticFriction)
        .field("dynamicFriction", &Material::dynamicFriction);

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
        .function("setPosition", &RigidBody::SetPosition)
        .function("setOrientation", &RigidBody::SetOrientation)
        .function("setVelocity", &RigidBody::SetVelocity)
        .function("setAngularVelocity", &RigidBody::SetAngularVelocity)
        .function("setMass", &RigidBody::SetMass)
        .function("setCollisionCategoryBits", &RigidBody::SetCollisionCategoryBits)
        .function("setCollisionMaskBits", &RigidBody::SetCollisionMaskBits);

    function("createRigidBody", &CreateRigidBody, allow_raw_pointers());

    class_<ParticleSystem>("ParticleSystem")
        .smart_ptr<ParticleSystemPtr>("ParticleSystemPtr")
        .function("reserve", &ParticleSystem::reserve)
        .function("addParticle", &ParticleSystem::addParticle)
        .function("removeParticle", &ParticleSystem::removeParticle)
        .function("applyForce", &ParticleSystem::applyForce)
        .function("clear", &ParticleSystem::clear)
        .function("step", &ParticleSystem::step)
        .function("setUniformAcceleration", &ParticleSystem::setUniformAcceleration)
        .function("getUniformAcceleration", &ParticleSystem::getUniformAcceleration)
        .function("size", &ParticleSystem::size)
        .function("empty", &ParticleSystem::empty)
        .function("getParticlePosition", optional_override([](
            const ParticleSystem& system,
            std::size_t index
        ) {
            return system.getParticles().at(index).position;
        }))
        .function("getParticleVelocity", optional_override([](
            const ParticleSystem& system,
            std::size_t index
        ) {
            return system.getParticles().at(index).velocity;
        }));

    function("createParticleSystem", &CreateParticleSystem);

    class_<Engine>("Engine")
        .constructor<>()
        .function("step", &Engine::step)
        .function("addBody", &Engine::addBody)
        .function("addParticleSystem", &Engine::addParticleSystem)
        .function("removeParticleSystem", &Engine::removeParticleSystem)
        .function("getMaterial", &Engine::getMaterial);
}

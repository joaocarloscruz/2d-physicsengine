#include <emscripten/bind.h>

#include <memory>

#include "engine.h"
#include "physics/core/material.h"
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
        .function("isStatic", &RigidBody::IsStatic)
        .function("setPosition", &RigidBody::SetPosition)
        .function("setOrientation", &RigidBody::SetOrientation)
        .function("setVelocity", &RigidBody::SetVelocity)
        .function("setAngularVelocity", &RigidBody::SetAngularVelocity)
        .function("setMass", &RigidBody::SetMass);

    function("createRigidBody", &CreateRigidBody, allow_raw_pointers());

    class_<Engine>("Engine")
        .constructor<>()
        .function("step", &Engine::step)
        .function("addBody", &Engine::addBody)
        .function("getMaterial", &Engine::getMaterial);
}

#pragma once
#include <emscripten/bind.h>
#include <memory>
#include "engine.h"

namespace PhysicsEngine::Wasm {
// Embind creates every JS Engine through this factory. The returned shared_ptr
// owns the engine together with its event recorder, so the recorder outlives
// every callback the World can make and is unregistered before it is freed.
std::shared_ptr<Engine> CreateEngine();
// Adds enableCollisionEvents, getCollisionEvents and the other event methods.
void AddCollisionEventMethods(emscripten::class_<Engine>& engineClass);
}

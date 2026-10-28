#include <emscripten/bind.h>

#include <memory>

#include "engine.h"
#include "physics/core/forces/gravity.h"
#include "physics/math/vector2.h"

// Engine::addUniversalForce takes an owned generator, which Embind cannot
// express; this exposes the one generator the JavaScript scenes need.
EMSCRIPTEN_BINDINGS(universal_forces) {
    using namespace emscripten;
    using namespace PhysicsEngine;
    // Adds a uniform-acceleration generator to the engine. Calls stack, and a
    // generator cannot be removed without clearing and rebuilding the Engine.
    function("addGravity", optional_override([](Engine& engine, Vector2 acceleration) {
        engine.addUniversalForce(std::make_unique<Gravity>(acceleration));
    }));
}

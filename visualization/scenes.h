#ifndef SCENES_H
#define SCENES_H

#include <cstddef>
#include <functional>
#include <string>
#include <vector>
#include "physics/physics.h"

namespace PhysicsEngine {
namespace Visualization {

// What a scene needs from the renderer after it has been built.
struct SceneRuntime {
    // Called before every fixed step with the elapsed scene time in seconds.
    // Scenes use it to drive controllers such as a reversing slider.
    std::function<void(World&, float)> beforeStep;
    // Scenes with a vehicle: called every step with a throttle in [-1, 1]
    // (positive is right). Their scene entry sets `controls`.
    std::function<void(float)> drive;
    // Extra HUD lines, for example a counter. May be empty.
    std::function<std::string()> status;
    // Sensors only: whether to draw this sensor in its highlighted colour.
    std::function<bool(const RigidBody&)> highlighted;
    // Called before the scene's bodies are cleared, so a scene can unregister
    // anything it gave the world, such as a collision listener.
    std::function<void(World&)> onUnload;
};

struct SceneInfo {
    const char* name;
    const char* description;
    float zoom;                  // pixels per meter
    Vector2 cameraCenter;
    SimulationConfig config;
    // Fills an empty world with bodies, joints and forces.
    SceneRuntime (*build)(World&);
    const char* controls = nullptr; // extra help line for this scene
};

// Scenes in key order: entry i is selected with number key i + 1, and the
// tenth with key 0.
const std::vector<SceneInfo>& sceneList();

} // namespace Visualization
} // namespace PhysicsEngine

#endif // SCENES_H

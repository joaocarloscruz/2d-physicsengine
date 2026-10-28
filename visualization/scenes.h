#ifndef SCENES_H
#define SCENES_H

#include <cstddef>
#include <functional>
#include <vector>
#include "physics/physics.h"

namespace PhysicsEngine {
namespace Visualization {

// What a scene needs from the renderer after it has been built.
struct SceneRuntime {
    // Called before every fixed step with the elapsed scene time in seconds.
    // Scenes use it to drive controllers such as a reversing slider.
    std::function<void(World&, float)> beforeStep;
};

struct SceneInfo {
    const char* name;
    const char* description;
    float zoom;                  // pixels per meter
    Vector2 cameraCenter;
    SimulationConfig config;
    // Fills an empty world with bodies, joints and forces.
    SceneRuntime (*build)(World&);
};

// Scenes in key order: entry i is selected with number key i + 1.
const std::vector<SceneInfo>& sceneList();

} // namespace Visualization
} // namespace PhysicsEngine

#endif // SCENES_H

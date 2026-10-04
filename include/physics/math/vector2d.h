#pragma once

namespace PhysicsEngine {
// Double-precision coordinates for modules whose scales exceed Vector2 precision.
struct Vector2d {
    double x = 0.0;
    double y = 0.0;
};
}

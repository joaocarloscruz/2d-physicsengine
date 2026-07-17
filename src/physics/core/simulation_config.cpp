#include "physics/core/simulation_config.h"

#include <cmath>
#include <stdexcept>

namespace PhysicsEngine {

void SimulationConfig::Validate() const {
    if (!std::isfinite(fixedTimeStep) || fixedTimeStep <= 0.0f) {
        throw std::invalid_argument("Fixed timestep must be positive and finite.");
    }
    if (maxSubstepsPerAdvance <= 0) {
        throw std::invalid_argument("Maximum substeps per advance must be positive.");
    }
    if (solverIterations <= 0) {
        throw std::invalid_argument("Solver iterations must be positive.");
    }
    if (!std::isfinite(positionCorrectionFactor)
        || positionCorrectionFactor < 0.0f
        || positionCorrectionFactor > 1.0f) {
        throw std::invalid_argument(
            "Position correction factor must be finite and between zero and one."
        );
    }
    if (!std::isfinite(penetrationSlop) || penetrationSlop < 0.0f) {
        throw std::invalid_argument(
            "Penetration slop must be finite and non-negative."
        );
    }
    if (!std::isfinite(warmStartFactor)
        || warmStartFactor < 0.0f
        || warmStartFactor > 1.0f) {
        throw std::invalid_argument(
            "Warm-start factor must be finite and between zero and one."
        );
    }
    if (enableLinearVelocityLimit
        && (!std::isfinite(maxLinearSpeed) || maxLinearSpeed <= 0.0f)) {
        throw std::invalid_argument(
            "Enabled maximum linear speed must be positive and finite."
        );
    }
    if (enableAngularVelocityLimit
        && (!std::isfinite(maxAngularSpeed) || maxAngularSpeed <= 0.0f)) {
        throw std::invalid_argument(
            "Enabled maximum angular speed must be positive and finite."
        );
    }
}

}

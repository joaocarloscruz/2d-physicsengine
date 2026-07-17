#include "physics/core/fixed_step_runner.h"

#include "physics/core/world.h"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace PhysicsEngine {

FixedStepRunner::FixedStepRunner(World& world) : world(world) {}

FixedStepResult FixedStepRunner::advance(double elapsedTime) {
    if (!std::isfinite(elapsedTime) || elapsedTime < 0.0) {
        throw std::invalid_argument(
            "Elapsed time must be finite and non-negative."
        );
    }

    const SimulationConfig& config = world.getSimulationConfig();
    const double fixedTimeStep = static_cast<double>(config.fixedTimeStep);
    const double tolerance = fixedTimeStep * 1e-6;
    if (elapsedTime > std::numeric_limits<double>::max() - accumulatedTime) {
        throw std::overflow_error("Elapsed-time accumulation overflowed.");
    }
    accumulatedTime += elapsedTime;

    std::uint32_t stepsPerformed = 0;
    while (stepsPerformed < static_cast<std::uint32_t>(config.maxSubstepsPerAdvance)
        && accumulatedTime + tolerance >= fixedTimeStep) {
        world.step();
        accumulatedTime -= fixedTimeStep;
        ++stepsPerformed;
        ++totalStepCount;
    }

    if (std::abs(accumulatedTime) <= tolerance) {
        accumulatedTime = 0.0;
    }

    const double fractionalRemainder = std::fmod(accumulatedTime, fixedTimeStep);
    return FixedStepResult{
        stepsPerformed,
        static_cast<double>(stepsPerformed) * fixedTimeStep,
        accumulatedTime,
        fractionalRemainder / fixedTimeStep
    };
}

void FixedStepRunner::reset() {
    accumulatedTime = 0.0;
    totalStepCount = 0;
}

double FixedStepRunner::getAccumulatedTime() const {
    return accumulatedTime;
}

std::uint64_t FixedStepRunner::getTotalStepCount() const {
    return totalStepCount;
}

}

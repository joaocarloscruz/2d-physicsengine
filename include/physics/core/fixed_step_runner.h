#ifndef FIXED_STEP_RUNNER_H
#define FIXED_STEP_RUNNER_H

#include <cstdint>

namespace PhysicsEngine {

class World;

struct FixedStepResult {
    std::uint32_t stepsPerformed = 0;
    double simulatedTime = 0.0;
    double remainingTime = 0.0;
    double interpolationAlpha = 0.0;
};

class FixedStepRunner {
public:
    explicit FixedStepRunner(World& world);

    FixedStepResult advance(double elapsedTime);
    void reset();

    double getAccumulatedTime() const;
    std::uint64_t getTotalStepCount() const;

private:
    World& world;
    double accumulatedTime = 0.0;
    std::uint64_t totalStepCount = 0;
};

}

#endif // FIXED_STEP_RUNNER_H

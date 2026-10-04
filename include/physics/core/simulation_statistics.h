#ifndef SIMULATION_STATISTICS_H
#define SIMULATION_STATISTICS_H

#include <cstdint>

namespace PhysicsEngine {

struct SimulationStatistics {
    std::uint32_t integratedBodyCount = 0;
    std::uint32_t integratedParticleCount = 0;
    std::uint32_t broadPhaseCandidateCount = 0;
    std::uint32_t narrowPhaseCandidateCount = 0;
    std::uint32_t resolvedContactCount = 0;
    std::uint32_t solverIterationCount = 0;
    std::uint32_t activeContactCount = 0;
    std::uint32_t fluidIterationCount = 0;
    std::uint32_t ccdImpactCount = 0;
    bool ccdIterationLimitReached = false;
    std::uint32_t islandCount = 0;
    std::uint32_t solvedIslandCount = 0;
    std::uint32_t sleepingBodyCount = 0;
    std::uint32_t solvedConstraintCount = 0;
};

}

#endif // SIMULATION_STATISTICS_H

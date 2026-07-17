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
};

}

#endif // SIMULATION_STATISTICS_H

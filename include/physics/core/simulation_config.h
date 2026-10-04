#ifndef SIMULATION_CONFIG_H
#define SIMULATION_CONFIG_H

namespace PhysicsEngine {

struct SimulationConfig {
    float fixedTimeStep = 1.0f / 60.0f;
    int maxSubstepsPerAdvance = 8;
    int solverIterations = 10;
    float positionCorrectionFactor = 0.8f;
    float penetrationSlop = 0.005f;
    float warmStartFactor = 0.8f;
    float restitutionVelocityThreshold = 1.0f;
    float velocityTolerance = 0.0001f;
    float maxPositionCorrection = 0.2f;
    bool enableLinearVelocityLimit = true;
    float maxLinearSpeed = 200.0f;
    bool enableAngularVelocityLimit = true;
    float maxAngularSpeed = 30.0f;
    int maximumCcdImpacts = 32;
    bool enableSleeping = false;
    float sleepEnergyThreshold = 0.0005f; // kinetic energy per unit mass
    float sleepTimeThreshold = 0.5f;

    void Validate() const;
};

}

#endif // SIMULATION_CONFIG_H

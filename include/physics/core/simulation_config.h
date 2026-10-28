#ifndef SIMULATION_CONFIG_H
#define SIMULATION_CONFIG_H

namespace PhysicsEngine {

struct SimulationConfig {
    float fixedTimeStep = 1.0f / 60.0f;
    int maxSubstepsPerAdvance = 8;
    int solverIterations = 10;
    float positionCorrectionFactor = 0.8f;
    float penetrationSlop = 0.005f;
    float warmStartFactor = 1.0f;
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
    // Opt-in: treat each solved tangential contact impulse, and the torque of the
    // complete contact impulse, as a force acting over the step. Bodies then move
    // by half of that velocity change times the step, matching the constant-load
    // position integration and removing static-friction creep. Normal settling
    // keeps the penetration-slop behavior. Not applied with CCD impacts.
    bool enableFrictionDisplacementCorrection = false;

    void Validate() const;
};

}

#endif // SIMULATION_CONFIG_H

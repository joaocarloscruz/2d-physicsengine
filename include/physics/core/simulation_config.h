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
    // Only used by the legacy step order (enableSpeculativeContacts = false);
    // the speculative step already integrates positions after the solve.
    bool enableFrictionDisplacementCorrection = false;
    // Step order. true: detect contacts at the start-of-step poses, including
    // speculative contacts closer than a margin (4 * penetrationSlop plus the
    // distance the bodies can cover this step), solve velocities, then
    // integrate positions and run position correction. false: the legacy order
    // (integrate, detect at the new poses, solve), which lets fast bodies sink
    // up to v * dt before their contact is seen.
    bool enableSpeculativeContacts = true;
    // Speculative order only. Substeps per step for groups of touching
    // bodies without joints: each substep integrates velocities, solves the
    // contacts as soft springs that push overlap out, integrates positions
    // and relaxes (Box2D v3's soft step). Groups whose contacts would sag
    // past min(penetrationSlop, 1% of the contact width) under the soft model,
    // and groups with joints, keep the single rigid solve, as does every group
    // with 1. In 1..64; solverIterations / solverSubsteps (at least one)
    // iterations run per substep, plus one relax pass.
    int solverSubsteps = 4;
    // Soft contact stiffness, capped at a quarter of the substep rate, and
    // its damping ratio.
    float contactHertz = 60.0f;
    float contactDampingRatio = 2.0f;
    // Fastest speed (m/s) at which a substep pushes overlapping bodies apart.
    float contactPushSpeed = 3.0f;

    void Validate() const;
};

}

#endif // SIMULATION_CONFIG_H

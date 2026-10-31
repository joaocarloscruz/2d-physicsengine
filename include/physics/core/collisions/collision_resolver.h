#ifndef COLLISION_RESOLVER_H
#define COLLISION_RESOLVER_H

#include "collision_manifold.h"
#include "../simulation_config.h"
#include <array>
#include <cstdint>

namespace PhysicsEngine {
    struct ContactImpulse {
        double normal = 0.0;
        double tangent = 0.0;
    };

    struct CachedContactImpulse {
        std::uint32_t featureId = 0;
        ContactImpulse impulse;
    };

    struct ContactImpulseCache {
        std::array<CachedContactImpulse, 2> contacts{};
        std::uint8_t contactCount = 0;
        // Substeps per step of the solve that produced the impulses; a
        // substepped impulse is the impulse of one substep.
        std::uint8_t impulseSubsteps = 1;
        // The contact's group takes the rigid single-pass solve: the soft
        // substep model would let it sag past its tolerance (sticky, with
        // hysteresis, so the choice does not flip every step).
        bool rigidSolve = false;
    };

    struct ContactConstraintPoint {
        Vector2 localAnchorA;
        Vector2 localAnchorB;
        float penetration = 0.0f;
        double normalMass = 0.0;
        double tangentMass = 0.0;
        double velocityBias = 0.0;
        std::uint32_t featureId = 0;
        ContactImpulse impulse;
        // Contact point relative to each body's center when the constraint
        // was prepared: the fixed levers of the substepped solve.
        double leverA[2]{};
        double leverB[2]{};
        // Largest accumulated normal impulse of any substep this step.
        double maxNormalImpulse = 0.0;
    };

    struct ContactConstraint {
        RigidBody* bodyA = nullptr;
        RigidBody* bodyB = nullptr;
        Vector2 normal;
        Vector2 tangent;
        std::array<ContactConstraintPoint, 2> points{};
        std::uint8_t pointCount = 0;
        float staticFriction = 0.0f;
        float dynamicFriction = 0.0f;
        float positionCorrectionFactor = 0.0f;
        float penetrationSlop = 0.0f;
        float maxPositionCorrection = 0.0f;
        float velocityTolerance = 0.0f;
        ContactImpulseCache* cache = nullptr;

        // Solver scratch, not contact state: the points' world positions for
        // the recorded bodies, poses and anchors. Velocity iterations leave
        // poses unchanged, so the positions are reused until any input differs.
        struct PointPositionCache {
            const RigidBody* bodyA = nullptr;
            const RigidBody* bodyB = nullptr;
            float poses[6]{};
            float anchors[8]{};
            std::uint8_t count = 0;
            bool valid = false;
            double positions[2][2]{};
        } pointPositions;

        // Solver scratch of the substepped solve, which keeps the levers,
        // anchors, masses and impulses' validity fixed through one step.
        struct SubstepCache {
            // The first warm start validated both bodies and the impulses;
            // within the step they change only through checked updates.
            bool validated = false;
            // The two-point normal block's matrix terms (they depend only on
            // the levers and the inverse masses).
            bool blockReady = false;
            double ca[2]{}, cb[2]{};
            double k11 = 0, k12 = 0, k22 = 0, rigidDeterminant = 0;
            // Diagonal and determinant of K plus the soft rows' compliance,
            // by soft rows (bit 0: point 0, bit 1: point 1), for softness
            // {massScale, impulseScale}.
            bool softReady = false;
            double softness[2]{};
            double diagonal[4][2]{};
            double determinants[4]{};
            // Each point's separation at the recorded poses.
            bool separationsValid = false;
            float poses[6]{};
            double separations[2]{};
        } substep;
    };

    class CollisionResolver {
    public:
        static void Resolve(const CollisionManifold& manifold);
        static void Resolve(
            const CollisionManifold& manifold,
            ContactImpulse& accumulatedImpulse,
            bool reduceWarmStart = false
        );
        static void WarmStart(const CollisionManifold& manifold, const ContactImpulse& accumulatedImpulse);

    private:
        friend class World;

        static void WarmStart(
            const CollisionManifold& manifold,
            ContactImpulseCache& cache
        );

        static void Resolve(
            const CollisionManifold& manifold,
            ContactImpulseCache& cache,
            const SimulationConfig& config,
            bool reduceWarmStart = false
        );

        static ContactConstraint PrepareConstraint(
            const CollisionManifold& manifold,
            ContactImpulseCache& cache,
            const SimulationConfig& config,
            // true when config is known valid (World validates its own
            // configuration when it is set), skipping the re-check
            bool configValidated = false
        );

        static void WarmStart(ContactConstraint& constraint);
        static void SolveVelocity(ContactConstraint& constraint);
        static bool SolvePosition(ContactConstraint& constraint);

        // Substepped soft contact solve (SimulationConfig::solverSubsteps > 1).
        // Overlap is pushed out by a damped spring: target speed biasRate *
        // overlap, at most maxPushSpeed, with the impulse scaled by massScale
        // and relaxed by impulseScale times the accumulated impulse.
        struct Softness {
            double biasRate = 0.0;
            double massScale = 1.0;
            double impulseScale = 0.0;
        };
        static Softness MakeSoftness(double hertz, double dampingRatio, double substep);
        // Updates and returns the contact's rigidSolve choice: whether the
        // soft model, carrying the cached load, would sag past
        // min(penetrationSlop, 1% of the manifold's width).
        static bool UpdateRigidSolve(ContactConstraint& constraint, const Softness& softness,
            int substeps);
        // The same, given SagSizeLimit of each dynamic body (ignored for a
        // static one), so a caller can compute it once per body.
        static bool UpdateRigidSolve(ContactConstraint& constraint, const Softness& softness,
            int substeps, double sizeLimitA, double sizeLimitB);
        // 1% of the body's size (the square root of its area).
        static double SagSizeLimit(const RigidBody& body);
        // Rescales the warm-start impulses to impulses per step / substeps.
        static void MatchImpulseSubsteps(ContactConstraint& constraint, int substeps);
        // Applies the accumulated impulses at the prepared levers.
        static void WarmStartSubstep(ContactConstraint& constraint);
        // useBias false is the relax pass: rigid rows and speculative gaps
        // only. gapRows false also leaves rows with an open gap unsolved.
        static void SolveSubstep(ContactConstraint& constraint, double inverseSubstep,
            const Softness& softness, double maxPushSpeed, bool useBias, bool gapRows);
    };
}

#endif // COLLISION_RESOLVER_H

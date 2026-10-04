#pragma once
#include <cstddef>
#include <vector>

namespace PhysicsEngine {
struct ElasticWaveGridConfig {
    std::size_t columns = 16, rows = 16;
    double spacingX = 1, spacingY = 1;
    double density = 1, lambda = 1, shearModulus = 1; // kg/m^3, Pa, Pa in SI.
    double cflSafety = .9, maxSubstep = .1;
    std::size_t maximumSubsteps = 10000, maximumCellVisits = 100000000;
    static constexpr std::size_t MaximumCells = 262144;
    static constexpr std::size_t MaximumSubsteps = 1000000;
    static constexpr std::size_t MaximumCellVisits = 1000000000;
    void Validate() const;
};
struct ElasticWaveState {
    // Row-major i+columns*j, periodic samples stored once, common physical time.
    // vx at (i*dx,(j+.5)*dy), vy at ((i+.5)*dx,j*dy),
    // sigmaXX/sigmaYY at ((i+.5)*dx,(j+.5)*dy), sigmaXY at (i*dx,j*dy).
    std::vector<double> vx, vy, sigmaXX, sigmaYY, sigmaXY;
};
struct ElasticWaveDiagnostics {
    double kineticEnergy = 0, strainEnergy = 0, totalEnergy = 0;
    double modifiedEnergy = 0, modifiedEnergyStep = 0, physicalEnergyUpperBound = 0;
    double meanVx = 0, meanVy = 0, meanSigmaXX = 0, meanSigmaYY = 0, meanSigmaXY = 0;
    double maxAbsVelocity = 0, maxAbsStress = 0, meanSigmaZZ = 0, maxAbsSigmaZZ = 0;
    // Local Saint-Venant defect in strain/length^2. Means are separate global
    // compatibility conditions for strains derived from periodic displacements.
    double compatibilityRms = 0, maxAbsCompatibility = 0;
    double time = 0, stableTimeStep = 0, lastSubstep = 0;
    std::size_t lastSubsteps = 0, lastCellVisits = 0;
};
struct ElasticWaveSpatialRates {
    // Acceleration at velocity faces (m/s^2); strain rates at normal-stress
    // cells and shear corners (1/s). Engineering shear rate is 2*epsXY_dot.
    std::vector<double> accelerationX, accelerationY, strainRateXX, strainRateYY,
        engineeringShearRate;
};
// Independent homogeneous isotropic small-strain plane-strain periodic solid.
// Energy is per unit out-of-plane depth. No World, contact, deformation tracking,
// material interfaces, forcing, damping, fracture or free-boundary coupling.
// Geometry/material/budgets are immutable; public snapshots own their storage.
class ElasticWaveGrid {
  public:
    explicit ElasticWaveGrid(const ElasticWaveGridConfig &config = {});
    ElasticWaveGridConfig getConfig() const { return config_; }
    ElasticWaveState getState() const { return state_; }
    ElasticWaveDiagnostics getDiagnostics() const { return diagnostics_; }
    double getCompressionalSpeed() const { return cp_; }
    double getShearSpeed() const { return cs_; }
    double getStableTimeStep() const { return limit_; }
    // Arbitrary finite, representable initial stresses are accepted and observed;
    // no projection onto strains from periodic displacements is performed.
    // Clock retained, last work/reference step reset. Failure retains everything.
    void setState(const ElasticWaveState &state);
    std::vector<double> getCompatibility() const;
    std::vector<double> getOutOfPlaneStress() const; // sigmaZZ=lambda*(epsXX+epsYY).
    ElasticWaveSpatialRates getSpatialRates() const;
    double getModifiedEnergy(double h) const; // h=0 physical; positive strict CFL.
    // Symmetric stress-half-kick/velocity-drift/stress-half-kick. Failed calls
    // preserve all state, clock and diagnostics. Zero is a complete no-op.
    void step(double dt);

  private:
    ElasticWaveGridConfig config_;
    ElasticWaveState state_;
    ElasticWaveDiagnostics diagnostics_;
    double ix_, iy_, inverseDensity_, bulk2_, inverseBulk2_, inverseShear_;
    double kineticScale_, traceScale_, deviatorScale_, shearScale_, rateTraceScale_,
        rateShearScale_;
    double cp_, cs_, rate_, limit_, zzFactor_;
    struct Strain {
        double xx, yy, xy;
    }; // xy is engineering shear gamma=2*epsXY.
    Strain strainRate(const ElasticWaveState &state, std::size_t i, std::size_t j) const;
    Strain strain(const ElasticWaveState &state, std::size_t k) const;
    double compatibilityAt(const ElasticWaveState &state, std::size_t i, std::size_t j) const;
    ElasticWaveDiagnostics measure(const ElasticWaveState &state, double h) const;
};
} // namespace PhysicsEngine

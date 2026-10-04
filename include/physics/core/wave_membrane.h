#pragma once
#include <cstddef>
#include <vector>

namespace PhysicsEngine {
enum class WaveBoundary { FixedZero, Periodic };

struct WaveMembraneConfig {
    double tension = 1; // N/m
    double surfaceDensity = 1; // kg/m^2
    double damping = 0; // gamma, 1/s; PDE velocity damping is -2*gamma*v.
    WaveBoundary boundary = WaveBoundary::FixedZero;
    double cflSafety = 0.9; // Strictly between zero and one.
    double maxSubstep = 0.01;
    std::size_t maxCells = 1000000;
    std::size_t maxSubsteps = 4096;
    std::size_t maxCellWork = 64000000; // Grid cells * accepted substeps.
};

struct WaveMembraneDiagnostics {
    double kineticEnergy = 0;
    double strainEnergy = 0;
    double totalEnergy = 0;
    double maxAbsDisplacement = 0;
    double maxAbsVelocity = 0;
    double time = 0;
    double stableTimeStep = 0;
    double lastSubstep = 0;
    std::size_t lastSubsteps = 0;
    std::size_t lastCellWork = 0;
};

// Uniform linear small-displacement membrane. Independent of World/Engine.
// Row-major index y*width+x. Geometry is immutable; observers cannot mutate it.
class WaveMembrane {
public:
    WaveMembrane(std::size_t width, std::size_t height, double spacingX, double spacingY,
                 const WaveMembraneConfig& config = {});
    std::size_t getWidth() const noexcept { return width_; }
    std::size_t getHeight() const noexcept { return height_; }
    double getSpacingX() const noexcept { return dx_; }
    double getSpacingY() const noexcept { return dy_; }
    const WaveMembraneConfig& getConfig() const noexcept { return config_; }
    void setConfig(const WaveMembraneConfig& config);
    const std::vector<double>& getDisplacements() const noexcept { return u_; }
    const std::vector<double>& getVelocities() const noexcept { return v_; }
    const std::vector<double>& getQueuedAccelerations() const noexcept { return loads_; }
    void setState(const std::vector<double>& displacement, const std::vector<double>& velocity);
    void setCellState(std::size_t x, std::size_t y, double displacement, double velocity = 0);
    void queueAcceleration(std::size_t x, std::size_t y, double acceleration);
    void clearAccelerations() noexcept;
    void clearAcceleration(std::size_t x, std::size_t y);
    double getStableTimeStep() const;
    WaveMembraneDiagnostics getDiagnostics() const;
    // Positive accepted dt consumes queued accelerations; zero and failures
    // retain them. Failed steps preserve all state and diagnostic counters.
    void step(double dt);
private:
    std::size_t width_, height_;
    double dx_, dy_;
    WaveMembraneConfig config_;
    std::vector<double> u_, v_, loads_;
    double time_ = 0, lastSubstep_ = 0;
    std::size_t lastSubsteps_ = 0, lastCellWork_ = 0;
    std::size_t index(std::size_t x, std::size_t y) const;
    bool fixed(std::size_t x, std::size_t y, WaveBoundary boundary) const;
    void validateState(const std::vector<double>& values, WaveBoundary boundary) const;
    WaveMembraneDiagnostics diagnostics(const std::vector<double>& displacement,
                                       const std::vector<double>& velocity) const;
};
}

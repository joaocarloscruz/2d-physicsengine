#include "physics/core/fluids/wcsph_solver.h"
#include "physics/core/fluids/dfsph_solver.h"
#include "physics/core/fluids/sph_kernels.h"
#include "physics/core/fluids/sph_kernels.h"
#include <chrono>
#include <fstream>
#include <iostream>

using namespace PhysicsEngine;
int main(int argc, char** argv) {
    try {
        std::ofstream file;
        if (argc > 1) { file.open(argv[1]); if (!file) throw std::runtime_error("Cannot open benchmark output."); }
        std::ostream& out = file.is_open() ? file : std::cout;
        out << "method,particles,tolerance,compression,compression_rate,density_error,absolute_density_rate,density_iterations,divergence_iterations,converged,milliseconds\n";
        for (int extent : {5, 10}) for (int method=0; method<4; ++method) {
            const float spacing = 0.1f, h = 0.25f;
            FluidParticleProperties properties;
            properties.smoothingLength = h; properties.viscosity = 0;
            properties.mass = properties.restDensity*spacing*spacing*SphKernels2D::SquareLatticeMassScale(spacing, h);
            std::vector<FluidParticle> particles;
            for (int y=-extent; y<=extent; ++y) for (int x=-extent; x<=extent; ++x) {
                const Vector2 p(x*spacing, y*spacing); particles.emplace_back(p, p*-1, properties);
            }
            WcsphConfig weak; weak.externalAcceleration = {}; weak.speedOfSound = method == 0 ? 5 : 20;
            DfsphConfig strong; strong.externalAcceleration = {}; strong.maximumIterations = 2000;
            strong.densityTolerance = 1e-4f;
            strong.divergenceTolerance = method == 2 ? 0.01f : 0.001f;
            std::unique_ptr<IFluidSolver> solver;
            if (method < 2) solver = std::make_unique<WcsphSolver>(h, weak);
            else solver = std::make_unique<DfsphSolver>(h, strong);
            const auto start = std::chrono::steady_clock::now();
            solver->step(particles, 0.01f);
            const double elapsed = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now()-start).count();
            const auto d = solver->getDiagnostics();
            out << (method == 0 ? "WCSPH-c5" : method == 1 ? "WCSPH-c20" : "DFSPH") << ',' << particles.size() << ','
                << (method < 2 ? 0 : strong.divergenceTolerance) << ',' << d.maximumCompression << ','
                << d.maximumCompressionRate << ',' << d.maximumDensityError << ',' << d.maximumAbsoluteDensityRate << ','
                << d.densityIterations << ',' << d.divergenceIterations << ',' << d.converged << ',' << elapsed << '\n';
        }
        if (!out) throw std::runtime_error("Failed to write benchmark output.");
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}

#include "physics/physics.h"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>

using namespace PhysicsEngine;

// A bounded observation of integration order, not an acceptance test for creep.
// Position correction is disabled to isolate advance-before-contact displacement.
int main() {
    try {
        std::ostringstream out;
        out << std::setprecision(17)
            << "{\"schemaVersion\":1,\"classification\":\"observation\","
               "\"positionCorrectionFactor\":0,\"warmStartFactor\":0,"
               "\"solverIterations\":64,\"friction\":1,\"velocityTolerance\":0,"
               "\"sleeping\":false,\"velocityCaps\":false,\"ccd\":false,"
               "\"idealEquilibriumDisplacement\":0,\"rows\":[";
        bool first=true;
        for (int steps : {120,240,480}) {
            SimulationConfig config;
            config.solverIterations=64;
            config.positionCorrectionFactor=0;
            config.warmStartFactor=0;
            config.velocityTolerance=0;
            config.enableLinearVelocityLimit=false;
            config.enableAngularVelocityLimit=false;
            World world(config);
            const float theta=3.14159265358979323846f/12, gravity=9.81f;
            const double sine=std::sin(double(theta)), cosine=std::cos(double(theta));
            const Vector2 normal{float(-sine),float(cosine)};
            auto floor=std::make_shared<RigidBody>(Polygon::MakeBox(20,1),Material{1,0,1,1},normal*-.5f,true);
            auto box=std::make_shared<RigidBody>(Polygon::MakeBox(1,1),Material{1,0,1,1},normal*.5f);
            floor->SetOrientation(theta); box->SetOrientation(theta); box->SetMass(1);
            const auto start=box->position;
            world.addBody(floor); world.addBody(box);
            world.addUniversalForce(std::make_unique<Gravity>(Vector2{0,-gravity}));
            const float dt=2.f/steps;
            double velocityDisplacement=0, peakSpeed=0, peakSpin=0;
            for (int i=0;i<steps;++i) {
                velocityDisplacement+=(-cosine*box->velocity.x-sine*box->velocity.y)*dt;
                world.step(dt);
                peakSpeed=std::max(peakSpeed,std::hypot(double(box->velocity.x),double(box->velocity.y)));
                peakSpin=std::max(peakSpin,std::abs(double(box->angularVelocity)));
            }
            const double displacement=-cosine*(double(box->position.x)-start.x)
                -sine*(double(box->position.y)-start.y);
            const double forceDisplacement=.5*gravity*sine*dt*dt*steps;
            const double remainder=displacement-velocityDisplacement-forceDisplacement;
            for (double value : {displacement,forceDisplacement,velocityDisplacement,remainder,peakSpeed,peakSpin})
                if (!std::isfinite(value)) throw std::runtime_error("Nonfinite contact integration observation");
            if (!first) out << ',';
            first=false;
            out << "{\"steps\":" << steps << ",\"dt\":" << double(dt)
                << ",\"duration\":" << double(dt)*steps << ",\"angle\":" << double(theta)
                << ",\"gravity\":" << double(gravity) << ",\"downhillDisplacement\":" << displacement
                << ",\"velocityAdvectionDisplacement\":" << velocityDisplacement
                << ",\"forceIntegrationDisplacement\":" << forceDisplacement
                << ",\"remainder\":" << remainder << ",\"peakPostSolveSpeed\":" << peakSpeed
                << ",\"peakPostSolveAngularSpeed\":" << peakSpin << '}';
        }
        std::cout << out.str() << "]}\n";
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}

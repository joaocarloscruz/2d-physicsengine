#include "experimental/planar_contact_interval.h"
#include <iomanip>
#include <iostream>
using namespace PlanarContactInterval;
int main() {
    try {
        std::cout << std::setprecision(17)
                  << "{\"classification\":\"bounded_planar_observation\","
                     "\"productionDefaultsChanged\":false,"
                     "\"worldBridge\":false,\"rows\":[";
        bool first = true;
        for (unsigned count : {1u, 2u, 4u, 8u, 16u, 32u}) {
            Input in;
            in.tangentVelocity = static_cast<double>(.2f);
            in.duration = .5 / count;
            const double initial = in.tangentVelocity;
            double external = 0, friction = 0, kinetic = 0, normalImpulse = 0, tangentImpulse = 0;
            for (unsigned i = 0; i < count; ++i) {
                const auto r = Advance(in);
                if (r.status != Status::Complete)
                    return 2;
                in.tangentPosition = r.tangentPosition;
                in.tangentVelocity = r.tangentVelocity;
                external += r.externalWork;
                friction += r.frictionWork;
                kinetic += r.kineticChange;
                normalImpulse += r.normalImpulse;
                tangentImpulse += r.tangentImpulse;
            }
            if (!first)
                std::cout << ',';
            first = false;
            std::cout << "{\"steps\":" << count << ",\"duration\":0.5,\"dt\":" << in.duration
                      << ",\"initialSpeed\":" << initial
                      << ",\"mass\":1,\"normalLoad\":-8,\"mu\":0.25"
                      << ",\"analyticStopTime\":" << initial / 2
                      << ",\"analyticDistance\":" << initial * initial / 4
                      << ",\"distance\":" << in.tangentPosition
                      << ",\"finalSpeed\":" << in.tangentVelocity
                      << ",\"normalImpulse\":" << normalImpulse
                      << ",\"tangentImpulse\":" << tangentImpulse
                      << ",\"externalWork\":" << external << ",\"frictionWork\":" << friction
                      << ",\"kineticChange\":" << kinetic
                      << ",\"workResidual\":" << kinetic - external - friction << '}';
        }
        std::cout << "]}\n";
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}

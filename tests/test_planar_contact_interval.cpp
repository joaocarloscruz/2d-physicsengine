#include "../benchmarks/experimental/planar_contact_interval.h"
#include "catch_amalgamated.hpp"
#include <array>
#include <limits>
using namespace PlanarContactInterval;
namespace {
void Near(double actual, double expected, double margin = 2e-14) {
    REQUIRE(actual == Catch::Approx(expected).epsilon(0).margin(margin));
}
void CheckWork(const Input &in, const Result &r) {
    double friction = 0, external = 0, kinetic = 0, normalImpulse = 0, tangentImpulse = 0;
    for (const auto &p : r.intervals) {
        REQUIRE(p.frictionWork <= 0);
        Near(p.externalWork, in.forceT * p.tangentDistance + in.forceN * p.normalDistance);
        Near(p.frictionWork, p.tangentForce * p.tangentDistance);
        Near(p.kineticChange, .5 * in.mass *
                                      (p.finalTangentVelocity * p.finalTangentVelocity -
                                       p.initialTangentVelocity * p.initialTangentVelocity) +
                                  .5 * in.mass *
                                      (p.finalNormalVelocity * p.finalNormalVelocity -
                                       p.initialNormalVelocity * p.initialNormalVelocity));
        Near(p.kineticChange, p.externalWork + p.frictionWork);
        if (p.mode != Mode::Free) {
            REQUIRE(p.leftNormal >= 0);
            REQUIRE(p.rightNormal >= 0);
            Near(p.leftNormal + p.rightNormal, p.normalForce);
            Near(p.leftTangent + p.rightTangent, p.tangentForce);
            REQUIRE(std::abs(p.leftTangent) <= in.staticFriction * p.leftNormal + 2e-14);
            REQUIRE(std::abs(p.rightTangent) <= in.staticFriction * p.rightNormal + 2e-14);
            Near(in.left * p.leftNormal + in.right * p.rightNormal + in.leverDepth * p.tangentForce,
                 -in.torque);
            REQUIRE(std::abs(p.tangentForce) <= in.staticFriction * p.normalForce + 2e-14);
        }
        friction += p.frictionWork;
        external += p.externalWork;
        kinetic += p.kineticChange;
        normalImpulse += p.normalForce * p.duration;
        tangentImpulse += p.tangentForce * p.duration;
    }
    Near(r.frictionWork, friction);
    Near(r.externalWork, external);
    Near(r.kineticChange, kinetic);
    Near(r.normalImpulse, normalImpulse);
    Near(r.tangentImpulse, tangentImpulse);
    Near(in.mass * (r.tangentVelocity - in.tangentVelocity),
         in.forceT * r.elapsed + r.tangentImpulse);
    Near(in.mass * (r.normalVelocity - in.normalVelocity), in.forceN * r.elapsed + r.normalImpulse);
    if (r.intervals.empty() || r.intervals[0].mode != Mode::Free)
        Near(in.torque * r.elapsed + r.spinImpulse, 0);
}
} // namespace
TEST_CASE("Planar interval stops at the analytic within-step distance", "[contact-interval]") {
    Input in;
    in.tangentVelocity = static_cast<double>(.2f);
    const double stop = in.tangentVelocity / 2,
                 distance = in.tangentVelocity * in.tangentVelocity / 4;
    for (double h : {.125, .25, .5, 1.0}) {
        in.duration = h;
        const auto r = Advance(in);
        REQUIRE(r.status == Status::Complete);
        REQUIRE(r.intervals.size() == 2);
        Near(r.intervals[0].duration, stop);
        Near(r.tangentPosition, distance);
        REQUIRE(r.tangentVelocity == 0);
        REQUIRE(r.gap == 0);
        REQUIRE(r.normalVelocity == 0);
        Near(r.normalImpulse, 8 * h);
        Near(r.tangentImpulse, -in.tangentVelocity);
        CheckWork(in, r);
    }
    in.duration = stop;
    const auto endpoint = Advance(in);
    REQUIRE(endpoint.intervals.size() == 1);
    REQUIRE(endpoint.tangentVelocity == 0);
    Near(endpoint.tangentPosition, distance);
    CheckWork(in, endpoint);
}
TEST_CASE("Planar interval matches continuous slip and timestep partitioning",
          "[contact-interval]") {
    Input in;
    in.tangentVelocity = 2;
    in.duration = .5;
    const auto r = Advance(in);
    Near(r.tangentPosition, .75);
    Near(r.tangentVelocity, 1);
    CheckWork(in, r);
    for (unsigned count : {2u, 4u, 8u, 32u}) {
        Input piece = in;
        piece.duration = in.duration / count;
        double friction = 0;
        for (unsigned i = 0; i < count; ++i) {
            const auto step = Advance(piece);
            CheckWork(piece, step);
            piece.tangentPosition = step.tangentPosition;
            piece.tangentVelocity = step.tangentVelocity;
            friction += step.frictionWork;
        }
        Near(piece.tangentPosition, r.tangentPosition);
        Near(piece.tangentVelocity, r.tangentVelocity);
        Near(friction, r.frictionWork);
    }
}
TEST_CASE("Planar interval signed stop and restart integrates both work pieces",
          "[contact-interval]") {
    for (double sign : {-1., 1.}) {
        Input in;
        in.tangentVelocity = sign * .2;
        in.forceT = -sign * 3;
        in.duration = .125;
        const auto r = Advance(in);
        REQUIRE(r.intervals.size() == 2);
        const double stopped = .2 / 5, remaining = .125 - stopped;
        Near(r.tangentPosition,
             sign * (.2 * stopped - .5 * 5 * stopped * stopped - .5 * remaining * remaining));
        Near(r.tangentVelocity, -sign * remaining);
        CheckWork(in, r);
        REQUIRE(r.intervals[0].tangentForce == -sign * 2);
        REQUIRE(r.intervals[1].tangentForce == sign * 2);
    }
}
TEST_CASE("Planar interval static load and complete support wrench are independent",
          "[contact-interval]") {
    Input in;
    in.forceT = 1;
    in.torque = .5;
    const auto r = Advance(in);
    REQUIRE(r.intervals[0].mode == Mode::Sticking);
    REQUIRE(r.tangentPosition == 0);
    REQUIRE(r.tangentVelocity == 0);
    // -tau-ell*T=0 => equal normals, not an unbalanced friction torque.
    Near(r.intervals[0].leftNormal, 4);
    Near(r.intervals[0].rightNormal, 4);
    Near(r.spinImpulse, -.5 * .125);
    CheckWork(in, r);
    in.torque = 0;
    const auto shifted = Advance(in);
    Near(shifted.intervals[0].leftNormal, 3.5);
    Near(shifted.intervals[0].rightNormal, 4.5);
    CheckWork(in, shifted);
    in.forceT = 2;
    REQUIRE(Advance(in).intervals[0].mode == Mode::Sticking);
    in.forceT = std::nextafter(2., 3.);
    REQUIRE(Advance(in).intervals[0].mode == Mode::Sliding);
}
TEST_CASE("Planar interval checks wrench again after a stop", "[contact-interval]") {
    Input in;
    in.tangentVelocity = .2;
    in.forceT = -3;
    in.left = -.05;
    in.right = .2;
    const auto r = Advance(in);
    REQUIRE(r.status == Status::UnsupportedWrench);
    REQUIRE(r.intervals.size() == 1);
    Near(r.elapsed, .04);
    REQUIRE(r.tangentVelocity == 0);
    // Initial T=-2 gives COP=.125; restarted T=+2 gives COP=-.125 (outside).
    Near(r.intervals[0].leftNormal, 2.4);
    Near(r.intervals[0].rightNormal, 5.6);
    CheckWork(in, r);
    in.left = -.2;
    in.right = .05;
    REQUIRE(Advance(in).intervals.empty());
}
TEST_CASE("Planar interval releases without changing ballistic accuracy", "[contact-interval]") {
    Input in;
    in.forceN = 2;
    in.forceT = 3;
    in.tangentVelocity = .7;
    in.duration = .25;
    const auto r = Advance(in);
    REQUIRE(r.status == Status::Complete);
    REQUIRE(r.intervals[0].mode == Mode::Free);
    Near(r.gap, .5 * 2 * .25 * .25);
    Near(r.normalVelocity, .5);
    Near(r.tangentPosition, .7 * .25 + .5 * 3 * .25 * .25);
    REQUIRE(r.normalImpulse == 0);
    REQUIRE(r.tangentImpulse == 0);
    CheckWork(in, r);
    in.forceN = 0;
    in.forceT = 0;
    in.tangentVelocity = 1;
    const auto noLoad = Advance(in);
    Near(noLoad.tangentPosition, .25);
    REQUIRE(noLoad.normalImpulse == 0);
    in.forceN = 2;
    in.torque = 1;
    const auto rotation = Advance(in);
    REQUIRE(rotation.status == Status::UnsupportedWrench);
    REQUIRE(rotation.elapsed == 0);
}
TEST_CASE("Planar interval returns closing events without inventing impact impulses",
          "[contact-interval]") {
    Input in;
    in.normalVelocity = 1;
    in.forceN = -2;
    in.duration = 2;
    const auto returning = Advance(in);
    REQUIRE(returning.status == Status::NeedsImpact);
    Near(returning.elapsed, 1);
    REQUIRE(returning.gap == 0);
    Near(returning.normalVelocity, -1);
    REQUIRE(returning.normalImpulse == 0);
    CheckWork(in, returning);
    in.gap = 1;
    in.normalVelocity = 0;
    const auto falling = Advance(in);
    REQUIRE(falling.status == Status::NeedsImpact);
    Near(falling.elapsed, 1);
    Near(falling.normalVelocity, -2);
    CheckWork(in, falling);
    in.normalVelocity = -2;
    in.forceN = 0;
    const auto linear = Advance(in);
    Near(linear.elapsed, .5);
    in.gap = 0;
    const auto immediate = Advance(in);
    REQUIRE(immediate.status == Status::NeedsImpact);
    REQUIRE(immediate.elapsed == 0);
    REQUIRE(immediate.intervals.empty());
    REQUIRE(immediate.normalVelocity == -2);
    // A tangent touch has no closing speed and requires no impulsive response.
    in.gap = 1;
    in.normalVelocity = -2;
    in.forceN = 2;
    const auto tangent = Advance(in);
    REQUIRE(tangent.status == Status::Complete);
    Near(tangent.gap, 1);
    Near(tangent.normalVelocity, 2);
}
TEST_CASE("Planar interval scales physical units without absolute eligibility floors",
          "[contact-interval]") {
    Input reference;
    reference.tangentVelocity = .2;
    const auto base = Advance(reference);
    for (double scale : {1e-100, 1e100}) {
        Input in = reference;
        in.mass *= scale;
        in.forceN *= scale;
        in.forceT *= scale;
        in.torque *= scale;
        const auto r = Advance(in);
        Near(r.tangentPosition, base.tangentPosition);
        Near(r.tangentVelocity, base.tangentVelocity);
        Near(r.normalImpulse / scale, base.normalImpulse);
        Near(r.frictionWork / scale, base.frictionWork);
    }
}
TEST_CASE("Planar interval rejects invalid and unrepresentable derived ranges",
          "[contact-interval]") {
    Input in;
    in.mass = 0;
    REQUIRE_THROWS_AS(Advance(in), std::invalid_argument);
    in = Input{};
    in.dynamicFriction = .5;
    REQUIRE_THROWS_AS(Advance(in), std::invalid_argument);
    in = Input{};
    in.gap = -1;
    REQUIRE_THROWS_AS(Advance(in), std::invalid_argument);
    in = Input{};
    in.duration = std::numeric_limits<double>::infinity();
    REQUIRE_THROWS_AS(Advance(in), std::invalid_argument);
    in = Input{};
    in.staticFriction = std::numeric_limits<double>::max();
    REQUIRE_THROWS_AS(Advance(in), std::overflow_error);
    in = Input{};
    in.gap = 1;
    in.normalVelocity = std::numeric_limits<double>::max();
    REQUIRE_THROWS_AS(Advance(in), std::overflow_error);
    in = Input{};
    in.mass = std::numeric_limits<double>::denorm_min();
    REQUIRE_THROWS_AS(Advance(in), std::overflow_error);
    in = Input{};
    in.duration = 0;
    REQUIRE(Advance(in).intervals.empty());
}

TEST_CASE("Planar interval resolves separating quadratic cancellation", "[contact-interval]") {
    for (auto [gap, velocity, force] :
         {std::array<double, 3>{1e-20, 1., -1.}, std::array<double, 3>{1e-100, 1e100, -1e100}}) {
        Input in;
        in.gap = gap;
        in.normalVelocity = velocity;
        in.forceN = force;
        in.duration = 3;
        const auto r = Advance(in);
        REQUIRE(r.status == Status::NeedsImpact);
        Near(r.elapsed, 2);
        REQUIRE(r.gap == 0);
        REQUIRE(r.normalVelocity == -velocity);
        REQUIRE(r.normalImpulse == 0);
        REQUIRE(r.eventGapCorrection == -gap);
        in.duration = 1;
        const auto before = Advance(in);
        REQUIRE(before.status == Status::Complete);
        REQUIRE(before.gap > 0);
    }
}

#include "physics/physics.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>

// A destructible wall: a grid of boxes held together by WeldJoints, hit by a
// heavy CCD ball. WeldJoint has no reaction-force getter, so the application
// estimates the load from the collision events instead. For each contact
// between two moving bodies it takes the approach speed along the event normal
// (from the velocities saved before the step) and the pair's reduced mass:
// impulse ~ mu * approach speed, the impulse of a perfectly inelastic hit
// between free bodies. A body whose estimate exceeds kBreakImpulse loses its
// welds after the step. The program exits nonzero if the wall does not break
// at the impact, breaks far from it, or misbehaves after joints are removed.
using namespace PhysicsEngine;

namespace {
constexpr int kColumns = 6, kRows = 5;
constexpr float kPitch = 0.5f, kBox = 0.49f; // a 1 cm gap, so neighbours never touch
constexpr double kBreakImpulse = 8.0;        // N s; a resting box or a gentle bump stays far below it
constexpr int kNearColumns = 3;              // the shot hits column 0; damage may reach column 2 at most

struct Weld {
    std::shared_ptr<WeldJoint> joint;
    std::uint64_t idA, idB;
    bool inCorner; // both boxes are in the upper-right corner, away from the shot
};

std::size_t countCornerWelds(const std::vector<Weld>& welds) {
    return static_cast<std::size_t>(std::count_if(welds.begin(), welds.end(), [](const Weld& w) { return w.inCorner; }));
}

// Keeps the solid contacts reported during one step. The application clears it
// before each step and reads it afterwards.
class ContactLog : public ICollisionListener {
public:
    void onCollisionBegin(const CollisionEvent& e) override { if (!e.sensor) events.push_back(e); }
    void onCollisionPersist(const CollisionEvent& e) override { if (!e.sensor) events.push_back(e); }
    std::vector<CollisionEvent> events;
};
}

int main() {
    World world;
    world.addUniversalForce(std::make_unique<Gravity>(Vector2{0, -9.81f}));
    const Material stone{6.0f, 0.0f, 0.6f, 0.5f};
    world.addBody(std::make_shared<RigidBody>(Polygon::MakeBox(40, 1), stone, Vector2{0, -0.5f}, true));

    // Box (c, r) is at x = 5 + c * pitch. Each box is welded to its right and upper neighbour.
    std::shared_ptr<RigidBody> boxes[kColumns][kRows];
    std::unordered_map<std::uint64_t, RigidBody*> moving; // id -> every dynamic body, for event lookups
    std::unordered_map<std::uint64_t, int> column;        // id -> wall column
    for (int c = 0; c < kColumns; ++c)
        for (int r = 0; r < kRows; ++r) {
            boxes[c][r] = std::make_shared<RigidBody>(Polygon::MakeBox(kBox, kBox), stone,
                Vector2{5.0f + c * kPitch, kPitch * (r + 0.5f)});
            world.addBody(boxes[c][r]);
            moving[boxes[c][r]->GetId()] = boxes[c][r].get();
            column[boxes[c][r]->GetId()] = c;
        }
    std::vector<Weld> welds;
    auto weld = [&](const std::shared_ptr<RigidBody>& a, const std::shared_ptr<RigidBody>& b, Vector2 anchorA, Vector2 anchorB, bool corner) {
        auto joint = std::make_shared<WeldJoint>(a, b, anchorA, anchorB); // both anchors on the shared edge
        world.addJoint(joint);
        welds.push_back({joint, a->GetId(), b->GetId(), corner});
    };
    for (int c = 0; c < kColumns; ++c)
        for (int r = 0; r < kRows; ++r) {
            if (c + 1 < kColumns) weld(boxes[c][r], boxes[c + 1][r], {kPitch / 2, 0}, {-kPitch / 2, 0}, c >= 3 && r >= 3);
            if (r + 1 < kRows) weld(boxes[c][r], boxes[c][r + 1], {0, kPitch / 2}, {0, -kPitch / 2}, c >= 3 && r >= 3);
        }
    const std::size_t weldCount = welds.size();
    // The upper-right corner is far from the shot (row 2, left face): its welds must all survive.
    const std::size_t cornerBefore = countCornerWelds(welds);

    // The ball covers 0.5 m per step, about one box width. CCD guarantees it cannot skip
    // a box; speculative contacts alone also catch it at this speed.
    const Vector2 ballStart{1, 1.0f};
    auto ball = std::make_shared<RigidBody>(Circle(0.3f), Material{10.0f, 0.0f, 0.5f, 0.4f}, ballStart);
    ball->SetCcdEnabled(true);
    world.addBody(ball);
    moving[ball->GetId()] = ball.get();
    ContactLog log;
    world.addCollisionListener(&log);

    const int launchStep = 60; // the wall settles first
    double settledWorstSeparation = 0;
    int removed = 0, removedFar = 0, firstRemovalStep = -1;
    for (int frame = 0; frame < 480; ++frame) {
        if (frame < launchStep) { ball->SetPosition(ballStart); ball->SetVelocity({0, 0}); } // held until the shot
        if (frame == launchStep) ball->SetVelocity({30, 0});
        std::unordered_map<std::uint64_t, Vector2> velocityBefore;
        for (const auto& [id, body] : moving) velocityBefore[id] = body->velocity;
        log.events.clear();
        world.step();
        if (frame < launchStep)
            for (const auto& w : welds) settledWorstSeparation = std::max(settledWorstSeparation, w.joint->getSeparation());

        std::unordered_set<std::uint64_t> overloaded;
        for (const auto& e : log.events) {
            const auto a = moving.find(e.bodyAId), b = moving.find(e.bodyBId);
            if (a == moving.end() || b == moving.end()) continue; // contact with the static floor
            const Vector2 dv = velocityBefore[e.bodyBId] - velocityBefore[e.bodyAId];
            const double approach = -(double(dv.x) * e.normal.x + double(dv.y) * e.normal.y); // normal points A to B
            if (approach <= 0) continue;
            const double reduced = 1.0 / (a->second->GetInverseMass() + b->second->GetInverseMass());
            const double impulse = reduced * approach;
            if (impulse > kBreakImpulse) {
                overloaded.insert(e.bodyAId);
                overloaded.insert(e.bodyBId);
                std::printf("  step %3d: bodies %llu-%llu approach %.1f m/s, impulse estimate %.1f N s\n", frame,
                    static_cast<unsigned long long>(e.bodyAId), static_cast<unsigned long long>(e.bodyBId), approach, impulse);
            }
        }
        // Remove after the step. Removing a joint wakes its bodies; the bodies stay in the world.
        for (auto it = welds.begin(); it != welds.end();) {
            if (overloaded.count(it->idA) || overloaded.count(it->idB)) {
                world.removeJoint(it->joint);
                it = welds.erase(it);
                ++removed;
                removedFar += std::min(column[it->idA], column[it->idB]) >= kNearColumns;
                if (firstRemovalStep < 0) firstRemovalStep = frame;
            } else {
                ++it;
            }
        }
    }
    world.removeCollisionListener(&log);

    const std::size_t cornerWelds = countCornerWelds(welds);
    bool finite = true;
    for (const auto& body : world.getBodies()) finite = finite && std::isfinite(body->position.x) && std::isfinite(body->position.y);
    std::printf("welds: %zu at start, %d removed (first at step %d, shot at %d), %zu left, joint list size %zu\n",
        weldCount, removed, firstRemovalStep, launchStep, welds.size(), world.getJoints().size());
    std::printf("worst weld separation before the shot %.5f m; removed welds reaching past column %d: %d\n",
        settledWorstSeparation, kNearColumns - 1, removedFar);
    std::printf("corner welds intact: %zu of %zu; ball ended at x=%.2f (wall front face at 4.76)\n",
        cornerWelds, cornerBefore, ball->position.x);

    const bool ok = finite && settledWorstSeparation < 0.002
        && removed >= 4 && removed < static_cast<int>(weldCount) / 3 // a local hole, not the whole wall
        && removedFar == 0 && cornerWelds == cornerBefore
        && firstRemovalStep >= launchStep && firstRemovalStep < launchStep + 20
        && world.getJoints().size() == welds.size() && world.getBodies().size() == 32
        && ball->position.x < 4.7f; // the CCD ball did not tunnel through the wall
    std::printf("%s\n", ok ? "destructible wall: ok" : "destructible wall: FAIL");
    return ok ? 0 : 1;
}

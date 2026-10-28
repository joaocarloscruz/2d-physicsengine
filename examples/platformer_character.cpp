#include "physics/physics.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <memory>

// A platformer character (see docs/sensors.md): a box on a floor and a raised
// platform, a sensor "foot" that tells the game whether the character is
// grounded, a jump allowed only while grounded, horizontal control that sets
// the velocity directly, and a one-time pickup sensor identified by user data.
// It exits nonzero if the character does not land, jump to the expected apex,
// refuse to double-jump, and collect the pickup exactly once.
using namespace PhysicsEngine;

namespace {
constexpr float kGravity = 9.81f;
constexpr float kJumpSpeed = 6.0f;
constexpr std::uint32_t kWorldBit = 0x1, kPlayerBit = 0x2, kFootBit = 0x4, kPickupBit = 0x8;
constexpr std::uintptr_t kCoinId = 7; // the application's own id for the pickup

// Counts what the foot touches and reports pickups. Events arrive after the
// step, so the game reads this state on its next frame.
class PlayerSensors : public ICollisionListener {
public:
    PlayerSensors(const RigidBody& foot, const RigidBody& coin) : footId(foot.GetId()), coinId(coin.GetId()) {}
    void onCollisionBegin(const CollisionEvent& event) override {
        if (!event.sensor) return;
        if (involves(event, footId)) ++groundContacts;
        if (involves(event, coinId)) { ++coinBegins; coinSeen = true; }
    }
    void onCollisionEnd(const CollisionEvent& event) override {
        if (event.sensor && involves(event, footId)) --groundContacts;
    }
    bool grounded() const { return groundContacts > 0; }
    int groundContacts = 0, coinBegins = 0;
    bool coinSeen = false;
private:
    static bool involves(const CollisionEvent& e, std::uint64_t id) { return e.bodyAId == id || e.bodyBId == id; }
    std::uint64_t footId, coinId;
};

std::shared_ptr<RigidBody> makeStatic(float w, float h, float x, float y) {
    // Restitution 0 so landings do not bounce (a contact uses the larger value).
    auto body = std::make_shared<RigidBody>(Polygon::MakeBox(w, h), Material{1.0f, 0.0f, 0.6f, 0.4f}, Vector2{x, y}, true);
    body->SetCollisionCategoryBits(kWorldBit);
    return body;
}
}

int main() {
    World world;
    world.addUniversalForce(std::make_unique<Gravity>(Vector2{0, -kGravity}));

    world.addBody(makeStatic(40, 1, 0, -0.5f));  // floor, top at y = 0
    world.addBody(makeStatic(4, 1, 8, 1.0f));    // platform, x in [6, 10], top at y = 1.5

    // Friction 0 on the character: a contact uses the geometric mean of both
    // frictions, so the character never sticks to a wall or fights the velocity
    // the game sets. It stops because the game sets its speed to 0.
    auto player = std::make_shared<RigidBody>(Polygon::MakeBox(0.6f, 1.0f), Material{1.0f, 0.0f, 0.0f, 0.0f}, Vector2{0, 3});
    player->SetCollisionCategoryBits(kPlayerBit);
    world.addBody(player);

    // A static sensor never reports against static bodies, so the foot is a
    // dynamic sensor. It copies the player's velocity every frame and so falls
    // with it; it only collides with the world, never with the player or coin.
    auto foot = std::make_shared<RigidBody>(Polygon::MakeBox(0.4f, 0.1f), Material{}, Vector2{0, 3});
    foot->SetSensor(true);
    foot->SetCollisionCategoryBits(kFootBit);
    foot->SetCollisionMaskBits(kWorldBit);
    world.addBody(foot);

    auto coin = std::make_shared<RigidBody>(Polygon::MakeBox(0.5f, 0.5f), Material{}, Vector2{3, 0.3f}, true);
    coin->SetSensor(true);
    coin->SetUserData(kCoinId);
    coin->SetCollisionCategoryBits(kPickupBit);
    coin->SetCollisionMaskBits(kPlayerBit);
    world.addBody(coin);

    PlayerSensors sensors(*foot, *coin);
    world.addCollisionListener(&sensors);

    int jumps = 0, rejectedJumpRequests = 0, score = 0;
    bool groundedAtRest = false;
    float runVelocity = 0, groundY = 0, apexY = 0, landY = 0;
    bool jumpHeld = false;
    const int jumpTestStart = 90, jumpHoldEnd = 155, jumpTestEnd = 240, runStart = 260;

    for (int frame = 0; frame < 600; ++frame) {
        // Game logic reads last step's events, then sets velocities for this step.
        if (frame == jumpTestStart - 1) { groundY = player->position.y; groundedAtRest = sensors.grounded(); }
        jumpHeld = (frame >= jumpTestStart && frame < jumpHoldEnd);  // held through the whole flight: asked every frame
        if (frame >= runStart && player->position.x >= 3.8f && player->position.x < 4.2f) jumpHeld = true;
        if (frame < 5) jumpHeld = true;                               // asked while still falling
        if (jumpHeld) {
            if (sensors.grounded()) {
                player->SetVelocity({player->velocity.x, kJumpSpeed});
                ++jumps;
            } else {
                ++rejectedJumpRequests;
            }
        }
        // Horizontal control: set the velocity, 0 when there is no input.
        runVelocity = (frame >= runStart && player->position.x < 8.0f) ? 4.0f : 0.0f;
        player->SetVelocity({runVelocity, player->velocity.y});
        foot->SetPosition({player->position.x, player->position.y - 0.53f});
        foot->SetVelocity(player->velocity);

        world.step();

        if (frame >= jumpTestStart && frame < jumpTestEnd) apexY = std::max(apexY, player->position.y);
        if (frame == jumpTestEnd - 1) landY = player->position.y;
        if (sensors.coinSeen && coin) { // one-time pickup: score, then remove the body
            score += static_cast<int>(coin->GetUserData() == kCoinId);
            world.removeBody(coin);
            coin.reset();
        }
    }
    world.removeCollisionListener(&sensors);

    const float expectedApex = kJumpSpeed * kJumpSpeed / (2 * kGravity);
    const float apex = apexY - groundY;
    std::printf("landed: y=%.4f (floor top 0, half height 0.5), grounded after landing=%d\n", groundY, int(sensors.groundContacts >= 0));
    std::printf("jump apex %.4f m, expected v^2/2g = %.4f m; back on the floor at y=%.4f\n", apex, expectedApex, landY);
    std::printf("jumps taken=%d (floor test 1 + platform 1), jump requests refused in the air=%d\n", jumps, rejectedJumpRequests);
    std::printf("pickup: begin events=%d, score=%d, bodies left=%zu\n", sensors.coinBegins, score, world.getBodies().size());
    std::printf("final: x=%.3f y=%.4f vx=%.4f grounded=%d (platform top 1.5 -> y=2.0)\n",
        player->position.x, player->position.y, player->velocity.x, int(sensors.grounded()));

    const bool ok = std::fabs(groundY - 0.5f) < 0.02f && groundedAtRest
        && std::fabs(apex - expectedApex) < 0.05f && std::fabs(landY - 0.5f) < 0.02f
        && jumps == 2 && rejectedJumpRequests > 20
        && sensors.coinBegins == 1 && score == 1 && world.getBodies().size() == 4
        && std::fabs(player->position.y - 2.0f) < 0.02f && std::fabs(player->velocity.x) < 0.01f
        && player->position.x > 7.5f && sensors.grounded();
    std::printf("%s\n", ok ? "platformer character: ok" : "platformer character: FAIL");
    return ok ? 0 : 1;
}

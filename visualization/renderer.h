#ifndef RENDERER_H
#define RENDERER_H

#include <SFML/Graphics.hpp>
#include <memory>
#include "physics/core/world.h"
#include "physics/core/rigidbody.h"
#include "physics/core/shape.h"

namespace PhysicsEngine {
namespace Visualization {

class Renderer {
public:
    Renderer(int width, int height, const std::string& title);
    ~Renderer();

    void setWorld(World* world);
    void run();

private:
    void processEvents();
    void update(float deltaTime);
    void render();
    
    void renderBody(const RigidBody* body);
    void renderCircle(const RigidBody* body);
    void renderPolygon(const RigidBody* body);
    
    Vector2 screenToWorld(const sf::Vector2i& screenPos) const;
    sf::Vector2f worldToScreen(const Vector2& worldPos) const;
    
    void handleMousePressed(sf::Mouse::Button button, const sf::Vector2i& position);
    void handleMouseReleased(sf::Mouse::Button button);
    void handleMouseMoved(const sf::Vector2i& position);
    
    void addCircle(const Vector2& position);
    void addBox(const Vector2& position);
    void setupInitialScene();

    sf::RenderWindow window;
    World* world;
    
    // Camera settings
    float zoom;
    Vector2 cameraCenter;
    
    // Interaction
    RigidBody* draggedBody;
    Vector2 dragOffset;
    float draggedBodyOriginalMass;
    bool isPaused;
    
    // Simulation
    sf::Clock clock;
    float timeAccumulator;
    const float fixedTimeStep = 1.0f / 60.0f;
    
    // Shapes (owned by renderer to keep them alive)
    std::vector<std::unique_ptr<Shape>> shapes;
    
    // Visual settings
    sf::Font font;
    bool showDebugInfo;
};

} // namespace Visualization
} // namespace PhysicsEngine

#endif // RENDERER_H

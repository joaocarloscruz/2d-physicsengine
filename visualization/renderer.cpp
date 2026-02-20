#include "renderer.h"
#include "physics/core/forces/gravity.h"
#include "physics/math/matrix2x2.h"
#include <iostream>
#include <cmath>
#include <optional>

namespace PhysicsEngine {
namespace Visualization {

Renderer::Renderer(int width, int height, const std::string& title)
    : window(sf::VideoMode(sf::Vector2u(width, height)), title),
      world(nullptr),
      zoom(40.0f), // 40 pixels per meter
      cameraCenter(0.0f, 0.0f),
      draggedBody(nullptr),
      draggedBodyOriginalMass(0.0f),
      isPaused(false),
      timeAccumulator(0.0f),
      showDebugInfo(true) {
    
    window.setVerticalSyncEnabled(true);
    window.setFramerateLimit(60);
}

Renderer::~Renderer() {}

void Renderer::setWorld(World* world) {
    this->world = world;
}

void Renderer::run() {
    if (!world) {
        std::cerr << "Error: World not set!" << std::endl;
        return;
    }
    
    setupInitialScene();
    clock.restart();
    
    while (window.isOpen()) {
        processEvents();
        
        float deltaTime = clock.restart().asSeconds();
        deltaTime = std::min(deltaTime, 0.1f); // Cap at 100ms to prevent spiral of death
        
        update(deltaTime);
        render();
    }
}

void Renderer::setupInitialScene() {
    // Add ground - wide platform
    shapes.push_back(std::make_unique<Polygon>(Polygon::MakeBox(20.0f, 2.0f)));
    Material groundMaterial = {1.0f, 0.1f};
    auto ground = std::make_shared<RigidBody>(shapes.back().get(), groundMaterial, Vector2(0.0f, -6.0f), true);
    world->addBody(ground);
    
    // Add left wall (thick)
    shapes.push_back(std::make_unique<Polygon>(Polygon::MakeBox(2.0f, 14.0f)));
    auto leftWall = std::make_shared<RigidBody>(shapes.back().get(), groundMaterial, Vector2(-11.0f, 0.0f), true);
    world->addBody(leftWall);
    
    // Add right wall (thick)
    shapes.push_back(std::make_unique<Polygon>(Polygon::MakeBox(2.0f, 14.0f)));
    auto rightWall = std::make_shared<RigidBody>(shapes.back().get(), groundMaterial, Vector2(11.0f, 0.0f), true);
    world->addBody(rightWall);
    
    // Add a couple of boxes on the ground
    shapes.push_back(std::make_unique<Polygon>(Polygon::MakeBox(1.0f, 1.0f)));
    Material boxMaterial = {1.0f, 0.1f};
    auto box1 = std::make_shared<RigidBody>(shapes.back().get(), boxMaterial, Vector2(-3.0f, -3.0f));
    world->addBody(box1);
    
    shapes.push_back(std::make_unique<Polygon>(Polygon::MakeBox(1.5f, 1.0f)));
    auto box2 = std::make_shared<RigidBody>(shapes.back().get(), boxMaterial, Vector2(2.0f, -3.0f));
    world->addBody(box2);
    
    // Add a circle
    shapes.push_back(std::make_unique<Circle>(0.5f));
    Material circleMaterial = {1.0f, 0.1f};
    auto circle1 = std::make_shared<RigidBody>(shapes.back().get(), circleMaterial, Vector2(0.0f, 0.0f));
    world->addBody(circle1);
    
    // Add gravity
    world->addUniversalForce(std::make_unique<Gravity>(Vector2(0.0f, -9.81f)));
}

void Renderer::processEvents() {
    while (std::optional<sf::Event> event = window.pollEvent()) {
        if (const auto* closed = event->getIf<sf::Event::Closed>()) {
            window.close();
        }
        else if (const auto* keyPressed = event->getIf<sf::Event::KeyPressed>()) {
            if (keyPressed->code == sf::Keyboard::Key::Escape) {
                window.close();
            } else if (keyPressed->code == sf::Keyboard::Key::Space) {
                isPaused = !isPaused;
            } else if (keyPressed->code == sf::Keyboard::Key::R) {
                // Reset scene
                world->clearBodies();
                shapes.clear();
                setupInitialScene();
            } else if (keyPressed->code == sf::Keyboard::Key::D) {
                showDebugInfo = !showDebugInfo;
            } else if (keyPressed->code == sf::Keyboard::Key::C) {
                // Add circle at mouse position
                sf::Vector2i mousePos = sf::Mouse::getPosition(window);
                addCircle(screenToWorld(mousePos));
            } else if (keyPressed->code == sf::Keyboard::Key::B) {
                // Add box at mouse position
                sf::Vector2i mousePos = sf::Mouse::getPosition(window);
                addBox(screenToWorld(mousePos));
            } else if (keyPressed->code == sf::Keyboard::Key::Left) {
                cameraCenter.x -= 1.0f / zoom;
            } else if (keyPressed->code == sf::Keyboard::Key::Right) {
                cameraCenter.x += 1.0f / zoom;
            } else if (keyPressed->code == sf::Keyboard::Key::Up) {
                cameraCenter.y += 1.0f / zoom;
            } else if (keyPressed->code == sf::Keyboard::Key::Down) {
                cameraCenter.y -= 1.0f / zoom;
            }
        }
        else if (const auto* mousePressed = event->getIf<sf::Event::MouseButtonPressed>()) {
            handleMousePressed(mousePressed->button, mousePressed->position);
        }
        else if (const auto* mouseReleased = event->getIf<sf::Event::MouseButtonReleased>()) {
            handleMouseReleased(mouseReleased->button);
        }
        else if (const auto* mouseMoved = event->getIf<sf::Event::MouseMoved>()) {
            handleMouseMoved(mouseMoved->position);
        }
        else if (const auto* mouseWheel = event->getIf<sf::Event::MouseWheelScrolled>()) {
            // Zoom in/out
            if (mouseWheel->delta > 0) {
                zoom *= 1.1f;
            } else {
                zoom *= 0.9f;
            }
            zoom = std::max(10.0f, std::min(zoom, 200.0f));
        }
    }
}

void Renderer::update(float deltaTime) {
    if (isPaused) return;
    
    // Fixed timestep simulation
    timeAccumulator += deltaTime;
    
    while (timeAccumulator >= fixedTimeStep) {
        world->step(fixedTimeStep);
        timeAccumulator -= fixedTimeStep;
    }
}

void Renderer::render() {
    window.clear(sf::Color(30, 30, 40));
    
    // Render all bodies
    for (const auto& body : world->getBodies()) {
        renderBody(body.get());
    }
    
    // Render debug info
    if (showDebugInfo) {
        if (font.openFromFile("C:/Windows/Fonts/arial.ttf") || 
            font.openFromFile("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf")) {
            
            std::string info = "Bodies: " + std::to_string(world->getBodies().size()) + "\n";
            info += "FPS: " + std::to_string(static_cast<int>(1.0f / clock.getElapsedTime().asSeconds())) + "\n";
            info += isPaused ? "[PAUSED]" : "[RUNNING]";
            info += "\n\nControls:\n";
            info += "Left Click: Drag bodies\n";
            info += "Right Click: Add Box at mouse\n";
            info += "C: Add Circle at mouse\n";
            info += "B: Add Box at mouse\n";
            info += "Arrow Keys: Move camera\n";
            info += "Mouse Wheel: Zoom\n";
            info += "Space: Pause/Resume\n";
            info += "R: Reset  D: Toggle debug";
            
            sf::Text text(font, info);
            text.setCharacterSize(14);
            text.setFillColor(sf::Color::White);
            text.setPosition(sf::Vector2f(10.f, 10.f));
            window.draw(text);
        }
    }
    
    window.display();
}

void Renderer::renderBody(const RigidBody* body) {
    if (!body) return;
    
    if (body->shape->type == ShapeType::CIRCLE) {
        renderCircle(body);
    } else if (body->shape->type == ShapeType::POLYGON) {
        renderPolygon(body);
    }
}

void Renderer::renderCircle(const RigidBody* body) {
    Circle* circle = static_cast<Circle*>(body->shape);
    float radius = circle->GetRadius();
    
    sf::CircleShape shape(radius * zoom);
    shape.setOrigin(sf::Vector2f(radius * zoom, radius * zoom));
    
    sf::Vector2f screenPos = worldToScreen(body->GetPosition());
    shape.setPosition(screenPos);
    
    // Color based on static/dynamic and dragged state
    if (body == draggedBody) {
        shape.setFillColor(sf::Color(255, 255, 100));  // Yellow when dragged
        shape.setOutlineThickness(3);
        shape.setOutlineColor(sf::Color(255, 255, 0));
    } else if (body->IsStatic()) {
        shape.setFillColor(sf::Color(100, 100, 100));
        shape.setOutlineThickness(2);
        shape.setOutlineColor(sf::Color(200, 200, 200));
    } else {
        shape.setFillColor(sf::Color(100, 150, 255));
        shape.setOutlineThickness(2);
        shape.setOutlineColor(sf::Color(200, 200, 200));
    }
    
    window.draw(shape);
    
    // Draw a line to show rotation
    sf::Vertex line[] = {
        {{screenPos.x, screenPos.y}, sf::Color::White},
        {worldToScreen(body->GetPosition() + Vector2(radius * cos(body->GetOrientation()), 
                                                                 radius * sin(body->GetOrientation()))),
         sf::Color::White}
    };
    window.draw(line, 2, sf::PrimitiveType::Lines);
}

void Renderer::renderPolygon(const RigidBody* body) {
    Polygon* polygon = static_cast<Polygon*>(body->shape);
    const std::vector<Vector2>& localVertices = polygon->getVertices();
    
    Matrix2x2 rot = Matrix2x2::rotation(body->GetOrientation());
    Vector2 pos = body->GetPosition();
    
    sf::ConvexShape shape;
    shape.setPointCount(localVertices.size());
    
    for (size_t i = 0; i < localVertices.size(); ++i) {
        Vector2 worldVertex = pos + (rot * localVertices[i]);
        shape.setPoint(i, worldToScreen(worldVertex));
    }
    
    // Color based on static/dynamic and dragged state
    if (body == draggedBody) {
        shape.setFillColor(sf::Color(255, 255, 100));  // Yellow when dragged
        shape.setOutlineThickness(3);
        shape.setOutlineColor(sf::Color(255, 255, 0));
    } else if (body->IsStatic()) {
        shape.setFillColor(sf::Color(80, 80, 80));
        shape.setOutlineThickness(2);
        shape.setOutlineColor(sf::Color(200, 200, 200));
    } else {
        shape.setFillColor(sf::Color(255, 150, 100));
        shape.setOutlineThickness(2);
        shape.setOutlineColor(sf::Color(200, 200, 200));
    }
    
    window.draw(shape);
}

Vector2 Renderer::screenToWorld(const sf::Vector2i& screenPos) const {
    float worldX = (screenPos.x - static_cast<int>(window.getSize().x) / 2.0f) / zoom + cameraCenter.x;
    float worldY = -(screenPos.y - static_cast<int>(window.getSize().y) / 2.0f) / zoom + cameraCenter.y;
    return Vector2(worldX, worldY);
}

sf::Vector2f Renderer::worldToScreen(const Vector2& worldPos) const {
    float screenX = (worldPos.x - cameraCenter.x) * zoom + static_cast<float>(window.getSize().x) / 2.0f;
    float screenY = -(worldPos.y - cameraCenter.y) * zoom + static_cast<float>(window.getSize().y) / 2.0f;
    return sf::Vector2f(screenX, screenY);
}

void Renderer::handleMousePressed(sf::Mouse::Button button, const sf::Vector2i& position) {
    Vector2 worldPos = screenToWorld(position);
    
    if (button == sf::Mouse::Button::Left) {
        // Try to grab a body (iterate in reverse so we grab top-most)
        auto& bodies = world->getBodies();
        for (auto it = bodies.rbegin(); it != bodies.rend(); ++it) {
            const auto& body = *it;
            if (body->IsStatic()) continue;
            
            bool hit = false;
            
            if (body->shape->type == ShapeType::CIRCLE) {
                Circle* circle = static_cast<Circle*>(body->shape);
                Vector2 diff = body->GetPosition() - worldPos;
                float distSq = diff.magnitudeSquared();
                hit = (distSq < circle->GetRadius() * circle->GetRadius());
            } else if (body->shape->type == ShapeType::POLYGON) {
                // Better polygon hit test using point-in-polygon
                Polygon* polygon = static_cast<Polygon*>(body->shape);
                const std::vector<Vector2>& localVertices = polygon->getVertices();
                Matrix2x2 rot = Matrix2x2::rotation(body->GetOrientation());
                Vector2 pos = body->GetPosition();
                
                // Transform world point to local space
                Vector2 localPoint = rot.inverse() * (worldPos - pos);
                
                // Simple AABB check first
                float minX = 1e10f, maxX = -1e10f, minY = 1e10f, maxY = -1e10f;
                for (const auto& v : localVertices) {
                    minX = std::min(minX, v.x);
                    maxX = std::max(maxX, v.x);
                    minY = std::min(minY, v.y);
                    maxY = std::max(maxY, v.y);
                }
                
                hit = (localPoint.x >= minX && localPoint.x <= maxX && 
                       localPoint.y >= minY && localPoint.y <= maxY);
            }
            
            if (hit) {
                draggedBody = body.get();
                dragOffset = body->GetPosition() - worldPos;
                
                // Make body kinematic (infinite mass) so it doesn't collide
                draggedBodyOriginalMass = draggedBody->GetMass();
                draggedBody->SetMass(0.0f);  // Infinite mass (inverseMass = 0)
                draggedBody->SetVelocity(Vector2(0, 0));
                draggedBody->SetAngularVelocity(0);
                break;
            }
        }
    } else if (button == sf::Mouse::Button::Right) {
        addBox(worldPos);
    } else if (button == sf::Mouse::Button::Middle) {
        addCircle(worldPos);
    }
}

void Renderer::handleMouseReleased(sf::Mouse::Button button) {
    if (button == sf::Mouse::Button::Left && draggedBody) {
        // Restore original mass (this also recalculates inertia)
        draggedBody->SetMass(draggedBodyOriginalMass);
        draggedBody = nullptr;
    }
}

void Renderer::handleMouseMoved(const sf::Vector2i& position) {
    if (draggedBody) {
        Vector2 worldPos = screenToWorld(position);
        draggedBody->SetPosition(worldPos + dragOffset);
        draggedBody->SetVelocity(Vector2(0, 0));
        draggedBody->SetAngularVelocity(0);
    }
}

void Renderer::addCircle(const Vector2& position) {
    shapes.push_back(std::make_unique<Circle>(0.5f));
    Material material = {1.0f, 0.2f};
    auto circle = std::make_shared<RigidBody>(shapes.back().get(), material, position);
    world->addBody(circle);
}

void Renderer::addBox(const Vector2& position) {
    shapes.push_back(std::make_unique<Polygon>(Polygon::MakeBox(1.0f, 1.0f)));
    Material material = {1.0f, 0.2f};
    auto box = std::make_shared<RigidBody>(shapes.back().get(), material, position);
    world->addBody(box);
}

} // namespace Visualization
} // namespace PhysicsEngine

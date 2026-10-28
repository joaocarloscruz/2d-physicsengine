#include "renderer.h"
#include "physics/core/forces/gravity.h"
#include "physics/math/matrix2x2.h"
#include "physics/core/spatial_queries.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <optional>
#include <stdexcept>

namespace PhysicsEngine {
namespace Visualization {

Renderer::Renderer(int width, int height, const std::string& title, bool visible)
    : window(sf::VideoMode(sf::Vector2u(width, height)), title),
      world(nullptr),
      zoom(40.0f), // 40 pixels per meter
      cameraCenter(0.0f, 0.0f),
      draggedBody(nullptr),
      isPaused(false),
      timeAccumulator(0.0f),
      showDebugInfo(true) {

    window.setVisible(visible);
    window.setVerticalSyncEnabled(true);
    window.setFramerateLimit(60);

    // SFML 3 ships no font. Try common system locations; without one no text is
    // drawn and the scene name goes to the window title instead.
    const char* candidates[] = {
        "C:/Windows/Fonts/segoeui.ttf",
        "C:/Windows/Fonts/arial.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf",
        "/System/Library/Fonts/Supplemental/Arial.ttf",
        "/System/Library/Fonts/Helvetica.ttc",
    };
    for (const char* path : candidates) {
        if (font.openFromFile(path)) {
            fontAvailable = true;
            break;
        }
    }
}

Renderer::~Renderer() {}

void Renderer::setWorld(World* world) {
    this->world = world;
}

void Renderer::run(std::size_t frameLimit, const std::string& capturePath) {
    if (!world) {
        std::cerr << "Error: World not set!" << std::endl;
        return;
    }
    if (sceneIndex >= sceneList().size()) {
        throw std::out_of_range("Scene number out of range.");
    }

    loadScene(sceneIndex, true);
    for (std::size_t i = 0; i < warmupSteps; ++i) stepOnce();
    clock.restart();

    std::size_t frames = 0;
    while (window.isOpen()) {
        processEvents();
        if (!window.isOpen()) break;

        float deltaTime = clock.restart().asSeconds();
        lastFrameTime = deltaTime;
        deltaTime = std::min(deltaTime, 0.1f); // Cap at 100ms to prevent spiral of death

        update(deltaTime);
        render();
        if (frameLimit && ++frames >= frameLimit) {
            if (!capturePath.empty()) {
                sf::Texture capture(window.getSize());
                capture.update(window);
                if (!capture.copyToImage().saveToFile(capturePath))
                    throw std::runtime_error("Could not save visualizer capture.");
            }
            window.close();
        }
    }
}

void Renderer::loadScene(std::size_t index, bool resetCamera) {
    const SceneInfo& scene = sceneList().at(index);
    draggedBody = nullptr;
    world->clearBodies();
    world->setSimulationConfig(scene.config);
    sceneRuntime = scene.build(*world);
    sceneIndex = index;
    sceneTime = 0.0f;
    timeAccumulator = 0.0f;
    stepMilliseconds = 0.0f;
    if (resetCamera) {
        zoom = scene.zoom;
        cameraCenter = scene.cameraCenter;
    }
    window.setTitle("2D Physics Engine - " + std::string(scene.name));
}

namespace {
// 1-9 from the number row or the keypad, else 0.
int sceneNumber(sf::Keyboard::Key key) {
    const int code = static_cast<int>(key);
    const int row = code - static_cast<int>(sf::Keyboard::Key::Num0);
    const int pad = code - static_cast<int>(sf::Keyboard::Key::Numpad0);
    if (row >= 1 && row <= 9) return row;
    if (pad >= 1 && pad <= 9) return pad;
    return 0;
}
}

void Renderer::processEvents() {
    while (std::optional<sf::Event> event = window.pollEvent()) {
        if (event->is<sf::Event::Closed>()) {
            window.close();
        }
        else if (const auto* keyPressed = event->getIf<sf::Event::KeyPressed>()) {
            const int number = sceneNumber(keyPressed->code);
            if (keyPressed->code == sf::Keyboard::Key::Escape) {
                window.close();
            } else if (keyPressed->code == sf::Keyboard::Key::Space) {
                isPaused = !isPaused;
            } else if (keyPressed->code == sf::Keyboard::Key::R) {
                loadScene(sceneIndex, false);
            } else if (keyPressed->code == sf::Keyboard::Key::D) {
                showDebugInfo = !showDebugInfo;
            } else if (keyPressed->code == sf::Keyboard::Key::H) {
                showHelp = !showHelp;
            } else if (number > 0 && static_cast<std::size_t>(number) <= sceneList().size()) {
                loadScene(static_cast<std::size_t>(number) - 1, true);
            } else if (keyPressed->code == sf::Keyboard::Key::C) {
                addCircle(screenToWorld(sf::Mouse::getPosition(window)));
            } else if (keyPressed->code == sf::Keyboard::Key::B) {
                addBox(screenToWorld(sf::Mouse::getPosition(window)));
            } else if (keyPressed->code == sf::Keyboard::Key::K) {
                addCapsule(screenToWorld(sf::Mouse::getPosition(window)));
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
        stepOnce();
        timeAccumulator -= fixedTimeStep;
    }
}

void Renderer::stepOnce() {
    if (sceneRuntime.beforeStep) sceneRuntime.beforeStep(*world, sceneTime);
    if (draggedBody) applyDragSpring();
    const auto start = std::chrono::steady_clock::now();
    world->step(fixedTimeStep);
    const float elapsed =
        std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - start).count();
    stepMilliseconds = stepMilliseconds == 0.0f
        ? elapsed : stepMilliseconds + 0.1f * (elapsed - stepMilliseconds);
    sceneTime += fixedTimeStep;
}

// Pull the grab point toward the mouse with a critically damped spring so the
// dragged body still collides, swings about the grab point and keeps its
// velocity when released, instead of being teleported through its neighbours.
void Renderer::applyDragSpring() {
    constexpr float frequency = 5.0f; // Hz
    const float omega = 2.0f * 3.14159265f * frequency;
    const float mass = draggedBody->GetMass();
    const Vector2 target = screenToWorld(sf::Mouse::getPosition(window));
    const Vector2 arm = Matrix2x2::rotation(draggedBody->GetOrientation()) * dragLocalAnchor;
    const Vector2 anchor = draggedBody->GetPosition() + arm;
    const Vector2 anchorVelocity = draggedBody->GetVelocityAtPoint(anchor);
    Vector2 force = ((target - anchor) * (omega * omega) - anchorVelocity * (2.0f * omega)) * mass;
    const float limit = 1000.0f * mass;
    if (force.magnitude() > limit) force = force.normalized() * limit;
    draggedBody->ApplyForce(force);
    draggedBody->ApplyTorque(arm.cross(force));
}

void Renderer::render() {
    window.clear(sf::Color(30, 30, 40));

    // Render all bodies
    for (const auto& body : world->getBodies()) {
        renderBody(body.get());
    }
    renderJoints();
    renderHud();

    window.display();
}

void Renderer::renderJoints() {
    const sf::Color color(255, 225, 110);
    for (const auto& joint : world->getJoints()) {
        const sf::Vector2f a = worldToScreen(joint->getAnchorA());
        const sf::Vector2f b = worldToScreen(joint->getAnchorB());
        const sf::Vertex line[] = {{a, color}, {b, color}};
        window.draw(line, 2, sf::PrimitiveType::Lines);

        // Anchor markers: round for revolute joints, square for the others.
        const bool revolute = dynamic_cast<const RevoluteJoint*>(joint.get()) != nullptr;
        for (const sf::Vector2f& point : {a, b}) {
            sf::CircleShape marker(3.5f, revolute ? 16 : 4);
            marker.setOrigin(sf::Vector2f(3.5f, 3.5f));
            marker.setPosition(point);
            marker.setFillColor(sf::Color(30, 30, 40));
            marker.setOutlineThickness(1.5f);
            marker.setOutlineColor(color);
            window.draw(marker);
        }
    }
}

// Draws text on a translucent backing. With anchorBottom, position is the
// bottom-left corner instead of the top-left.
void Renderer::drawLabel(const std::string& content, sf::Vector2f position, unsigned size, bool anchorBottom) {
    sf::Text text(font, content);
    text.setCharacterSize(size);
    text.setFillColor(sf::Color::White);
    const sf::FloatRect bounds = text.getLocalBounds();
    if (anchorBottom) position.y -= bounds.position.y + bounds.size.y + 8.0f;
    text.setPosition(position);
    sf::RectangleShape backing(sf::Vector2f(bounds.position.x + bounds.size.x + 12.0f,
                                            bounds.position.y + bounds.size.y + 8.0f));
    backing.setPosition(position - sf::Vector2f(4.0f, 2.0f));
    backing.setFillColor(sf::Color(15, 15, 22, 170));
    window.draw(backing);
    window.draw(text);
}

void Renderer::renderHud() {
    if (!showDebugInfo || !fontAvailable) return;
    const SceneInfo& scene = sceneList()[sceneIndex];

    char timing[48];
    std::snprintf(timing, sizeof(timing), "Step: %.2f ms", stepMilliseconds);
    std::string info = "Scene " + std::to_string(sceneIndex + 1) + ": " + scene.name + "\n";
    info += scene.description;
    info += "\nBodies: " + std::to_string(world->getBodies().size());
    info += "   Joints: " + std::to_string(world->getJoints().size());
    info += "\n" + std::string(timing);
    info += "   FPS: " + std::to_string(static_cast<int>(1.0f / std::max(lastFrameTime, 1e-6f)));
    info += isPaused ? "\n[PAUSED]" : "\n[RUNNING]";
    if (showHelp) {
        info += "\n\n1-8: Select scene   R: Reset scene\n";
        info += "Left drag: Grab body   Right click: Box\n";
        info += "Middle click / C: Circle   B: Box   K: Capsule\n";
        info += "Space: Pause   Arrows: Camera   Wheel: Zoom\n";
        info += "H: Hide help   D: Hide HUD   Esc: Quit";
    } else {
        info += "\nH: Show help";
    }
    drawLabel(info, sf::Vector2f(10.0f, 8.0f), 14);

    std::string tabs;
    for (std::size_t i = 0; i < sceneList().size(); ++i) {
        const std::string label = std::to_string(i + 1) + " " + sceneList()[i].name;
        tabs += i == sceneIndex ? "[" + label + "]  " : label + "  ";
    }
    drawLabel(tabs, sf::Vector2f(10.0f, static_cast<float>(window.getSize().y) - 6.0f), 13, true);
}

void Renderer::renderBody(const RigidBody* body) {
    if (!body) return;

    if (body->shape->type == ShapeType::CIRCLE) {
        renderCircle(body);
    } else if (body->shape->type == ShapeType::POLYGON) {
        renderPolygon(body);
    } else if (body->shape->type == ShapeType::CAPSULE) {
        renderCapsule(body);
    }
}

void Renderer::renderCircle(const RigidBody* body) {
    const Circle* circle = static_cast<const Circle*>(body->shape.get());
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
    const Polygon* polygon = static_cast<const Polygon*>(body->shape.get());
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

void Renderer::renderCapsule(const RigidBody* body) {
    const Capsule* capsule = static_cast<const Capsule*>(body->shape.get());
    const float halfLength = capsule->GetHalfLength();
    const float radius = capsule->GetRadius();
    const float orientation = body->GetOrientation();
    const Vector2 axis(std::cos(orientation), std::sin(orientation));
    const Vector2 side(-axis.y, axis.x);
    const Vector2 pos = body->GetPosition();

    // Each cap is a semicircle around its spine end, traced counterclockwise.
    constexpr std::size_t capSegments = 16;
    sf::ConvexShape shape;
    shape.setPointCount(2 * (capSegments + 1));
    std::size_t point = 0;
    for (int end = 0; end < 2; ++end) {
        const float sign = end == 0 ? 1.0f : -1.0f;
        const Vector2 center = pos + axis * (sign * halfLength);
        for (std::size_t i = 0; i <= capSegments; ++i) {
            const float angle = -1.5707963f + 3.14159265f * static_cast<float>(i) / capSegments;
            const Vector2 offset = (axis * std::cos(angle) + side * std::sin(angle)) * (sign * radius);
            shape.setPoint(point++, worldToScreen(center + offset));
        }
    }

    if (body == draggedBody) {
        shape.setFillColor(sf::Color(255, 255, 100));  // Yellow when dragged
        shape.setOutlineThickness(3);
        shape.setOutlineColor(sf::Color(255, 255, 0));
    } else if (body->IsStatic()) {
        shape.setFillColor(sf::Color(80, 80, 80));
        shape.setOutlineThickness(2);
        shape.setOutlineColor(sf::Color(200, 200, 200));
    } else {
        shape.setFillColor(sf::Color(120, 210, 140));
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
                const Circle* circle = static_cast<const Circle*>(body->shape.get());
                Vector2 diff = body->GetPosition() - worldPos;
                float distSq = diff.magnitudeSquared();
                hit = (distSq < circle->GetRadius() * circle->GetRadius());
            } else if (body->shape->type == ShapeType::POLYGON) {
                // Better polygon hit test using point-in-polygon
                const Polygon* polygon = static_cast<const Polygon*>(body->shape.get());
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
            } else if (body->shape->type == ShapeType::CAPSULE) {
                hit = ContainsPoint(*body, worldPos);
            }

            if (hit) {
                draggedBody = body.get();
                dragLocalAnchor = Matrix2x2::rotation(-body->GetOrientation())
                    * (worldPos - body->GetPosition());
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
        draggedBody = nullptr;
    }
}

void Renderer::handleMouseMoved(const sf::Vector2i&) {
    // The drag spring reads the mouse position every physics step.
}

void Renderer::addCircle(const Vector2& position) {
    Material material = {1.0f, 0.2f};
    world->addBody(std::make_shared<RigidBody>(Circle(0.5f), material, position));
}

void Renderer::addBox(const Vector2& position) {
    Material material = {1.0f, 0.2f};
    world->addBody(std::make_shared<RigidBody>(Polygon::MakeBox(1.0f, 1.0f), material, position));
}

void Renderer::addCapsule(const Vector2& position) {
    Material material = {1.0f, 0.2f};
    world->addBody(std::make_shared<RigidBody>(Capsule(0.6f, 0.25f), material, position));
}

} // namespace Visualization
} // namespace PhysicsEngine

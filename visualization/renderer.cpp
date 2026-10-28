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
#include <string>
#include <vector>

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

Renderer::~Renderer() {
    if (world && sceneRuntime.onUnload) sceneRuntime.onUnload(*world);
}

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
    if (sceneRuntime.onUnload) sceneRuntime.onUnload(*world);
    sceneRuntime = SceneRuntime{};
    draggedBody = nullptr;
    dragJoint.reset(); // clearBodies below removes the joint and its ground body
    dragGround.reset();
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
// Scene number for the number row or keypad: 1-9, and 0 selects scene 10.
// Returns -1 for any other key.
int sceneNumber(sf::Keyboard::Key key) {
    const int code = static_cast<int>(key);
    const int row = code - static_cast<int>(sf::Keyboard::Key::Num0);
    const int pad = code - static_cast<int>(sf::Keyboard::Key::Numpad0);
    if (row >= 0 && row <= 9) return row == 0 ? 10 : row;
    if (pad >= 0 && pad <= 9) return pad == 0 ? 10 : pad;
    return -1;
}

// Tab label key for scene `index`: 1-9, then 0.
std::string sceneKeyLabel(std::size_t index) {
    return index == 9 ? "0" : std::to_string(index + 1);
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
            } else if (keyPressed->code == sf::Keyboard::Key::Tab
                       || (keyPressed->code == sf::Keyboard::Key::D && !sceneDrives())) {
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
            } else if (keyPressed->code == sf::Keyboard::Key::Left && (keyPressed->shift || !sceneDrives())) {
                cameraCenter.x -= 1.0f / zoom;
            } else if (keyPressed->code == sf::Keyboard::Key::Right && (keyPressed->shift || !sceneDrives())) {
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
    if (sceneRuntime.drive) sceneRuntime.drive(currentThrottle());
    if (dragJoint) dragJoint->setTarget(screenToWorld(sf::Mouse::getPosition(window)));
    const auto start = std::chrono::steady_clock::now();
    world->step(fixedTimeStep);
    const float elapsed =
        std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - start).count();
    stepMilliseconds = stepMilliseconds == 0.0f
        ? elapsed : stepMilliseconds + 0.1f * (elapsed - stepMilliseconds);
    sceneTime += fixedTimeStep;
}

// Throttle in [-1, 1] for the scene's vehicle: Left/A is -1, Right/D is +1.
float Renderer::currentThrottle() const {
    if (hasForcedThrottle) return forcedThrottle;
    if (!window.hasFocus()) return 0.0f;
    float throttle = 0.0f;
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Right) || sf::Keyboard::isKeyPressed(sf::Keyboard::Key::D))
        throttle += 1.0f;
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Left) || sf::Keyboard::isKeyPressed(sf::Keyboard::Key::A))
        throttle -= 1.0f;
    return throttle;
}

// Pull the grab point toward the mouse with a MouseJoint so the dragged body
// still collides, swings about the grab point and keeps its velocity when
// released, instead of being teleported through its neighbours. The joint is
// a 5 Hz spring with the force capped at 1000 times the body mass. The engine
// solves the spring implicitly, which adds numerical damping at 60 Hz, so a
// requested ratio of 0.9 settles like the old hand-written critically damped
// spring (a grabbed box 5 m away is within 2 cm of the old path at 0.2 s).
void Renderer::beginDrag(const RigidBodyPtr& body, Vector2 worldPoint) {
    constexpr float frequency = 5.0f;
    constexpr float dampingRatio = 0.9f;
    dragGround = std::make_shared<RigidBody>(Circle(0.05f), Material{1.0f, 0.1f, 0.5f, 0.4f}, Vector2(0.0f, 0.0f), true);
    dragGround->SetCollisionMaskBits(0u);
    world->addBody(dragGround);
    const Vector2 localAnchor = Matrix2x2::rotation(-body->GetOrientation()) * (worldPoint - body->GetPosition());
    dragJoint = std::make_shared<MouseJoint>(dragGround, body, localAnchor, worldPoint,
                                             frequency, dampingRatio, 1000.0f * body->GetMass());
    world->addJoint(dragJoint);
    draggedBody = body.get();
}

void Renderer::endDrag() {
    if (dragJoint) world->removeJoint(dragJoint);
    if (dragGround) world->removeBody(dragGround);
    dragJoint.reset();
    dragGround.reset();
    draggedBody = nullptr;
}

void Renderer::render() {
    window.clear(sf::Color(30, 30, 40));

    // Sensors first, so zones sit behind the bodies that overlap them.
    for (const auto& body : world->getBodies()) {
        if (body->IsSensor()) renderBody(body.get());
    }
    for (const auto& body : world->getBodies()) {
        if (!body->IsSensor()) renderBody(body.get());
    }
    renderJoints();
    renderHud();

    window.display();
}

// A zig-zag line between two screen points, drawn for spring-like joints.
void Renderer::drawZigzag(sf::Vector2f from, sf::Vector2f to, sf::Color color) {
    constexpr int teeth = 8;
    constexpr float amplitude = 5.0f;
    const sf::Vector2f delta = to - from;
    const float length = std::sqrt(delta.x * delta.x + delta.y * delta.y);
    if (length < 2.0f * amplitude) {
        const sf::Vertex line[] = {{from, color}, {to, color}};
        window.draw(line, 2, sf::PrimitiveType::Lines);
        return;
    }
    const sf::Vector2f along = delta / length;
    const sf::Vector2f side(-along.y, along.x);
    std::vector<sf::Vertex> points;
    points.push_back({from, color});
    for (int i = 0; i < 2 * teeth; ++i) {
        const float t = (static_cast<float>(i) + 0.5f) / (2.0f * teeth);
        const float offset = i % 2 == 0 ? amplitude : -amplitude;
        points.push_back({from + delta * t + side * offset, color});
    }
    points.push_back({to, color});
    window.draw(points.data(), points.size(), sf::PrimitiveType::LineStrip);
}

void Renderer::renderJoints() {
    const sf::Color yellow(255, 225, 110);
    const sf::Color pink(240, 130, 220);
    const sf::Color cyan(90, 215, 235);
    const sf::Color green(130, 230, 140);
    const sf::Color tan(215, 175, 120);
    const sf::Color background(30, 30, 40);
    for (const auto& joint : world->getJoints()) {
        const IJoint* raw = joint.get();

        // The drag joint is drawn as a line from the grab point to the cursor.
        if (const auto* mouse = dynamic_cast<const MouseJoint*>(raw)) {
            const sf::Vector2f from = worldToScreen(mouse->getAnchorB());
            const sf::Vector2f to = worldToScreen(mouse->getTarget());
            const sf::Vertex line[] = {{from, yellow}, {to, yellow}};
            window.draw(line, 2, sf::PrimitiveType::Lines);
            continue;
        }

        const sf::Vector2f a = worldToScreen(raw->getAnchorA());
        const sf::Vector2f b = worldToScreen(raw->getAnchorB());
        sf::Color color = yellow;
        std::size_t sides = 4; // square marker unless a branch below changes it

        if (dynamic_cast<const RevoluteJoint*>(raw)) {
            sides = 16;
        } else if (dynamic_cast<const WeldJoint*>(raw)) {
            color = pink;
            sides = 6;
        } else if (const auto* wheel = dynamic_cast<const WheelJoint*>(raw)) {
            // The suspension stroke along the axis, with the spring at its rest point.
            color = cyan;
            sides = 16;
            const Vector2 axis = wheel->getAxis();
            const Vector2 base = raw->getAnchorA();
            const float lower = wheel->areLimitsEnabled() ? wheel->getLowerLimit() : -0.3f;
            const float upper = wheel->areLimitsEnabled() ? wheel->getUpperLimit() : 0.3f;
            const sf::Vector2f low = worldToScreen(base + axis * lower);
            const sf::Vector2f high = worldToScreen(base + axis * upper);
            sf::RectangleShape rail(sf::Vector2f(std::hypot(high.x - low.x, high.y - low.y), 5.0f));
            rail.setOrigin(sf::Vector2f(0.0f, 2.5f));
            rail.setPosition(low);
            rail.setRotation(sf::radians(std::atan2(high.y - low.y, high.x - low.x)));
            rail.setFillColor(sf::Color(90, 215, 235, 70));
            window.draw(rail);
        } else if (const auto* distance = dynamic_cast<const DistanceJoint*>(raw)) {
            if (distance->getSpringFrequency() > 0.0f) color = green;
            else if (distance->getMinLength() < distance->getMaxLength()) color = tan;
        }

        if (dynamic_cast<const DistanceJoint*>(raw) && color == green) {
            drawZigzag(a, b, color);
        } else {
            const sf::Vertex line[] = {{a, color}, {b, color}};
            window.draw(line, 2, sf::PrimitiveType::Lines);
        }

        // Anchor markers: round for revolute and wheel joints, hexagons for
        // welds, squares for the others.
        for (const sf::Vector2f& point : {a, b}) {
            sf::CircleShape marker(3.5f, sides);
            marker.setOrigin(sf::Vector2f(3.5f, 3.5f));
            marker.setPosition(point);
            marker.setFillColor(background);
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
    // The drag joint and its ground body exist only while dragging; leave them out.
    const std::size_t dragBodies = dragGround ? 1 : 0;
    const std::size_t dragJoints = dragJoint ? 1 : 0;
    info += "\nBodies: " + std::to_string(world->getBodies().size() - dragBodies);
    info += "   Joints: " + std::to_string(world->getJoints().size() - dragJoints);
    if (sceneRuntime.status) info += "\n" + sceneRuntime.status();
    info += "\n" + std::string(timing);
    info += "   FPS: " + std::to_string(static_cast<int>(1.0f / std::max(lastFrameTime, 1e-6f)));
    info += isPaused ? "\n[PAUSED]" : "\n[RUNNING]";
    if (showHelp) {
        info += "\n\n1-9, 0: Select scene   R: Reset scene\n";
        info += "Left drag: Grab body   Right click: Box\n";
        info += "Middle click / C: Circle   B: Box   K: Capsule\n";
        info += "Space: Pause   Arrows: Camera   Wheel: Zoom\n";
        if (scene.controls) info += std::string(scene.controls) + "\n";
        info += "H: Hide help   Tab: Hide HUD   Esc: Quit";
    } else {
        info += "\nH: Show help";
    }
    drawLabel(info, sf::Vector2f(10.0f, 8.0f), 14);

    // Two rows of scene tabs, so ten names fit in the window width.
    const std::size_t perRow = (sceneList().size() + 1) / 2;
    std::string tabs;
    for (std::size_t i = 0; i < sceneList().size(); ++i) {
        const std::string label = sceneKeyLabel(i) + " " + sceneList()[i].name;
        tabs += i == sceneIndex ? "[" + label + "]  " : label + "  ";
        if (i + 1 == perRow) tabs += "\n";
    }
    drawLabel(tabs, sf::Vector2f(10.0f, static_cast<float>(window.getSize().y) - 6.0f), 13, true);
}

// Fill and outline by role: dragged (yellow), sensor (translucent zone, brighter
// while the scene reports it active), static or dynamic.
void Renderer::styleShape(sf::Shape& shape, const RigidBody* body, sf::Color dynamicFill, sf::Color staticFill) const {
    if (body->IsSensor()) {
        const bool active = sceneRuntime.highlighted && sceneRuntime.highlighted(*body);
        shape.setFillColor(active ? sf::Color(255, 210, 70, 120) : sf::Color(70, 220, 140, 50));
        shape.setOutlineThickness(2);
        shape.setOutlineColor(active ? sf::Color(255, 225, 110) : sf::Color(70, 220, 140, 170));
    } else if (body == draggedBody) {
        shape.setFillColor(sf::Color(255, 255, 100));  // Yellow when dragged
        shape.setOutlineThickness(3);
        shape.setOutlineColor(sf::Color(255, 255, 0));
    } else {
        shape.setFillColor(body->IsStatic() ? staticFill : dynamicFill);
        shape.setOutlineThickness(2);
        shape.setOutlineColor(sf::Color(200, 200, 200));
    }
}

void Renderer::renderBody(const RigidBody* body) {
    if (!body || body == dragGround.get()) return;

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

    styleShape(shape, body, sf::Color(100, 150, 255), sf::Color(100, 100, 100));

    window.draw(shape);
    if (body->IsSensor()) return;

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

    styleShape(shape, body, sf::Color(255, 150, 100), sf::Color(80, 80, 80));

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

    styleShape(shape, body, sf::Color(120, 210, 140), sf::Color(80, 80, 80));

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
        // Grab after the loop: beginDrag adds a body to the list being walked.
        RigidBodyPtr grabbed;
        auto& bodies = world->getBodies();
        for (auto it = bodies.rbegin(); it != bodies.rend(); ++it) {
            const auto& body = *it;
            if (body->IsStatic() || body->IsSensor()) continue;

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
                grabbed = body;
                break;
            }
        }
        if (grabbed && !dragJoint) beginDrag(grabbed, worldPos);
    } else if (button == sf::Mouse::Button::Right) {
        addBox(worldPos);
    } else if (button == sf::Mouse::Button::Middle) {
        addCircle(worldPos);
    }
}

void Renderer::handleMouseReleased(sf::Mouse::Button button) {
    if (button == sf::Mouse::Button::Left && dragJoint) endDrag();
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

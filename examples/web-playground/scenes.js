// Scene builders and the stepping logic for the web playground.
//
// This file has no DOM dependency: the page loads it with a <script> tag
// (global `PlaygroundScenes`) and scenes-test.cjs loads it with require().
// The ten scenes mirror visualization/scenes.cpp. Every Embind handle a
// scene creates is kept in the Scene and released by Scene.dispose().
(function (root, factory) {
    if (typeof module === 'object' && module.exports) module.exports = factory();
    else root.PlaygroundScenes = factory();
})(typeof self !== 'undefined' ? self : this, function () {
    'use strict';

    const FLOOR_TOP = -5;
    const GRAVITY = -9.81;
    // Chain parts live in category 2 and ignore it, so neighbours linked by a
    // joint do not fight their own contact but still collide with everything else.
    const CHAIN_CATEGORY = 2;
    const NO_CHAIN_MASK = (~CHAIN_CATEGORY) >>> 0;
    const SLIDER_TRAVEL = 6;
    const DRAG_FREQUENCY = 5; // Hz
    // The engine solves the mouse spring implicitly, which adds numerical
    // damping at 60 Hz; 0.9 settles like the old critically damped spring.
    const DRAG_DAMPING = 0.9;
    const DRAG_FORCE_PER_MASS = 1000;
    const CAR_TOP_SPEED = 16;    // rear wheel, rad/s
    const CAR_DRIVE_TORQUE = 16; // N m
    const CAR_IDLE_TORQUE = 1.5; // light brake so the car stays put
    const GOAL_TAG = 7;
    const MAX_FRAME_SECONDS = 0.1; // Same spiral-of-death cap as the native visualizer.

    const now = typeof performance !== 'undefined' && performance.now
        ? () => performance.now() : () => Date.now();

    const material = (density, restitution, staticFriction, dynamicFriction) =>
        ({density, restitution, staticFriction, dynamicFriction});
    const STONE = material(1, 0.1, 0.6, 0.4);
    const GROUND = material(1, 0.1, 0.7, 0.5);
    const ZERO = Object.freeze({x: 0, y: 0});

    class Scene {
        constructor(physics, definition) {
            this.physics = physics;
            this.definition = definition;
            this.engine = new physics.Engine();
            this.bodies = [];            // body records, in creation order
            this.joints = [];            // joint records
            this.byId = new Map();       // BigInt body id -> record
            this.controllers = [];       // (scene, sceneTime) called before every step
            this.input = {throttle: 0};  // vehicle throttle in [-1, 1], set by the page
            this.status = null;          // optional () => string with extra HUD lines
            this.goal = null;            // car scene: {record, inside, entries, exits}
            this.eventHandlers = [];     // called with each collision event after a step
            this.sceneTime = 0;
            this.stepMilliseconds = 0;   // smoothed wall-clock cost of one fixed step
            this.drag = null;
            this.disposed = false;

            const config = this.engine.getSimulationConfig();
            config.solverIterations = definition.solverIterations;
            this.engine.setSimulationConfig(config);
            this.fixedTimeStep = config.fixedTimeStep;
            this.maxSubsteps = config.maxSubstepsPerAdvance;
            physics.addGravity(this.engine, {x: 0, y: GRAVITY});
            definition.build(this);
            this.capturePrevious();
            this.readPoses();
        }

        // ---- building -------------------------------------------------------

        addShapeBody(shapeHandle, descriptor, mat, position, options) {
            const {isStatic = false, angle = 0, category = null, mask = null, spawned = false,
                sensor = false} = options;
            let body;
            try {
                body = this.physics.createRigidBody(shapeHandle, mat, position, isStatic);
            } finally {
                shapeHandle.delete(); // Bodies own a clone of their shape.
            }
            try {
                if (angle !== 0) body.setOrientation(angle);
                if (category !== null) body.setCollisionCategoryBits(category);
                if (mask !== null) body.setCollisionMaskBits(mask);
                if (sensor) { body.setSensor(true); body.setUserData(GOAL_TAG); }
                this.engine.addBody(body);
            } catch (error) {
                body.delete();
                throw error;
            }
            const record = {
                body, id: body.getId(), isStatic, spawned,
                ghost: mask === 0, // collides with nothing: a pivot or rail anchor
                sensor,            // overlap zone: drawn translucent, never solid
                x: position.x, y: position.y, a: angle, px: position.x, py: position.y, pa: angle,
                ...descriptor,
            };
            this.bodies.push(record);
            this.byId.set(record.id, record);
            return record;
        }

        box(width, height, position, options = {}) {
            return this.addShapeBody(this.physics.Polygon.makeBox(width, height),
                {kind: 'box', width, height}, options.material || STONE, position, options);
        }

        ball(radius, position, options = {}) {
            return this.addShapeBody(new this.physics.Circle(radius),
                {kind: 'circle', radius}, options.material || STONE, position, options);
        }

        capsule(halfLength, radius, position, options = {}) {
            return this.addShapeBody(new this.physics.Capsule(halfLength, radius),
                {kind: 'capsule', halfLength, radius}, options.material || STONE, position, options);
        }

        anchor(position, size = 0.3) {
            return this.box(size, size, position, {isStatic: true, mask: 0});
        }

        addJointRecord(kind, joint, a, b, localA, localB) {
            try {
                this.engine.addJoint(joint);
            } catch (error) {
                joint.delete();
                throw error;
            }
            const record = {kind, joint, a, b, localA, localB};
            this.joints.push(record);
            return record;
        }

        revolute(a, b, localAnchorA = ZERO, localAnchorB = ZERO) {
            const joint = this.physics.createRevoluteJoint(a.body, b.body, localAnchorA, localAnchorB);
            this.addJointRecord('revolute', joint, a, b, localAnchorA, localAnchorB);
            return joint;
        }

        // Optional {spring: [frequency, damping], range: [min, max]}. The record
        // kind tells the renderer how to draw it: a plain distance joint, a
        // spring (zig-zag) or a rope (free between the stops).
        distance(a, b, length, options = {}) {
            const joint = this.physics.createDistanceJoint(a.body, b.body, length, ZERO, ZERO);
            let kind = 'distance';
            try {
                if (options.spring) { joint.setSpring(...options.spring); kind = 'spring'; }
                if (options.range) { joint.setLengthRange(...options.range); if (!options.spring) kind = 'rope'; }
            } catch (error) {
                joint.delete();
                throw error;
            }
            this.addJointRecord(kind, joint, a, b, ZERO, ZERO);
            return joint;
        }

        weld(a, b, localAnchorA = ZERO, localAnchorB = ZERO) {
            const joint = this.physics.createWeldJoint(a.body, b.body, localAnchorA, localAnchorB);
            this.addJointRecord('weld', joint, a, b, localAnchorA, localAnchorB);
            return joint;
        }

        // A is the chassis (or frame), B the wheel; the axis is in A's frame.
        wheel(a, b, localAxisA, localAnchorA = ZERO, localAnchorB = ZERO) {
            const joint = this.physics.createWheelJoint(a.body, b.body, localAxisA, localAnchorA, localAnchorB);
            this.addJointRecord('wheel', joint, a, b, localAnchorA, localAnchorB);
            return joint;
        }

        prismatic(a, b, localAxisA) {
            const joint = this.physics.createPrismaticJoint(a.body, b.body, localAxisA, ZERO, ZERO);
            this.addJointRecord('prismatic', joint, a, b, ZERO, ZERO);
            return joint;
        }

        arena() {
            this.box(20, 2, {x: 0, y: -6}, {material: GROUND, isStatic: true});
            this.box(2, 14, {x: -11, y: 0}, {material: GROUND, isStatic: true});
            this.box(2, 14, {x: 11, y: 0}, {material: GROUND, isStatic: true});
        }

        // ---- stepping -------------------------------------------------------

        // Work done before every fixed step, however the step is triggered.
        beforeStep() {
            for (const controller of this.controllers) controller(this, this.sceneTime);
            this.sceneTime += this.fixedTimeStep;
        }

        timed(run) {
            const start = now();
            const result = run();
            const elapsed = now() - start;
            this.stepMilliseconds = this.stepMilliseconds === 0
                ? elapsed : this.stepMilliseconds + 0.1 * (elapsed - this.stepMilliseconds);
            return result;
        }

        // Exactly one fixed step, with its pre-step work and pose capture.
        stepOnce() {
            this.beforeStep();
            this.capturePrevious();
            this.timed(() => this.engine.stepFixed());
            this.pumpEvents();
            this.readPoses();
        }

        // Advances the simulation by `seconds` of wall time through
        // engine.advance(), one fixed step per call so that the controller and
        // drag spring run before every step. The engine keeps the sub-step
        // remainder, so interpolationAlpha stays the engine's own value.
        // Returns {steps, alpha}.
        advanceFrame(seconds) {
            const dt = this.fixedTimeStep;
            const tolerance = dt * 1e-6;
            let remaining = Math.min(Math.max(seconds, 0), MAX_FRAME_SECONDS);
            let steps = 0;
            let alpha = 0;
            let ranFinal = false;
            while (steps < this.maxSubsteps) {
                const accumulated = this.engine.getAccumulatedTime();
                if (accumulated + remaining + tolerance < dt) {
                    // No further step fits: bank the remainder in the engine.
                    alpha = this.engine.advance(Math.max(remaining, 0)).interpolationAlpha;
                    ranFinal = true;
                    break;
                }
                const need = Math.max(dt - accumulated, 0);
                this.beforeStep();
                this.capturePrevious();
                const result = this.timed(() => this.engine.advance(need));
                remaining -= need;
                steps += result.stepsPerformed;
                this.pumpEvents();
            }
            if (!ranFinal) alpha = Math.min(this.engine.getAccumulatedTime() / dt, 1);
            if (steps > 0) this.readPoses();
            return {steps, alpha};
        }

        // ---- poses ----------------------------------------------------------

        // The JS API has no previous-pose getter, so the pose before each step
        // is remembered here; render poses blend it with the current pose.
        capturePrevious() {
            for (const r of this.bodies) {
                if (r.isStatic) continue;
                r.px = r.x; r.py = r.y; r.pa = r.a;
            }
        }

        readPoses() {
            for (const r of this.bodies) {
                if (r.isStatic) continue;
                const p = r.body.getPosition();
                r.x = p.x; r.y = p.y; r.a = r.body.getOrientation();
            }
        }

        // ---- interaction ----------------------------------------------------

        spawn(kind, x, y) {
            const options = {material: material(1, 0.2, 0.6, 0.4), spawned: true};
            const position = {x, y};
            if (kind === 'circle') return this.ball(0.5, position, options);
            if (kind === 'box') return this.box(1, 1, position, options);
            if (kind === 'capsule') return this.capsule(0.6, 0.25, position, options);
            throw new Error('Unknown shape kind: ' + kind);
        }

        // Registers a handler for the engine's collision events and turns
        // recording on. The handler gets plain values (phase, ids, sensor flag);
        // events are pulled once per step, so nothing calls back into JS while
        // the engine is stepping.
        onCollisionEvent(handler) {
            this.engine.enableCollisionEvents(true);
            this.eventHandlers.push(handler);
        }

        // Drains the engine's event buffer into the handlers. The list is an
        // owned collection and is deleted here, even if a handler throws.
        pumpEvents() {
            if (this.eventHandlers.length === 0) return;
            const list = this.engine.takeCollisionEvents();
            try {
                for (let i = 0; i < list.size(); ++i) {
                    const event = list.getEvent(i);
                    for (const handler of this.eventHandlers) handler(event);
                }
                if (list.overflowed()) throw new Error('Collision event buffer overflowed');
            } finally {
                list.delete();
            }
        }

        // Dynamic body under a world point, or null. The query result is an
        // owned collection and is deleted here; only the BigInt ids are used.
        pick(x, y) {
            const results = this.physics.queryPoint(this.engine, {x, y});
            try {
                for (let i = 0; i < results.size(); ++i) {
                    const record = this.byId.get(results.getBodyId(i));
                    if (record && !record.isStatic) return record;
                }
                return null;
            } finally {
                results.delete();
            }
        }

        // Grabs `record` at a world point with a MouseJoint, which pulls the
        // grab point toward the target with a 5 Hz damped spring and a force cap
        // of 1000 * mass (the same feel as the native visualizer). The joint
        // needs a static body that is already in the engine, so a tiny
        // collision-free ground body is created for the drag and removed with
        // it. Neither is listed in `bodies` or `joints`.
        beginDrag(record, x, y) {
            this.endDrag();
            const p = record.body.getPosition();
            const angle = record.body.getOrientation();
            const dx = x - p.x, dy = y - p.y;
            const c = Math.cos(angle), s = Math.sin(angle);
            const localAnchor = {x: c * dx + s * dy, y: -s * dx + c * dy};
            const circle = new this.physics.Circle(0.05);
            let ground;
            try {
                ground = this.physics.createRigidBody(circle, material(1, 0.1, 0.5, 0.4), ZERO, true);
            } finally {
                circle.delete();
            }
            let joint = null;
            let grounded = false;
            try {
                ground.setCollisionMaskBits(0);
                this.engine.addBody(ground);
                grounded = true;
                joint = this.physics.createMouseJoint(ground, record.body, localAnchor, {x, y},
                    DRAG_FREQUENCY, DRAG_DAMPING, DRAG_FORCE_PER_MASS * record.body.getMass());
                this.engine.addJoint(joint);
            } catch (error) {
                if (joint) joint.delete();
                if (grounded) this.engine.removeBody(ground);
                ground.delete();
                throw error;
            }
            record.body.wake();
            this.drag = {record, target: {x, y}, localAnchor, joint, ground};
        }

        setDragTarget(x, y) {
            if (!this.drag) return;
            this.drag.target = {x, y};
            this.drag.joint.setTarget({x, y});
        }

        // Removes the joint and the ground body, then deletes both handles.
        endDrag() {
            const drag = this.drag;
            if (!drag) return;
            this.drag = null;
            this.engine.removeJoint(drag.joint);
            this.engine.removeBody(drag.ground);
            drag.joint.delete();
            drag.ground.delete();
        }

        // Drops spawned bodies that fell or flew far outside the arena.
        cull(limit = 60) {
            for (let i = this.bodies.length - 1; i >= 0; --i) {
                const r = this.bodies[i];
                if (!r.spawned || (Math.abs(r.x) < limit && Math.abs(r.y) < limit)) continue;
                if (this.drag && this.drag.record === r) this.endDrag();
                this.removeBodyAt(i);
            }
        }

        removeBodyAt(index) {
            const [record] = this.bodies.splice(index, 1);
            this.byId.delete(record.id);
            this.engine.removeBody(record.body);
            record.body.delete();
        }

        // ---- counts and teardown --------------------------------------------

        get bodyCount() { return this.bodies.length; }
        get jointCount() { return this.joints.length; }

        // Releases every handle this scene created. Safe to call twice.
        dispose() {
            if (this.disposed) return;
            this.endDrag();
            this.disposed = true;
            for (const j of this.joints) j.joint.delete();
            for (const b of this.bodies) b.body.delete();
            this.engine.delete();
            this.joints.length = 0;
            this.bodies.length = 0;
            this.byId.clear();
        }
    }

    // Interpolated pose between the previous and current step.
    function renderPose(record, alpha) {
        if (record.isStatic) return {x: record.x, y: record.y, a: record.a};
        let da = record.a - record.pa;
        if (da > Math.PI) da -= 2 * Math.PI; else if (da < -Math.PI) da += 2 * Math.PI;
        return {
            x: record.px + (record.x - record.px) * alpha,
            y: record.py + (record.y - record.py) * alpha,
            a: record.pa + da * alpha,
        };
    }

    // World position of a body-local point at the interpolated pose.
    function anchorPoint(record, local, alpha) {
        const pose = renderPose(record, alpha);
        const c = Math.cos(pose.a), s = Math.sin(pose.a);
        return {x: pose.x + c * local.x - s * local.y, y: pose.y + s * local.x + c * local.y};
    }

    // ---- scenes (mirroring visualization/scenes.cpp) ------------------------

    function buildPlayground(s) {
        s.arena();
        s.box(1, 1, {x: -3, y: -3});
        s.box(1.5, 1, {x: 2, y: -3});
        s.ball(0.5, {x: 0, y: 0});
    }

    function buildTower(s) {
        s.arena();
        const height = 0.8;
        for (let i = 0; i < 12; ++i) {
            const jitter = 0.03 * Math.sin(1.7 * i);
            s.box(1.2, height, {x: jitter, y: FLOOR_TOP + height * (i + 0.5) + 0.002 * i});
        }
    }

    function buildPyramid(s) {
        s.arena();
        const base = 8, width = 1.1, height = 0.7;
        for (let row = 0; row < base; ++row) {
            const columns = base - row;
            for (let column = 0; column < columns; ++column) {
                const x = (column - 0.5 * (columns - 1)) * (width + 0.01);
                s.box(width, height, {x, y: FLOOR_TOP + height * (row + 0.5) + 0.002 * row});
            }
        }
    }

    function buildDominoes(s) {
        s.arena();
        const count = 20, width = 0.22, height = 1.6, spacing = 0.82;
        for (let i = 0; i < count; ++i) {
            const x = (i - 0.5 * (count - 1)) * spacing;
            s.box(width, height, {x, y: FLOOR_TOP + 0.5 * height + 0.001});
        }
        const ball = s.ball(0.5, {x: -9.4, y: FLOOR_TOP + 0.51}, {material: material(4, 0.1, 0.6, 0.4)});
        ball.body.setVelocity({x: 6, y: 0});
    }

    // A hanging revolute chain beside a distance-joint pendulum that swings into
    // a stack of boxes.
    function buildChainAndPendulum(s) {
        s.arena();
        const halfLength = 0.4, radius = 0.2, tip = halfLength + radius, links = 5;
        const pivot = {x: -8.5, y: 3};
        let previous = s.anchor(pivot);
        let previousTip = {x: 0, y: 0};
        const chainOptions = {category: CHAIN_CATEGORY, mask: NO_CHAIN_MASK};
        for (let i = 0; i < links; ++i) {
            const link = s.capsule(halfLength, radius,
                {x: pivot.x + tip + 2 * tip * i, y: pivot.y},
                {...chainOptions, material: material(1, 0.1, 0.5, 0.4)});
            s.revolute(previous, link, previousTip, {x: -tip, y: 0});
            previous = link;
            previousTip = {x: tip, y: 0};
        }
        const weight = s.ball(0.45, {x: pivot.x + 2 * tip * links + 0.45, y: pivot.y},
            {...chainOptions, material: material(3, 0.1, 0.5, 0.4)});
        s.revolute(previous, weight, previousTip, {x: -0.45, y: 0});

        for (let row = 0; row < 5; ++row) {
            for (let column = 0; column < 2; ++column) {
                s.box(0.98, 0.98, {x: 0.5 + column, y: FLOOR_TOP + 0.5 + 0.99 * row + 0.002});
            }
        }

        const hinge = {x: 3, y: 3}, length = 6, angle = 70 * Math.PI / 180;
        const support = s.anchor(hinge);
        const bob = s.ball(0.6,
            {x: hinge.x + length * Math.sin(angle), y: hinge.y - length * Math.cos(angle)},
            {material: material(6, 0.1, 0.5, 0.4)});
        s.distance(support, bob, length);
    }

    function buildRamps(s) {
        s.arena();
        const slick = material(1, 0.15, 0.3, 0.2);
        const steep = 0.4, shallow = 0.35;
        const firstRamp = {x: -4.5, y: 2}, capsuleRamp = {x: 6.4, y: 3.6};

        s.box(8, 0.4, firstRamp, {material: slick, isStatic: true, angle: -steep});
        s.box(8, 0.4, {x: 3, y: -1.5}, {material: slick, isStatic: true, angle: steep});
        s.capsule(2.5, 0.3, capsuleRamp, {material: slick, isStatic: true, angle: shallow});
        s.box(0.3, 1.2, {x: -8.4, y: 3.5}, {material: slick, isStatic: true});

        // A point `along` a ramp's axis and `lift` above its centre line.
        const onRamp = (center, angle, along, lift) => ({
            x: center.x + Math.cos(angle) * along - Math.sin(angle) * lift,
            y: center.y + Math.sin(angle) * along + Math.cos(angle) * lift,
        });
        s.capsule(0.5, 0.2, onRamp(firstRamp, -steep, -3.2, 0.42), {material: slick, angle: -steep});
        s.ball(0.4, onRamp(firstRamp, -steep, -1.9, 0.62));
        s.capsule(0.5, 0.2, onRamp(firstRamp, -steep, -0.6, 0.42), {material: slick, angle: -steep});
        s.ball(0.3, onRamp(firstRamp, -steep, 0.8, 0.52));
        s.capsule(0.45, 0.2, onRamp(capsuleRamp, shallow, 1.4, 0.52), {material: slick, angle: shallow});
        s.ball(0.35, onRamp(capsuleRamp, shallow, 0, 0.67));
        s.capsule(0.45, 0.2, onRamp(capsuleRamp, shallow, -1.3, 0.52), {material: slick, angle: shallow});
    }

    // A heavy ball drops on one end of a plank on a hinge and throws a light box
    // from the other end (a mass ratio of roughly 25 to 1).
    function buildSeesaw(s) {
        s.arena();
        const halfPlank = 6, tilt = -0.3;
        const hinge = {x: 0, y: -3};
        const support = s.anchor(hinge);
        s.box(0.3, 2, {x: hinge.x, y: hinge.y - 1}, {isStatic: true, mask: 0}); // decorative post

        const plank = s.box(2 * halfPlank, 0.3, hinge, {material: GROUND, angle: tilt});
        const joint = s.revolute(support, plank);
        joint.setLimits(true, 0, 0.6);

        const along = {x: Math.cos(tilt), y: Math.sin(tilt)};
        const up = {x: -Math.sin(tilt), y: Math.cos(tilt)};
        const at = (a, u) => ({x: hinge.x + along.x * a + up.x * u, y: hinge.y + along.y * a + up.y * u});
        const light = material(0.8, 0.1, 0.7, 0.5);
        s.box(0.7, 0.7, at(halfPlank - 0.7, 0.51), {material: light, angle: tilt});
        s.box(0.5, 0.5, at(halfPlank - 1.6, 0.41), {material: light, angle: tilt});
        s.ball(0.6, {x: -5, y: 3.5}, {material: material(9, 0.1, 0.7, 0.5)});
    }

    // A motorized paddle wheel, a driven roller and a prismatic slider that
    // shuttles back and forth pushing a stack.
    function buildMachines(s) {
        s.arena();

        const hub = {x: -6.5, y: -1.5};
        const hubBody = s.anchor(hub, 0.4);
        const beam = s.box(6, 0.3, hub, {material: material(2, 0.1, 0.5, 0.4)});
        s.revolute(hubBody, beam).setMotor(true, 1.6, 5000);
        for (let i = 0; i < 3; ++i) s.box(0.9, 0.9, {x: -8.5 + 1.1 * i, y: FLOOR_TOP + 0.46});
        s.box(0.9, 0.9, {x: -7.9, y: FLOOR_TOP + 1.4});

        const axle = {x: -1, y: -3.4};
        const axleBody = s.anchor(axle, 0.3);
        const wheel = s.ball(1.55, axle, {material: material(1, 0.1, 1.2, 1)});
        s.revolute(axleBody, wheel).setMotor(true, -4, 8000);
        for (let i = 0; i < 4; ++i) {
            s.ball(0.35, {x: -1.6 + 0.55 * i, y: 1 + 1 * i}, {material: material(1, 0.2, 0.6, 0.4)});
        }

        const railY = FLOOR_TOP + 0.95;
        const start = {x: 9, y: railY};
        s.box(SLIDER_TRAVEL + 1, 0.1, {x: start.x - 0.5 * SLIDER_TRAVEL, y: railY},
            {isStatic: true, mask: 0});
        const railAnchor = s.anchor(start, 0.1);
        const carriage = s.box(1, 1.8, start, {material: material(5, 0.1, 0.4, 0.3)});
        const slider = s.prismatic(railAnchor, carriage, {x: -1, y: 0});
        slider.setLimits(true, 0, SLIDER_TRAVEL);
        slider.setMotor(true, 1.5, 3000);
        for (let row = 0; row < 3; ++row) {
            s.box(0.9, 0.9, {x: 5, y: FLOOR_TOP + 0.46 + 0.92 * row});
        }
        s.box(0.9, 0.9, {x: 4, y: FLOOR_TOP + 0.46});

        s.controllers.push(() => {
            const position = slider.getTranslation();
            const speed = slider.getMotorSpeed();
            if (position >= SLIDER_TRAVEL - 0.05 && speed > 0) slider.setMotor(true, -1.5, 3000);
            else if (position <= 0.05 && speed < 0) slider.setMotor(true, 1.5, 3000);
        });
    }

    // A car on two wheel-joint suspensions with a rear-wheel motor, bumpy
    // terrain and a static sensor goal. The goal counts entries from the
    // engine's own sensor Begin/End events for the chassis and both wheels.
    // The page sets scene.input.throttle; the controller below applies it.
    function buildCar(s) {
        s.arena();
        const dirt = material(1, 0.1, 0.8, 0.7);
        const bumps = [[-5.6, 0.9, 0.30], [-3.4, 0.7, 0.35], [-1.4, 1.3, 0.55], [0.6, 0.6, 0.30], [1.7, 0.6, 0.35]];
        for (const [x, radius, height] of bumps) {
            s.ball(radius, {x, y: FLOOR_TOP + height - radius}, {material: dirt, isStatic: true});
        }
        // A gentle hill of two tilted slabs, 0.35 m high at the peak.
        s.box(2, 0.3, {x: 3.24, y: -4.967}, {material: dirt, isStatic: true, angle: 0.17});
        s.box(2, 0.3, {x: 5.16, y: -4.967}, {material: dirt, isStatic: true, angle: -0.17});

        const goal = s.box(3, 2.6, {x: 7.7, y: -3.7}, {isStatic: true, sensor: true});

        const start = {x: -8, y: -4.1};
        const chassis = s.box(2.4, 0.6, start, {material: material(4, 0.1, 0.4, 0.3)});
        const tire = material(1, 0.1, 1.4, 1.2);
        const wheels = [];
        let rear = null;
        for (const side of [-1, 1]) {
            const mount = {x: 0.85 * side, y: -0.35};
            const wheel = s.ball(0.45, {x: start.x + mount.x, y: start.y + mount.y}, {material: tire});
            const suspension = s.wheel(chassis, wheel, {x: 0, y: 1}, mount, ZERO);
            suspension.setSpring(3, 0.7);
            suspension.setLimits(true, -0.3, 0.3);
            wheels.push(wheel);
            if (side < 0) rear = suspension;
        }

        s.goal = {record: goal, inside: false, entries: 0, exits: 0, car: {chassis, wheels}};
        // A car part is "in" from its sensor Begin to its sensor End; the car
        // enters when its first part does and leaves when its last part does.
        const carIds = new Set([chassis.id, ...wheels.map(w => w.id)]);
        const touching = new Set();
        s.onCollisionEvent(event => {
            if (!event.sensor) return;
            const other = event.bodyAId === goal.id ? event.bodyBId : event.bodyBId === goal.id ? event.bodyAId : null;
            if (other === null || !carIds.has(other)) return;
            const before = touching.size;
            if (event.phase === s.physics.CollisionPhase.End) touching.delete(other);
            else touching.add(other);
            if (before === 0 && touching.size > 0) ++s.goal.entries;
            if (before > 0 && touching.size === 0) ++s.goal.exits;
            s.goal.inside = touching.size > 0;
        });
        s.controllers.push(scene => {
            const throttle = scene.input.throttle;
            if (throttle !== 0) chassis.body.wake();
            rear.setMotor(true, -throttle * CAR_TOP_SPEED, throttle !== 0 ? CAR_DRIVE_TORQUE : CAR_IDLE_TORQUE);
        });
        s.status = () => `goal entries ${s.goal.entries}${s.goal.inside ? '  [CAR IN GOAL]' : ''}` +
            `   speed ${chassis.body.getVelocity().x.toFixed(1)} m/s`;
    }

    // A welded cantilever, a soft weld, a rope, a spring and a wheel suspension
    // rig, each carrying or hit by something.
    function buildGallery(s) {
        s.arena();
        const plank = material(1, 0.1, 0.6, 0.4);
        const heavy = material(4, 0.1, 0.6, 0.4);

        // Rigid weld: a cantilever of five segments, with a ball dropped on its tip.
        const segments = 5, segment = 1;
        const wall = {x: -9.45, y: 2.4};
        let previous = s.box(0.7, 1, wall, {isStatic: true});
        for (let i = 0; i < segments; ++i) {
            const part = s.box(segment, 0.25, {x: wall.x + 0.35 + segment * (i + 0.5), y: wall.y}, {material: plank});
            s.weld(previous, part, {x: i === 0 ? 0.35 : 0.5 * segment, y: 0}, {x: -0.5 * segment, y: 0});
            previous = part;
        }
        s.ball(0.4, {x: wall.x + 0.35 + segment * (segments - 0.5), y: 4.6}, {material: heavy});

        // Soft weld: one beam welded through an angular spring, so it sags and bounces.
        const post = {x: -9.45, y: -1.4};
        const stub = s.box(0.7, 1, post, {isStatic: true});
        const beam = s.box(4.4, 0.25, {x: post.x + 0.35 + 2.2, y: post.y}, {material: plank});
        s.weld(stub, beam, {x: 0.35, y: 0}, {x: -2.2, y: 0}).setAngularSpring(2, 0.15);
        s.ball(0.4, {x: -5.6, y: 0.9}, {material: heavy});

        // Rope: slack until the ball has fallen the full length.
        const ropeTop = {x: -1.2, y: 6.3};
        const ropeAnchor = s.anchor(ropeTop);
        const ropeBall = s.ball(0.5, {x: ropeTop.x + 3.2, y: ropeTop.y - 0.5}, {material: material(3, 0.1, 0.5, 0.4)});
        s.distance(ropeAnchor, ropeBall, 4, {range: [0, 4]});

        // Spring: a box on a soft distance joint, released stretched.
        const springTop = {x: 4.6, y: 6.3}, restLength = 2.5;
        const springAnchor = s.anchor(springTop);
        const bob = s.box(0.9, 0.9, {x: springTop.x, y: springTop.y - restLength - 1.2}, {material: material(2, 0.1, 0.5, 0.4)});
        s.distance(springAnchor, bob, restLength, {spring: [1.2, 0.08]});

        // Suspension rig: a wheel on a vertical spring axis fixed to the world,
        // with a stroke limit. A motorized arm sweeps under it like a cam, so
        // the wheel keeps bouncing against its spring.
        const rig = {x: 7.5, y: 0};
        const frame = s.anchor(rig, 0.4);
        s.box(0.12, 2.6, rig, {isStatic: true, mask: 0}); // guide rail
        const wheel = s.ball(0.55, rig, {material: material(1, 0.1, 0.8, 0.6)});
        const strut = s.wheel(frame, wheel, {x: 0, y: 1});
        strut.setSpring(2, 0.3);
        strut.setLimits(true, -1.1, 0.5);
        const pivot = {x: rig.x, y: -2.2}, armLength = 2, armStart = -1.2;
        const hub = s.anchor(pivot, 0.4);
        const arm = s.box(armLength, 0.25, {
            x: pivot.x + 0.5 * armLength * Math.cos(armStart), y: pivot.y + 0.5 * armLength * Math.sin(armStart),
        }, {material: material(2, 0.1, 0.5, 0.4), angle: armStart});
        s.revolute(hub, arm, ZERO, {x: -0.5 * armLength, y: 0}).setMotor(true, 1.6, 5000);
    }

    const DEFAULT_ITERATIONS = 10, STACKED_ITERATIONS = 20;
    const SCENES = [
        {name: 'Playground', description: 'Two boxes and a ball; spawn more and drag them.',
            solverIterations: DEFAULT_ITERATIONS, build: buildPlayground},
        {name: 'Tower', description: 'Twelve boxes stacked with small offsets.',
            solverIterations: STACKED_ITERATIONS, build: buildTower},
        {name: 'Pyramid', description: 'A 36-box pyramid held up by friction.',
            solverIterations: STACKED_ITERATIONS, build: buildPyramid},
        {name: 'Dominoes', description: 'Twenty thin boxes set off by a rolling ball.',
            solverIterations: STACKED_ITERATIONS, build: buildDominoes},
        {name: 'Chain and pendulum',
            description: 'Revolute capsule chain and a distance-joint pendulum hitting a stack.',
            solverIterations: STACKED_ITERATIONS, build: buildChainAndPendulum},
        {name: 'Capsules and ramps', description: 'Capsules and balls sliding down static ramps.',
            solverIterations: DEFAULT_ITERATIONS, build: buildRamps},
        {name: 'Seesaw and mass ratio',
            description: 'A heavy ball launches a light box from a hinged plank.',
            solverIterations: STACKED_ITERATIONS, build: buildSeesaw},
        {name: 'Machines', description: 'A motorized paddle and roller, and a prismatic slider.',
            solverIterations: STACKED_ITERATIONS, build: buildMachines},
        {name: 'Car and sensors',
            description: 'A car on wheel-joint suspension drives to a sensor goal; the HUD counts entries.',
            controls: 'Left/Right or A/D drive',
            solverIterations: STACKED_ITERATIONS, build: buildCar},
        {name: 'Joints gallery',
            description: 'Weld cantilever, soft weld, rope, spring and wheel suspension rig.',
            solverIterations: STACKED_ITERATIONS, build: buildGallery},
    ];

    return {
        SCENES,
        ARENA: {minX: -10, maxX: 10, floorTop: FLOOR_TOP},
        createScene(physics, index) { return new Scene(physics, SCENES[index]); },
        renderPose,
        anchorPoint,
        Scene,
    };
});

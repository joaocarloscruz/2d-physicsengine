// Browser front end for the web playground: canvas renderer, camera, input,
// HUD and the animation-frame loop. Simulation logic lives in scenes.js.
(function () {
    'use strict';

    const Playground = window.PlaygroundScenes;
    const $ = id => document.getElementById(id);
    const canvas = $('view');
    const ctx = canvas.getContext('2d');
    const hud = $('hud');
    const banner = $('banner');

    const MIN_ZOOM = 10, MAX_ZOOM = 200, DEFAULT_ZOOM = 40; // pixels per metre
    const MAX_BODIES = 500;
    const PAN_STEP_PIXELS = 24;
    const COLORS = {
        background: '#1e1e28',
        dynamic: 'rgb(100, 150, 255)',
        outline: 'rgb(200, 200, 200)',
        fixed: 'rgb(100, 100, 100)',
        ghost: 'rgba(100, 100, 100, 0.35)',
        dragged: 'rgb(255, 255, 100)',
        joint: 'rgb(255, 225, 110)',
        weld: 'rgb(240, 130, 220)',
        wheel: 'rgb(90, 215, 235)',
        wheelStroke: 'rgba(90, 215, 235, 0.3)',
        spring: 'rgb(130, 230, 140)',
        rope: 'rgb(215, 175, 120)',
        marker: 'rgb(30, 30, 40)',
        zone: 'rgba(70, 220, 140, 0.2)',
        zoneOutline: 'rgba(70, 220, 140, 0.7)',
        zoneActive: 'rgba(255, 210, 70, 0.47)',
        zoneActiveOutline: 'rgb(255, 225, 110)',
    };
    const SPRING_TEETH = 8, SPRING_AMPLITUDE = 5; // pixels

    const state = {
        physics: null,
        scene: null,
        sceneIndex: 0,
        paused: false,
        alpha: 1,
        camera: {x: 0, y: 0, zoom: DEFAULT_ZOOM},
        showHud: true,
        keys: new Set(),          // lower-case keys currently held, for driving
        pointer: null,            // last pointer position in CSS pixels, or null
        mode: null,               // 'drag' | 'pan' while a button is held
        fps: 0,
        lastFrame: 0,
        lastHud: 0,
        failed: false,
    };

    function fail(message) {
        state.failed = true;
        banner.textContent = message;
        banner.classList.add('open');
    }

    // ---- camera ---------------------------------------------------------------

    const viewSize = () => ({width: canvas.clientWidth, height: canvas.clientHeight});

    function toWorld(px, py) {
        const {width, height} = viewSize(), c = state.camera;
        return {x: c.x + (px - width / 2) / c.zoom, y: c.y - (py - height / 2) / c.zoom};
    }

    function toScreen(point) {
        const {width, height} = viewSize(), c = state.camera;
        return {x: width / 2 + (point.x - c.x) * c.zoom, y: height / 2 - (point.y - c.y) * c.zoom};
    }

    function zoomAt(px, py, factor) {
        const c = state.camera;
        const before = toWorld(px, py);
        c.zoom = Math.min(MAX_ZOOM, Math.max(MIN_ZOOM, c.zoom * factor));
        const after = toWorld(px, py);
        c.x += before.x - after.x;
        c.y += before.y - after.y;
    }

    function resize() {
        const dpr = window.devicePixelRatio || 1;
        canvas.width = Math.max(1, Math.round(canvas.clientWidth * dpr));
        canvas.height = Math.max(1, Math.round(canvas.clientHeight * dpr));
    }

    // ---- scenes ---------------------------------------------------------------

    function loadScene(index, resetCamera) {
        const next = Playground.createScene(state.physics, index);
        if (state.scene) state.scene.dispose();
        state.scene = next;
        state.sceneIndex = index;
        state.alpha = 1;
        if (resetCamera) Object.assign(state.camera, {x: 0, y: 0, zoom: DEFAULT_ZOOM});
        for (const button of document.querySelectorAll('#scenes button')) {
            button.setAttribute('aria-pressed', String(Number(button.dataset.scene) === index));
        }
        document.title = `${Playground.SCENES[index].name} - 2D Physics Engine Playground`;
    }

    // Keys 1-9 pick scenes 1-9; key 0 picks scene 10.
    const sceneKey = index => String((index + 1) % 10);
    const sceneForKey = key => (key >= '0' && key <= '9' && key.length === 1)
        ? (key === '0' ? 9 : Number(key) - 1) : -1;

    // Right/D minus Left/A, while the scene has a vehicle to drive.
    function throttle() {
        const keys = state.keys;
        return (keys.has('arrowright') || keys.has('d') ? 1 : 0) - (keys.has('arrowleft') || keys.has('a') ? 1 : 0);
    }
    const sceneDrives = () => Boolean(Playground.SCENES[state.sceneIndex].controls);

    function buildSceneButtons() {
        const container = $('scenes');
        Playground.SCENES.forEach((scene, index) => {
            const button = document.createElement('button');
            button.type = 'button';
            button.dataset.scene = String(index);
            button.title = scene.description;
            button.setAttribute('aria-pressed', 'false');
            button.textContent = `${sceneKey(index)} ${scene.name}`;
            container.appendChild(button);
        });
    }

    function spawnShape(kind, world) {
        const scene = state.scene;
        if (scene.bodyCount >= MAX_BODIES) return;
        const at = world || (state.pointer ? toWorld(state.pointer.x, state.pointer.y)
            : {x: state.camera.x, y: state.camera.y});
        scene.spawn(kind, at.x, at.y);
        if (state.paused) { scene.readPoses(); scene.capturePrevious(); }
    }

    function togglePause() {
        state.paused = !state.paused;
        $('pause').firstChild.textContent = state.paused ? 'Resume ' : 'Pause ';
    }

    function stepOnce() {
        if (!state.paused) togglePause();
        state.scene.input.throttle = sceneDrives() ? throttle() : 0;
        state.scene.stepOnce();
        state.alpha = 1;
    }

    function toggleHelp(force) {
        const open = force === undefined ? !$('help').classList.contains('open') : force;
        $('help').classList.toggle('open', open);
        $('helpButton').setAttribute('aria-expanded', String(open));
    }

    // ---- input ----------------------------------------------------------------

    function localPoint(event) {
        const rect = canvas.getBoundingClientRect();
        return {x: event.clientX - rect.left, y: event.clientY - rect.top};
    }

    canvas.addEventListener('contextmenu', event => event.preventDefault());

    canvas.addEventListener('pointerdown', event => {
        if (state.mode || !state.scene) return;
        canvas.setPointerCapture(event.pointerId);
        const p = localPoint(event);
        state.pointer = p;
        state.mode = 'pan';
        if (event.button === 0) {
            const world = toWorld(p.x, p.y);
            const record = state.scene.pick(world.x, world.y);
            if (record) {
                state.scene.beginDrag(record, world.x, world.y);
                state.mode = 'drag';
            }
        }
        canvas.classList.add('dragging');
    });

    canvas.addEventListener('pointermove', event => {
        const p = localPoint(event);
        const previous = state.pointer;
        state.pointer = p;
        if (state.mode === 'drag') {
            const world = toWorld(p.x, p.y);
            state.scene.setDragTarget(world.x, world.y);
        } else if (state.mode === 'pan' && previous) {
            state.camera.x -= (p.x - previous.x) / state.camera.zoom;
            state.camera.y += (p.y - previous.y) / state.camera.zoom;
        }
    });

    function endPointer() {
        if (state.scene) state.scene.endDrag();
        state.mode = null;
        canvas.classList.remove('dragging');
    }
    canvas.addEventListener('pointerup', endPointer);
    canvas.addEventListener('pointercancel', endPointer);
    canvas.addEventListener('pointerleave', () => { if (!state.mode) state.pointer = null; });

    canvas.addEventListener('wheel', event => {
        event.preventDefault();
        const p = localPoint(event);
        zoomAt(p.x, p.y, event.deltaY < 0 ? 1.1 : 1 / 1.1);
    }, {passive: false});

    window.addEventListener('keydown', event => {
        if (!state.scene || event.ctrlKey || event.metaKey || event.altKey) return;
        const key = event.key.toLowerCase();
        const scene = sceneForKey(key);
        const pan = PAN_STEP_PIXELS / state.camera.zoom;
        // In a scene with a vehicle, Left/Right drive it; Shift+arrows still pan.
        const panHorizontally = event.shiftKey || !sceneDrives();
        if (key === 'a' || key === 'd' || ((key === 'arrowleft' || key === 'arrowright') && !event.shiftKey)) {
            state.keys.add(key);
        }
        if (key === 'arrowleft' && panHorizontally) state.camera.x -= pan;
        else if (key === 'arrowright' && panHorizontally) state.camera.x += pan;
        else if (key === 'arrowleft' || key === 'arrowright') { /* drive: handled each frame */ }
        else if (key === 'arrowup') state.camera.y += pan;
        else if (key === 'arrowdown') state.camera.y -= pan;
        else if (event.repeat) return;
        else if (scene >= 0 && scene < Playground.SCENES.length) loadScene(scene, true);
        else if (key === ' ') togglePause();
        else if (key === 'n' || key === '.') stepOnce();
        else if (key === 'r') loadScene(state.sceneIndex, false);
        else if (key === 'c') spawnShape('circle');
        else if (key === 'b') spawnShape('box');
        else if (key === 'k') spawnShape('capsule');
        else if (key === 'tab' || (key === 'd' && !sceneDrives())) state.showHud = !state.showHud;
        else if (key === 'a' || key === 'd') { /* drive: handled each frame */ }
        else if (key === 'h') toggleHelp();
        else if (key === 'escape') toggleHelp(false);
        else return;
        event.preventDefault();
    });
    window.addEventListener('keyup', event => state.keys.delete(event.key.toLowerCase()));
    window.addEventListener('blur', () => state.keys.clear());

    // Buttons must not keep focus, or Space would press them again.
    document.addEventListener('mousedown', event => {
        if (event.target.closest('button')) event.preventDefault();
    });
    $('pause').addEventListener('click', togglePause);
    $('step').addEventListener('click', () => stepOnce());
    $('reset').addEventListener('click', () => loadScene(state.sceneIndex, false));
    $('helpButton').addEventListener('click', () => toggleHelp());
    for (const button of document.querySelectorAll('[data-spawn]')) {
        button.addEventListener('click', () => spawnShape(button.dataset.spawn,
            {x: state.camera.x + (Math.random() - 0.5) * 2, y: state.camera.y + 3}));
    }
    $('scenes').addEventListener('click', event => {
        const button = event.target.closest('button[data-scene]');
        if (button) loadScene(Number(button.dataset.scene), true);
    });
    window.addEventListener('resize', resize);

    // ---- drawing --------------------------------------------------------------

    function bodyPath(r) {
        ctx.beginPath();
        if (r.kind === 'circle') {
            ctx.arc(0, 0, r.radius, 0, 2 * Math.PI);
        } else if (r.kind === 'box') {
            ctx.rect(-r.width / 2, -r.height / 2, r.width, r.height);
        } else {
            // Capsule: a spine of half-length halfLength along the local x axis.
            ctx.arc(r.halfLength, 0, r.radius, -Math.PI / 2, Math.PI / 2);
            ctx.arc(-r.halfLength, 0, r.radius, Math.PI / 2, 1.5 * Math.PI);
            ctx.closePath();
        }
    }

    function drawBody(r, dragged) {
        const pose = Playground.renderPose(r, state.alpha);
        const s = toScreen(pose);
        const zoom = state.camera.zoom;
        ctx.save();
        ctx.translate(s.x, s.y);
        ctx.scale(zoom, -zoom);
        ctx.rotate(pose.a);
        bodyPath(r);
        const active = r.sensor && state.scene.goal && state.scene.goal.inside;
        ctx.fillStyle = dragged ? COLORS.dragged
            : r.sensor ? (active ? COLORS.zoneActive : COLORS.zone)
            : r.ghost ? COLORS.ghost : r.isStatic ? COLORS.fixed : COLORS.dynamic;
        ctx.fill();
        ctx.lineWidth = (r.sensor ? 2 : 1.5) / zoom;
        ctx.strokeStyle = dragged ? COLORS.dragged
            : r.sensor ? (active ? COLORS.zoneActiveOutline : COLORS.zoneOutline) : COLORS.outline;
        if (!r.ghost) ctx.stroke();
        if (!r.isStatic) { // orientation tick
            ctx.beginPath();
            ctx.moveTo(0, 0);
            ctx.lineTo(r.kind === 'circle' ? r.radius : r.kind === 'capsule' ? r.halfLength : r.width / 2, 0);
            ctx.strokeStyle = 'rgba(255, 255, 255, 0.85)';
            ctx.stroke();
        }
        ctx.restore();
    }

    // A zig-zag between two screen points, for spring-like joints.
    function zigzag(a, b) {
        const dx = b.x - a.x, dy = b.y - a.y, length = Math.hypot(dx, dy);
        ctx.beginPath();
        ctx.moveTo(a.x, a.y);
        if (length >= 2 * SPRING_AMPLITUDE) {
            const nx = -dy / length, ny = dx / length;
            for (let i = 0; i < 2 * SPRING_TEETH; ++i) {
                const t = (i + 0.5) / (2 * SPRING_TEETH), offset = i % 2 === 0 ? SPRING_AMPLITUDE : -SPRING_AMPLITUDE;
                ctx.lineTo(a.x + dx * t + nx * offset, a.y + dy * t + ny * offset);
            }
        }
        ctx.lineTo(b.x, b.y);
        ctx.stroke();
    }

    const JOINT_COLORS = {weld: COLORS.weld, wheel: COLORS.wheel, spring: COLORS.spring, rope: COLORS.rope};

    function drawJoints() {
        const alpha = state.alpha;
        ctx.lineWidth = 1.5;
        for (const j of state.scene.joints) {
            const a = toScreen(Playground.anchorPoint(j.a, j.localA, alpha));
            const b = toScreen(Playground.anchorPoint(j.b, j.localB, alpha));
            const color = JOINT_COLORS[j.kind] || COLORS.joint;
            ctx.strokeStyle = color;
            if (j.kind === 'wheel') {
                // The suspension stroke along the axis through the chassis anchor.
                const axis = j.joint.getAxis();
                const lower = j.joint.areLimitsEnabled() ? j.joint.getLowerLimit() : -0.3;
                const upper = j.joint.areLimitsEnabled() ? j.joint.getUpperLimit() : 0.3;
                const base = Playground.anchorPoint(j.a, j.localA, alpha);
                const low = toScreen({x: base.x + axis.x * lower, y: base.y + axis.y * lower});
                const high = toScreen({x: base.x + axis.x * upper, y: base.y + axis.y * upper});
                ctx.save();
                ctx.lineWidth = 5;
                ctx.strokeStyle = COLORS.wheelStroke;
                ctx.beginPath();
                ctx.moveTo(low.x, low.y);
                ctx.lineTo(high.x, high.y);
                ctx.stroke();
                ctx.restore();
            }
            if (j.kind === 'spring') {
                zigzag(a, b);
            } else {
                ctx.beginPath();
                ctx.moveTo(a.x, a.y);
                ctx.lineTo(b.x, b.y);
                ctx.stroke();
            }
            // Round markers for revolute and wheel joints, hexagons for welds, squares otherwise.
            for (const p of [a, b]) {
                ctx.beginPath();
                if (j.kind === 'revolute' || j.kind === 'wheel') {
                    ctx.arc(p.x, p.y, 3.5, 0, 2 * Math.PI);
                } else if (j.kind === 'weld') {
                    for (let i = 0; i < 6; ++i) {
                        const t = i * Math.PI / 3;
                        ctx.lineTo(p.x + 3.5 * Math.cos(t), p.y + 3.5 * Math.sin(t));
                    }
                    ctx.closePath();
                } else {
                    ctx.rect(p.x - 3.5, p.y - 3.5, 7, 7);
                }
                ctx.fillStyle = COLORS.marker;
                ctx.fill();
                ctx.stroke();
            }
        }
    }

    function drawDragLine() {
        const drag = state.scene.drag;
        if (!drag) return;
        const from = toScreen(Playground.anchorPoint(drag.record, drag.localAnchor, state.alpha));
        const to = toScreen(drag.target);
        ctx.strokeStyle = COLORS.dragged;
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.moveTo(from.x, from.y);
        ctx.lineTo(to.x, to.y);
        ctx.stroke();
    }

    function draw() {
        const dpr = canvas.width / Math.max(canvas.clientWidth, 1);
        ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
        ctx.fillStyle = COLORS.background;
        ctx.fillRect(0, 0, canvas.clientWidth, canvas.clientHeight);
        const scene = state.scene;
        const dragged = scene.drag && scene.drag.record;
        // Sensor zones first, then fixed bodies, so dynamic ones are never hidden behind them.
        for (const r of scene.bodies) if (r.sensor) drawBody(r, false);
        for (const r of scene.bodies) if (r.isStatic && !r.sensor) drawBody(r, false);
        for (const r of scene.bodies) if (!r.isStatic) drawBody(r, r === dragged);
        drawJoints();
        drawDragLine();
    }

    function updateHud(now) {
        if (!state.showHud) { hud.style.display = 'none'; return; }
        hud.style.display = '';
        if (now - state.lastHud < 200) return;
        state.lastHud = now;
        const scene = state.scene;
        hud.textContent =
            `${state.sceneIndex + 1}/${Playground.SCENES.length} ${Playground.SCENES[state.sceneIndex].name}\n` +
            `bodies ${scene.bodyCount}   joints ${scene.jointCount}\n` +
            (scene.status ? scene.status() + '\n' : '') +
            `step   ${scene.stepMilliseconds.toFixed(2)} ms\n` +
            `fps    ${state.fps.toFixed(0)}` + (state.paused ? '\nPAUSED' : '');
    }

    // ---- loop -----------------------------------------------------------------

    function frame(now) {
        if (state.failed) return;
        try {
            const elapsed = state.lastFrame ? (now - state.lastFrame) / 1000 : 0;
            state.lastFrame = now;
            if (elapsed > 0) {
                const instant = 1 / Math.max(elapsed, 1e-3);
                state.fps = state.fps === 0 ? instant : state.fps + 0.1 * (instant - state.fps);
            }
            if (!state.paused) {
                state.scene.input.throttle = sceneDrives() ? throttle() : 0;
                state.alpha = state.scene.advanceFrame(elapsed).alpha;
                state.scene.cull();
            }
            draw();
            updateHud(now);
            requestAnimationFrame(frame);
        } catch (error) {
            console.error(error);
            fail('The simulation stopped: ' + (error && error.message ? error.message : String(error)));
        }
    }

    function start(physics) {
        state.physics = physics;
        buildSceneButtons();
        loadScene(0, true);
        resize();
        // Read-only view of the running state, for debugging from the console.
        window.playgroundDebug = () => ({
            scene: Playground.SCENES[state.sceneIndex].name, bodies: state.scene.bodyCount,
            joints: state.scene.jointCount, paused: state.paused, stepMs: state.scene.stepMilliseconds,
            fps: state.fps, sceneTime: state.scene.sceneTime, camera: {...state.camera},
        });
        requestAnimationFrame(frame);
    }

    resize();
    if (window.playgroundLoadFailed || typeof window.createPhysicsEngineModule !== 'function') {
        fail('Could not load physics_engine.js. Serve this folder over HTTP (for example ' +
            '"python -m http.server" from build-wasm/wasm) after building the WebAssembly target; ' +
            'opening the file directly cannot load the .wasm module.');
    } else {
        window.createPhysicsEngineModule().then(start).catch(error => {
            console.error(error);
            fail('Could not start the physics module: ' + (error && error.message ? error.message : String(error)));
        });
    }
}());

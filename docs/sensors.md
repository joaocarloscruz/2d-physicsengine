# Sensors and user data

## Sensors

`body->SetSensor(true)` turns a body into a sensor. A sensor still reports
overlaps through the usual collision events, `onCollisionBegin`, `onCollisionPersist`
and `onCollisionEnd`, with `CollisionEvent::sensor` set to `true`. It never gets a
contact constraint or an impulse. It does not push other bodies and they do not
push it. The rules follow Box2D where that makes sense:

| Situation | Behaviour |
| --- | --- |
| Sensor and non-sensor overlap | Begin on the first step they overlap, Persist each step after, End on the first step they do not. |
| Sensor and sensor | No events, whether static or dynamic. |
| Static sensor and static body | No events. Static pairs are never tested. |
| Static sensor moved with `SetPosition` | Works as a kinematic trigger. Overlaps are tested at its new position. |
| Dynamic sensor | Integrates forces and gravity like any body, so it falls through everything and reports what it passes. |
| Collision filtering | Applies. A sensor reports a body only when the category and mask bits let the two collide. Jointed pairs that do not collide report nothing. |
| Sleeping | A sensor never wakes a body. Its overlaps do not link islands, so a sensor does not keep a group awake. A body asleep inside a sensor keeps getting Persist events. |
| Speculative margin | Only a real overlap at the end of the step counts. A body that is merely close is not reported, and the speculative-contact touch tolerance does not apply. |
| CCD | Sensors are not swept. A fast body can cross a thin sensor within one step and raise no event. Make the sensor thicker, or the step smaller. |
| Statistics | Sensor overlaps add nothing to `resolvedContactCount`, `activeContactCount`, `solvedConstraintCount` or `getPersistentContactCount()`. They are included in `narrowPhaseCandidateCount` and in `getPotentialCollisions()`, because they are tested. |

Sensor overlaps are tested once per step at the end-of-step poses, against the
pairs the broad phase found, for both step orders (`enableSpeculativeContacts`
on or off). A sensor adds no work to the solver. A body touching the sensor
exactly at its edge can flicker between overlapping and not, because the test
is exact.

### Changing the flag

`SetSensor` wakes the body and the bodies touching it. Turning a static floor
into a sensor wakes the bodies resting on it, so they fall through. Turning a
sensor back into a solid body wakes the bodies inside it, so they are pushed
out. A pair that changes kind ends one lifecycle and begins another in the
same step. The contact End comes first, then the sensor Begin, or the reverse.

Changing the flag from a collision callback takes effect from the next step.

### Telling sensor events apart

```cpp
class Pickups : public ICollisionListener {
public:
    void onCollisionBegin(const CollisionEvent& event) override {
        if (event.sensor) collected.push_back(event.bodyAId == coinId ? event.bodyBId : event.bodyAId);
    }
    std::uint64_t coinId = 0;
    std::vector<std::uint64_t> collected;
};
```

The event's normal, penetration and contact points describe the overlap
geometry. Nothing is applied with them. `onCollision(const CollisionManifold&)`
also fires for sensor Begin and Persist events. Check `manifold.A->IsSensor()`
or `manifold.B->IsSensor()` there.

### Event order

Events from one step are delivered in this order:

1. Contact Begin and Persist events, in narrow-phase order.
2. CCD impact events.
3. Sensor Begin and Persist events, in broad-phase pair order.
4. End events, sorted by `(bodyAId, bodyBId)`.

The exception is a pair that changed kind through `SetSensor`. Its End comes
immediately before its new Begin. `bodyAId` is always the lower id. Removing a
body raises End events for its pairs straight away, sorted the same way. A removed sensor ends its overlaps
without waking anyone. The same scene stepped the same way always produces
the same events.

## User data

`body->SetUserData(value)` stores one `std::uintptr_t`, and `GetUserData()`
returns it. The engine never reads it or changes it. It starts at 0.

It is an integer, not a `void*`, for three reasons:

- An entity index or handle fits directly, with no cast and no dangling pointer.
- A pointer still fits: `reinterpret_cast<std::uintptr_t>(ptr)`.
- It maps to a JavaScript number without Embind's raw-pointer handling. In
  JavaScript, `setUserData` accepts integers from 0 to 2^32 - 1, because
  `uintptr_t` is 32 bits in wasm32. It throws for anything else, including
  fractions, negatives and NaN.

The value lives on the body, so it is shared by every world that holds the
body. It is not part of `ExportWorldJson` or `ExportWorldCsv`.

## JavaScript

```js
const coin = physics.createRigidBody(shape, material, {x: 0, y: 2}, true);
coin.setSensor(true);
coin.setUserData(17);
coin.isSensor();    // true
coin.getUserData(); // 17
```

The JavaScript bindings do not expose collision listeners yet. A JavaScript
caller can still turn sensors on and read `"sensor"` in `exportJson`.

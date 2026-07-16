# Collision filtering

Each `RigidBody` has a 32-bit category and mask. A pair is eligible for
collision detection only when both bodies opt into the interaction:

```cpp
(bodyA.category & bodyB.mask) != 0
    && (bodyB.category & bodyA.mask) != 0
```

New bodies use category `0x00000001` and mask `0xFFFFFFFF`, so existing code
continues to collide with every category.

```cpp
constexpr std::uint32_t playerCategory = 0x00000001;
constexpr std::uint32_t enemyCategory = 0x00000002;

player.SetCollisionCategoryBits(playerCategory);
player.SetCollisionMaskBits(enemyCategory);

enemy.SetCollisionCategoryBits(enemyCategory);
enemy.SetCollisionMaskBits(playerCategory);
```

Setting a mask to zero disables collisions for that body. Filtered pairs do not
reach narrow-phase detection, collision listeners, resolution, or the persistent
contact cache. Changing a filter takes effect during the next `World::step`.

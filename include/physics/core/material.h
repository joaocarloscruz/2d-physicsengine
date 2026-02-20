#ifndef MATERIAL_H
#define MATERIAL_H

namespace PhysicsEngine
{
    struct Material
    {
        float density;
        float restitution;
        float staticFriction = 0.6f;
        float dynamicFriction = 0.4f;
    };
}

#endif // MATERIAL_H

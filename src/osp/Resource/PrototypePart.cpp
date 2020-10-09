#include "PrototypePart.h"
#include <iostream>

namespace osp
{

PrototypePart::PrototypePart()
{

}

float col_shape_volume(ECollisionShape shape, Vector3 scale)
{
    constexpr float pi = 3.14159265359f;
    switch (shape)
    {
    case osp::ECollisionShape::NONE:
        return 0.0f;
    case osp::ECollisionShape::SPHERE:
        // Default radius: 1
        return (4.0f/3.0f)*pi*scale.x()*scale.x()*scale.x();
    case osp::ECollisionShape::BOX:
        // Default width: 2
        return 2.0f*scale.x() * 2.0f*scale.y() * 2.0f*scale.z();
    case osp::ECollisionShape::CYLINDER:
        // Default radius: 1, default height: 2
        return pi * scale.x()*scale.x() * 2.0f*scale.z();
    case osp::ECollisionShape::CAPSULE:
    case osp::ECollisionShape::CONVEX_HULL:
    case osp::ECollisionShape::TERRAIN:
    case osp::ECollisionShape::COMBINED:
    default:
        std::cout << "Error: unsupported shape for volume calc\n";
        return 0.0f;
    }
}

}

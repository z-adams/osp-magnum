#pragma once

#include <string>
#include <Magnum/Math/Color.h>
#include <Magnum/Magnum.h>

struct PlumeEffectData
{
    float flowVelocity;
    Magnum::Color4 color;
    float zMin;
    float zMax;
    std::string meshName;
};
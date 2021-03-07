/**
 * Open Space Program
 * Copyright © 2019-2020 Open Space Program Project
 *
 * MIT License
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#pragma once

#include "types.h"
#include "CommonPhysics.h"
#include "CommonMath.h"
#include <Magnum/GL/CubeMapTexture.h>
#include <Magnum/PixelFormat.h>
#include <Corrade/PluginManager/PluginManager.h>
#include <MagnumPlugins/AnyImageImporter/AnyImageImporter.h>
#include <MagnumPlugins/AnyImageConverter/AnyImageConverter.h>
#include <Magnum/Trade/ImageData.h>
#include <Magnum/Math/Color.h>
#include <Corrade/Containers/Optional.h>
#include <Corrade/Containers/StridedArrayView.h>
#include <vector>
#include <array>
#include <iostream>

#include <Magnum/GL/AbstractShaderProgram.h>
#include <Magnum/GL/Attribute.h>
#include <Magnum/GL/Texture.h>
#include <Magnum/GL/Shader.h>

namespace osp::math::cubemap
{

// Source: https://en.wikipedia.org/wiki/Cube_mapping
inline Vector3 convert_cube_uv_to_xyz(Magnum::GL::CubeMapCoordinate index, Vector2 uv)
{
    float x = 0.0f, y = 0.0f, z = 0.0f;

    // convert range 0 to 1 to -1 to 1
    float uc = 2.0f * uv.x() - 1.0f;
    float vc = 2.0f * uv.y() - 1.0f;

    using Magnum::GL::CubeMapCoordinate;
    switch (index)
    {
    case CubeMapCoordinate::PositiveX:
        x = 1.0f;
        y = vc;
        z = -uc;
        break;
    case CubeMapCoordinate::NegativeX:
        x = -1.0f;
        y = vc;
        z = uc;
        break;
    case CubeMapCoordinate::PositiveY:
        x = uc;
        y = 1.0f;
        z = -vc;
        break;
    case CubeMapCoordinate::NegativeY:
        x = uc;
        y = -1.0f;
        z = vc;
        break;
    case CubeMapCoordinate::PositiveZ:
        x = uc;
        y = vc;
        z = 1.0f;
        break;
    case CubeMapCoordinate::NegativeZ:
        x = -uc;
        y = vc;
        z = -1.0f;
        break;
    }

    return Vector3{x, y, z};
}

inline std::pair<Magnum::GL::CubeMapCoordinate, Magnum::Vector2>
convert_xyz_to_cube_uv(Magnum::Vector3 r)
{
    float x = r.x();
    float y = r.y();
    float z = r.z();

    float absX = fabs(x);
    float absY = fabs(y);
    float absZ = fabs(z);

    int isXPositive = x > 0 ? 1 : 0;
    int isYPositive = y > 0 ? 1 : 0;
    int isZPositive = z > 0 ? 1 : 0;

    float maxAxis = 0.0f;
    float uc = 0.0f;
    float vc = 0.0f;
    using Magnum::GL::CubeMapCoordinate;
    CubeMapCoordinate index{CubeMapCoordinate::PositiveX};

    // POSITIVE X
    if (isXPositive && absX >= absY && absX >= absZ)
    {
        // u (0 to 1) goes from +z to -z
        // v (0 to 1) goes from -y to +y
        maxAxis = absX;
        uc = -z;
        vc = y;
        index = CubeMapCoordinate::PositiveX;
    }
    // NEGATIVE X
    if (!isXPositive && absX >= absY && absX >= absZ)
    {
        // u (0 to 1) goes from -z to +z
        // v (0 to 1) goes from -y to +y
        maxAxis = absX;
        uc = z;
        vc = y;
        index = CubeMapCoordinate::NegativeX;
    }
    // POSITIVE Y
    if (isYPositive && absY >= absX && absY >= absZ)
    {
        // u (0 to 1) goes from -x to +x
        // v (0 to 1) goes from +z to -z
        maxAxis = absY;
        uc = x;
        vc = -z;
        index = CubeMapCoordinate::PositiveY;
    }
    // NEGATIVE Y
    if (!isYPositive && absY >= absX && absY >= absZ)
    {
        // u (0 to 1) goes from -x to +x
        // v (0 to 1) goes from -z to +z
        maxAxis = absY;
        uc = x;
        vc = z;
        index = CubeMapCoordinate::NegativeY;
    }
    // POSITIVE Z
    if (isZPositive && absZ >= absX && absZ >= absY)
    {
        // u (0 to 1) goes from -x to +x
        // v (0 to 1) goes from -y to +y
        maxAxis = absZ;
        uc = x;
        vc = y;
        index = CubeMapCoordinate::PositiveZ;
    }
    // NEGATIVE Z
    if (!isZPositive && absZ >= absX && absZ >= absY)
    {
        // u (0 to 1) goes from +x to -x
        // v (0 to 1) goes from -y to +y
        maxAxis = absZ;
        uc = -x;
        vc = y;
        index = CubeMapCoordinate::NegativeZ;
    }

    // Convert range from -1 to 1 to 0 to 1
    float u = 0.5f * (uc / maxAxis + 1.0f);
    float v = 0.5f * (vc / maxAxis + 1.0f);

    return {index, Magnum::Vector2{ u, v }};
}

inline void equirectangular_to_cubemap(std::string_view equirectangularImagePath)
{
    size_t cubemapWidth = 2048;

    using Corrade::PluginManager::Manager;
    using Magnum::Trade::ImageData2D;
    using Magnum::PixelFormat;

    Manager<Magnum::Trade::AbstractImporter> pluginManager;
    Magnum::Trade::AnyImageImporter importer(pluginManager);

    Manager<Magnum::Trade::AbstractImageConverter> converterManager;
    Magnum::Trade::AnyImageConverter converter(converterManager);

    if (!importer.openFile(std::string{equirectangularImagePath}))
    {
        std::cout << "ERROR opening file " << equirectangularImagePath << "\n";
        return;
    }

    Corrade::Containers::Optional<ImageData2D> imageInput = importer.image2D(0);

    size_t xRes = imageInput->size().x();
    size_t yRes = imageInput->size().y();

    size_t uRes = cubemapWidth;
    size_t vRes = cubemapWidth;
    using Corrade::Containers::Array;
    std::array<Array<Magnum::Color3ub>, 6> arrayData
    {
        Array<Magnum::Color3ub>(Corrade::Containers::ValueInit, uRes * vRes),
        Array<Magnum::Color3ub>(Corrade::Containers::ValueInit, uRes * vRes),
        Array<Magnum::Color3ub>(Corrade::Containers::ValueInit, uRes * vRes),
        Array<Magnum::Color3ub>(Corrade::Containers::ValueInit, uRes * vRes),
        Array<Magnum::Color3ub>(Corrade::Containers::ValueInit, uRes * vRes),
        Array<Magnum::Color3ub>(Corrade::Containers::ValueInit, uRes * vRes)
    };
    std::vector<ImageData2D> cubemapOutput;
    cubemapOutput.reserve(6);
    for (size_t i = 0; i < 6; i++)
    {
        //Array<char> data(Corrade::Containers::ValueInit, 3*uRes*vRes);
        cubemapOutput.emplace_back(
            PixelFormat::RGB8Unorm,
            Vector2i{static_cast<int>(uRes), static_cast<int>(vRes)},
            Magnum::Trade::DataFlag::Mutable,
            arrayData[i]);
    }

    float du = 1.0f / uRes;
    float dv = 1.0f / vRes;

    using Magnum::GL::CubeMapCoordinate;
    std::vector<std::pair<CubeMapCoordinate, size_t>> coords
    {
        {CubeMapCoordinate::PositiveX, 0},
        {CubeMapCoordinate::NegativeX, 1},
        {CubeMapCoordinate::PositiveY, 2},
        {CubeMapCoordinate::NegativeY, 3},
        {CubeMapCoordinate::PositiveZ, 4},
        {CubeMapCoordinate::NegativeZ, 5}
    };

    auto pixelsIn = imageInput->mutablePixels<Magnum::Color3ub>();
    for (auto [direction, index] : coords)
    {
        auto pixelsOut = cubemapOutput[index].mutablePixels<Magnum::Color3ub>();

        for (float u = 0.0f; u < 1.0f; u += du)
        {
            for (float v = 0.0f; v < 1.0f; v += dv)
            {
                Vector3 xyz = convert_cube_uv_to_xyz(direction, {u, v});
                Vector3 rtp = cartesian_to_spherical(xyz);

                // (theta, phi) == (0, 0) should be the center of the image
                constexpr float s_pi = Magnum::Constants::pi();
                constexpr float s_2pi = 2.0f * Magnum::Constants::pi();
                float eqX = (rtp.z() + s_2pi) / s_2pi;
                float eqY = rtp.y() / s_pi;

                if (eqX >= 1.0f) { eqX -= 1.0f; }
                if (eqY >= 1.0f) { eqY -= 1.0f; }

                size_t uI = u * uRes;
                size_t vI = v * vRes;
                size_t jI = eqX * xRes;
                size_t iI = eqY * yRes;
                auto& pixelOut = pixelsOut[vI][uI];
                auto const& pixelIn = pixelsIn[iI][jI];
                pixelOut = pixelIn;
                //cubemapOutput[index][uRes * vI + uI] = input[xRes * iI + jI];
            }
        }
    }

    converter.exportToFile(cubemapOutput[0], "posx.png");
    converter.exportToFile(cubemapOutput[1], "negx.png");
    converter.exportToFile(cubemapOutput[2], "posy.png");
    converter.exportToFile(cubemapOutput[3], "negy.png");
    converter.exportToFile(cubemapOutput[4], "posz.png");
    converter.exportToFile(cubemapOutput[5], "negz.png");
}

class CubemapComputeShader : public Magnum::GL::AbstractShaderProgram
{
public:
    CubemapComputeShader();

    void process(std::string_view input, std::string_view outputPath);
private:
    // GL init
    void init();

    // Uniforms
    enum class UniformPos : Magnum::Int
    {
        InputTex = 0,
        OutputCube = 1
    };

    // Texture slots
    enum class TextureSlots : Magnum::Int
    {
        InputEquirectSlot = 0,
        OutputCubemapSlot = 1
    };

    // Hide irrelevant calls
    using Magnum::GL::AbstractShaderProgram::drawTransformFeedback;
    using Magnum::GL::AbstractShaderProgram::draw;

    CubemapComputeShader& bind_input_map(Magnum::GL::Texture2D& rTex);
    CubemapComputeShader& bind_output_cube(Magnum::GL::CubeMapTexture& rTex);
};

}

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

#include "CubemapUtils.h"
#include <Magnum/GL/Version.h>
#include <Magnum/ImageView.h>
#include <Magnum/Image.h>
#include <Magnum/GL/TextureFormat.h>
#include <Magnum/GL/ImageFormat.h>
#include <Magnum/Math/Swizzle.h>
#include <osp/string_concat.h>

using namespace osp::math::cubemap;
using namespace Magnum;


Corrade::Optional<Magnum::ImageData2D> load_image(std::string_view filename)
{
    
}

CubemapComputeShader::CubemapComputeShader()
{
    init();
}

void CubemapComputeShader::init()
{
    GL::Shader prog{GL::Version::GL430, GL::Shader::Type::Compute};
    prog.addFile("OSPData/adera/Shaders/CubemapCompute.comp");

    CORRADE_INTERNAL_ASSERT_OUTPUT(prog.compile());
    attachShader(prog);
    CORRADE_INTERNAL_ASSERT_OUTPUT(link());

    setUniform(
        static_cast<Int>(UniformPos::InputTex),
        static_cast<Int>(TextureSlots::InputEquirectSlot));
    setUniform(
        static_cast<Int>(UniformPos::OutputCube),
        static_cast<Int>(TextureSlots::OutputCubemapSlot));
}

void CubemapComputeShader::process(std::string_view input, std::string_view outputPath, size_t outputSize)
{
    size_t cubemapWidth = outputSize;

    using Corrade::PluginManager::Manager;
    using Magnum::Trade::ImageData2D;
    using Magnum::ImageView2D;
    using Magnum::PixelFormat;
    using GL::CubeMapCoordinate;
    using Corrade::Containers::Array;

    Manager<Magnum::Trade::AbstractImporter> importManager;
    Manager<Magnum::Trade::AbstractImageConverter> convertManager;

    Magnum::Trade::AnyImageImporter importer(importManager);
    Magnum::Trade::AnyImageConverter converter(convertManager);

    if (!importer.openFile(std::string{input}))
    {
        std::cout << "ERROR opening input file " << input << std::endl;
        return;
    }

    Corrade::Containers::Optional<ImageData2D> imageInput = importer.image2D(0);

    GL::Texture2D inputTex;
    inputTex.setWrapping(SamplerWrapping::ClampToEdge)
        .setStorage(1, GL::TextureFormat::RGBA8, imageInput->size())
        .setSubImage(0, {}, *imageInput);

    int uRes = cubemapWidth, vRes = cubemapWidth;
    std::array<Array<Magnum::Color4ub>, 6> arrayData
    {
        Array<Magnum::Color4ub>(Corrade::Containers::ValueInit, uRes * vRes),
        Array<Magnum::Color4ub>(Corrade::Containers::ValueInit, uRes * vRes),
        Array<Magnum::Color4ub>(Corrade::Containers::ValueInit, uRes * vRes),
        Array<Magnum::Color4ub>(Corrade::Containers::ValueInit, uRes * vRes),
        Array<Magnum::Color4ub>(Corrade::Containers::ValueInit, uRes * vRes),
        Array<Magnum::Color4ub>(Corrade::Containers::ValueInit, uRes * vRes)
    };
    std::vector<ImageData2D> cubemapOutput;
    cubemapOutput.reserve(6);
    for (size_t i = 0; i < 6; i++)
    {
        cubemapOutput.emplace_back(
            PixelFormat::RGB8UI,
            Vector2i{uRes, vRes},
            Magnum::Trade::DataFlag::Mutable,
            arrayData[i]);
    }

    constexpr std::array<std::tuple<CubeMapCoordinate, std::string_view, size_t>, 6> faces
    {
        std::make_tuple(CubeMapCoordinate::PositiveX, "posX.png", 0),
        std::make_tuple(CubeMapCoordinate::NegativeX, "negX.png", 1),
        std::make_tuple(CubeMapCoordinate::PositiveY, "posY.png", 2),
        std::make_tuple(CubeMapCoordinate::NegativeY, "negY.png", 3),
        std::make_tuple(CubeMapCoordinate::PositiveZ, "posZ.png", 4),
        std::make_tuple(CubeMapCoordinate::NegativeZ, "negZ.png", 5)
    };

    GL::CubeMapTexture outputCube;
    outputCube
        .setStorage(1, GL::TextureFormat::RGBA8UI, Vector2i{static_cast<int>(cubemapWidth)});
    for (auto const [direction, filename, index] : faces)
    {
        outputCube.setSubImage(direction, 0, {}, cubemapOutput[index]);
    }

    bind_input_map(inputTex);
    bind_output_cube(outputCube);

    constexpr Magnum::Vector2ui blockSize{8, 8};

    Magnum::Vector3ui dispatch{uRes / blockSize.x(), vRes / blockSize.y(), 6};

    dispatchCompute(dispatch);
    glMemoryBarrier(GL_BUFFER_UPDATE_BARRIER_BIT);

    for (size_t i = 0; i < 6; i++)
    {
        auto const [direction, filename, index] = faces[i];
        std::string outputFilepath = osp::string_concat(outputPath, filename);
        Image2D outputImg{PixelFormat::RGBA8UI};
        outputCube.image(direction, 0, outputImg);

        ImageView2D img(PixelFormat::RGBA8Unorm, outputImg.size(), outputImg.data());
        converter.exportToFile(img, outputFilepath);
    }
}

CubemapComputeShader& CubemapComputeShader::bind_input_map(GL::Texture2D& rTex)
{
    rTex.bindImage(static_cast<Int>(TextureSlots::InputEquirectSlot), 0,
        GL::ImageAccess::ReadOnly, GL::ImageFormat::RGBA8UI);
    return *this;
}

CubemapComputeShader& CubemapComputeShader::bind_output_cube(GL::CubeMapTexture& rTex)
{
    rTex.bindImageLayered(static_cast<Int>(TextureSlots::OutputCubemapSlot), 0,
        GL::ImageAccess::WriteOnly, GL::ImageFormat::RGBA8UI);
    return *this;
}

NormalMapGenerator()
{
    init();
}

void NormalMapGenerator::init()
{
    GL::Shader prog{GL::Version::GL430, GL::Shader::Type::Compute};
    prog.addFile("OSPData/adera/Shaders/BumpToNormal.comp");

    CORRADE_INTERNAL_ASSERT_OUTPUT(prog.compile());
    attachShader(prog);
    CORRADE_INTERNAL_ASSERT_OUTPUT(link());

    setUniform(
        static_cast<Int>(UniformPos::InputMap),
        static_cast<Int>(ImageSlots::InputMap));
    setUniform(
        static_cast<Int>(UniformPos::OutputMap),
        static_cast<Int>(ImageSlots::OutputMap));
}

void NormalMapGenerator::process(std::string_view input, std::string_view output)
{
    
}


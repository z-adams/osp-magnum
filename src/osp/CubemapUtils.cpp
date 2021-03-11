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
#include <Magnum/GL/PixelFormat.h>
#include <Magnum/GL/BufferImage.h>
#include <Magnum/GL/ImageFormat.h>
#include <Magnum/Math/Swizzle.h>
#include <osp/string_concat.h>

using namespace osp::math::cubemap;
using namespace Magnum;
using namespace Corrade;

Containers::Optional<Trade::ImageData2D> osp::math::cubemap::load_image(std::string_view filename)
{
    using Corrade::PluginManager::Manager;
    using Corrade::Containers::Optional;
    using Magnum::Trade::ImageData2D;
    using Magnum::Trade::AbstractImporter;
    using Magnum::Trade::AnyImageImporter;

    Manager<AbstractImporter> importManager;
    AnyImageImporter importer(importManager);
    if (!importer.openFile(std::string{filename}))
    {
        std::cout << "ERROR opening input file " << filename << std::endl;
        return {};
    }

    Optional<ImageData2D> image = importer.image2D(0);
    return image;
}

void osp::math::cubemap::save_image(Magnum::ImageView2D image, std::string_view filepath)
{
    using Corrade::PluginManager::Manager;
    using Magnum::Trade::AbstractImageConverter;
    using Magnum::Trade::AnyImageConverter;

    Manager<AbstractImageConverter> convertManager;
    AnyImageConverter converter(convertManager);
    converter.exportToFile(image, std::string{filepath});
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

template <GL::TextureFormat INPUT_FMT_T, PixelFormat CUBE_FMT_T, PixelFormat OUTPUT_FMT_T>
void CubemapComputeShader::process(
    std::string_view input, std::string_view outputPath, size_t outputSize)
{
    using Magnum::Trade::ImageData2D;
    using Magnum::ImageView2D;
    using Magnum::PixelFormat;
    using GL::TextureFormat;
    using GL::CubeMapCoordinate;
    using Corrade::Containers::Array;
    using Corrade::Containers::Optional;

    /*constexpr TextureFormat input_t = TextureFormat::RGBA8;
    constexpr PixelFormat cube_t = PixelFormat::RGBA8UI;
    constexpr PixelFormat output_t = PixelFormat::RGBA8Unorm;*/

    // ======== INPUT MAP ======== //
    Optional<ImageData2D> imageInput = load_image(input);

    GL::Texture2D inputTex;
    inputTex.setWrapping(SamplerWrapping::ClampToEdge)
        .setStorage(1, INPUT_FMT_T, imageInput->size())
        .setSubImage(0, {}, *imageInput);

    // ======== OUTPUT MAP ======== //
    int uRes = outputSize, vRes = outputSize;
    Vector2i faceDims{uRes, vRes};
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
    outputCube.setStorage(1, GL::textureFormat(CUBE_FMT_T), faceDims);
    for (auto const [direction, filename, index] : faces)
    {
        Magnum::UnsignedInt faceSizeBytes = uRes * vRes * pixelSize(CUBE_FMT_T);
        Array<UnsignedByte> data{Corrade::Containers::ValueInit, faceSizeBytes};
        GL::BufferImage2D bufImg(CUBE_FMT_T, faceDims,
            std::move(data), GL::BufferUsage::StaticRead);
        outputCube.setSubImage(direction, 0, {}, std::move(bufImg));
    }

    // ======== COMPUTE ======== //
    bind_input_map(inputTex);
    bind_output_cube(outputCube);

    dispatchCompute({uRes / sm_blockSize.x(), vRes / sm_blockSize.y(), 6});
    glMemoryBarrier(GL_BUFFER_UPDATE_BARRIER_BIT);

    // ======== SAVE OUTPUT ======== //
    for (size_t i = 0; i < 6; i++)
    {
        auto const [direction, filename, index] = faces[i];
        std::string outputFilepath = osp::string_concat(outputPath, filename);
        Image2D outputImg{CUBE_FMT_T};
        outputCube.image(direction, 0, outputImg);

        ImageView2D img(OUTPUT_FMT_T, outputImg.size(), outputImg.data());
        save_image(img, outputFilepath);
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

NormalMapGenerator::NormalMapGenerator()
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

void NormalMapGenerator::process(std::string_view input,
    float circumference, float oblateness, std::string_view output)
{
    using Corrade::Containers::Optional;
    using Corrade::Containers::Array;
    using Magnum::Trade::ImageData2D;

    Optional<ImageData2D> inputImage = load_image(input);

    int uRes = inputImage->size().x();
    int vRes = inputImage->size().y();

    GL::Texture2D inputTex;
    inputTex.setWrapping(SamplerWrapping::Repeat)
        .setStorage(1, GL::TextureFormat::R32F, inputImage->size())
        .setSubImage(0, {}, std::move(*inputImage));

    Array<Vector4> imageData(Corrade::Containers::ValueInit, uRes*vRes);
    ImageData2D outputImgData(PixelFormat::RGBA32F, inputImage->size(),
        Magnum::Trade::DataFlag::Mutable, std::move(imageData));

    GL::Texture2D outputTex;
    outputTex.setWrapping(SamplerWrapping::Repeat)
        .setStorage(1, GL::TextureFormat::RGBA32F, inputImage->size())
        .setSubImage(0, {}, std::move(outputImgData));

    bind_input_map(inputTex);
    bind_output_map(outputTex);
    set_circumference(circumference);
    set_oblateness(oblateness);

    constexpr Magnum::Vector2ui blockSize{8, 8};

    Magnum::Vector3ui dispatch{uRes / blockSize.x(), vRes / blockSize.y(), 1};

    dispatchCompute(dispatch);
    glMemoryBarrier(GL_BUFFER_UPDATE_BARRIER_BIT);

    Image2D outputImg{PixelFormat::RGBA32F};
    outputTex.image(0, outputImg);

    save_image(outputImg, output);
}

NormalMapGenerator& NormalMapGenerator::bind_input_map(Magnum::GL::Texture2D& rTex)
{
    rTex.bindImage(static_cast<Int>(ImageSlots::InputMap), 0,
        GL::ImageAccess::ReadOnly, GL::ImageFormat::R32F);
    return *this;
}

NormalMapGenerator& NormalMapGenerator::bind_output_map(Magnum::GL::Texture2D& rTex)
{
    rTex.bindImage(static_cast<Int>(ImageSlots::OutputMap), 0,
        GL::ImageAccess::WriteOnly, GL::ImageFormat::RGBA32F);
    return *this;
}

NormalMapGenerator& NormalMapGenerator::set_circumference(float circ)
{
    setUniform(static_cast<Int>(UniformPos::Circumference), circ);
    return *this;
}

NormalMapGenerator& NormalMapGenerator::set_oblateness(float obl)
{
    setUniform(static_cast<Int>(UniformPos::Oblateness), obl);
    return *this;
}

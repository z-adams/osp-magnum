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
    Magnum::PixelFormat inputFmt = Magnum::PixelFormat::R16UI;

    int uRes = inputImage->size().x();
    int vRes = inputImage->size().y();

    GL::Texture2D inputTex;
    inputTex.setWrapping(SamplerWrapping::Repeat)
        .setStorage(1, GL::textureFormat(inputFmt), inputImage->size())
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
        GL::ImageAccess::ReadOnly, GL::ImageFormat::R16UI);
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
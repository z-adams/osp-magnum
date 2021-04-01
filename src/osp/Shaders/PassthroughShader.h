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

#include <Magnum/GL/AbstractShaderProgram.h>
#include <Magnum/GL/Texture.h>
#include <Magnum/GL/ImageFormat.h>
#include <assert.h>

namespace osp::active::shader
{

class PassthroughShader : public Magnum::GL::AbstractShaderProgram
{
public:
    PassthroughShader() { init(); }

    template <Magnum::GL::ImageFormat IMG_FORMAT_T = Magnum::GL::ImageFormat::RGBA8>
    void copy_textures(Magnum::GL::Texture2D& source, Magnum::GL::Texture2D& dest);

private:
    void init();

    // Uniforms
    enum class UniformPos : Magnum::Int
    {
        sourceSampler = 0,
        destSampler = 1
    };

    // Image Slots
    enum class ImagePos : Magnum::Int
    {
        sourceImg = 0,
        destImg = 1
    };

    using Magnum::GL::AbstractShaderProgram::dispatchCompute;
    using Magnum::GL::AbstractShaderProgram::drawTransformFeedback;
    using Magnum::GL::AbstractShaderProgram::draw;

    template <Magnum::GL::ImageFormat IMG_FORMAT_T=Magnum::GL::ImageFormat::RGBA8>
    void bind_source(Magnum::GL::Texture2D& source);

    template <Magnum::GL::ImageFormat IMG_FORMAT_T = Magnum::GL::ImageFormat::RGBA8>
    void bind_dest(Magnum::GL::Texture2D& dest);
};

template<Magnum::GL::ImageFormat IMG_FORMAT_T>
void PassthroughShader::copy_textures(Magnum::GL::Texture2D& source, Magnum::GL::Texture2D& dest)
{
    bind_source<IMG_FORMAT_T>(source);
    bind_dest<IMG_FORMAT_T>(dest);

    Magnum::Vector2i srcDims = source.imageSize(0);
    Magnum::Vector2i dstDims = dest.imageSize(0);

    assert(srcDims == dstDims);

    Magnum::Vector3ui blocks = {Magnum::Vector2ui{srcDims} / 8, 1};
    dispatchCompute(blocks);
    glMemoryBarrier(GL_TEXTURE_FETCH_BARRIER_BIT);
}

template<Magnum::GL::ImageFormat IMG_FORMAT_T>
void PassthroughShader::bind_source(Magnum::GL::Texture2D& source)
{
    source.bindImage(static_cast<Magnum::Int>(ImagePos::sourceImg), 0, Magnum::GL::ImageAccess::ReadOnly, IMG_FORMAT_T);
}

template<Magnum::GL::ImageFormat IMG_FORMAT_T>
void PassthroughShader::bind_dest(Magnum::GL::Texture2D& dest)
{
    dest.bindImage(static_cast<Magnum::Int>(ImagePos::destImg), 0,
        Magnum::GL::ImageAccess::WriteOnly, IMG_FORMAT_T);
}

}

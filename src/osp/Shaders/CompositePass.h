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

namespace osp::active::shader
{

class CompositePass : Magnum::GL::AbstractShaderProgram
{
public:
    CompositePass() { init(); }
    CompositePass(CompositePass const& copy) = delete;
    ~CompositePass() = default;

    // TODO mention uses Image Load/Store
    void compose_image(
        Magnum::GL::Texture2D& diffusePass,
        Magnum::GL::Texture2D& lightPass,
        Magnum::GL::Texture2D& outputTarget);

private:
    void init();

    // Uniforms
    enum class UniformPos : Magnum::Int
    {
        diffuseSampler = 0,
        lightingSampler = 1,
        outputSampler = 2
    };

    // Samplers
    enum class TexturePos : Magnum::Int
    {
        diffuseTex = 0,
        lightingTex = 1
    };

    // Images
    enum class ImagePos : Magnum::Int
    {
        outputImage = 0
    };

    // Make dispatch calls private
    using Magnum::GL::AbstractShaderProgram::draw;
    using Magnum::GL::AbstractShaderProgram::drawTransformFeedback;
    // Only this one is actually used since it's a compute shader
    using Magnum::GL::AbstractShaderProgram::dispatchCompute;

    // Private uniform setters
    CompositePass& bind_diffuse_tex(Magnum::GL::Texture2D& texture);
    CompositePass& bind_lighting_tex(Magnum::GL::Texture2D& texture);
    CompositePass& bind_output_image(Magnum::GL::Texture2D& target);
};

}

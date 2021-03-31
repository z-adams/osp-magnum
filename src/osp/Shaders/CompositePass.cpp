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
#include "CompositePass.h"

#include <Corrade/Containers/Reference.h>
#include <Magnum/GL/Shader.h>
#include <Magnum/GL/Version.h>
#include <Magnum/GL/ImageFormat.h>

using namespace Magnum;
using namespace osp::active::shader;
using namespace osp::active;


void CompositePass::compose_image(GL::Texture2D& diffusePass, GL::Texture2D& lightPass,
    GL::Texture2D& outputTarget)
{
    bind_diffuse_tex(diffusePass);
    bind_lighting_tex(lightPass);
    bind_output_image(outputTarget);

    constexpr Vector3ui dimensions{1280/8, 720/8, 1};
    dispatchCompute(dimensions);
    glMemoryBarrier(GL_TEXTURE_FETCH_BARRIER_BIT);
}

void CompositePass::init()
{
    GL::Shader prog{GL::Version::GL430, GL::Shader::Type::Compute};
    prog.addFile("OSPData/adera/Shaders/CompositePass.comp");

    CORRADE_INTERNAL_ASSERT_OUTPUT(prog.compile());
    attachShader(prog);
    CORRADE_INTERNAL_ASSERT_OUTPUT(link());

    // Set sampler2D uniforms
    setUniform(
        static_cast<Int>(UniformPos::diffuseSampler),
        static_cast<Int>(TexturePos::diffuseTex));
    setUniform(
        static_cast<Int>(UniformPos::lightingSampler),
        static_cast<Int>(TexturePos::lightingTex));
    setUniform(
        static_cast<Int>(UniformPos::outputSampler),
        static_cast<Int>(ImagePos::outputImage));
}

CompositePass& CompositePass::bind_diffuse_tex(GL::Texture2D& texture)
{
    texture.bind(static_cast<Int>(TexturePos::diffuseTex));
    return *this;
}

CompositePass& CompositePass::bind_lighting_tex(GL::Texture2D& texture)
{
    texture.bind(static_cast<Int>(TexturePos::lightingTex));
    return *this;
}

CompositePass& CompositePass::bind_output_image(GL::Texture2D& target)
{
    target.bindImage(static_cast<Int>(ImagePos::outputImage),
        0, GL::ImageAccess::WriteOnly, GL::ImageFormat::RGBA8);
    return *this;
}



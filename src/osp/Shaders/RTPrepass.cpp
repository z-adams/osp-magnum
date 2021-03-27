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
#include "RTPrepass.h"

#include <Magnum/GL/Shader.h>
#include <Magnum/GL/Version.h>
#include <Corrade/Containers/Reference.h>

using namespace Magnum;
using namespace osp::active::shader;
using namespace osp::active;

void PrepassShader::init()
{
    GL::Shader vert{GL::Version::GL430, GL::Shader::Type::Vertex};
    GL::Shader frag{GL::Version::GL430, GL::Shader::Type::Fragment};
    vert.addFile("OSPData/adera/Shaders/DepthPrepass.vert");
    frag.addFile("OSPData/adera/Shaders/DepthPrepass.frag");

    CORRADE_INTERNAL_ASSERT_OUTPUT(GL::Shader::compile({vert, frag}));
    attachShaders({vert, frag});
    CORRADE_INTERNAL_ASSERT_OUTPUT(link());
}

PrepassShader& PrepassShader::set_transform_matrix(Matrix4 const& matrix)
{
    setUniform(static_cast<Int>(UniformPos::modelViewMatrix), matrix);
    return *this;
}

PrepassShader& PrepassShader::set_proj_matrix(Matrix4 const& matrix)
{
    setUniform(static_cast<Int>(UniformPos::projMatrix), matrix);
    return *this;
}

PrepassShader& PrepassShader::set_camera_pos_world(Vector3 const& pos)
{
    setUniform(static_cast<Int>(UniformPos::cameraPos), pos);
    return *this;
}

PrepassShader& PrepassShader::bind_gbuffer(GL::Framebuffer& buffer)
{
    buffer.mapForDraw({
        {static_cast<Int>(Outputs::gCastRay_Depth), GL::Framebuffer::ColorAttachment{0}},
        {static_cast<Int>(Outputs::gNormalXY_HitUV), GL::Framebuffer::ColorAttachment{1}},
        });
    return *this;
}

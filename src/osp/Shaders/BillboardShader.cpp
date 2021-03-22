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
#include "BillboardShader.h"

#include <Corrade/Containers/Reference.h>
#include <Magnum/GL/Version.h>
#include <Magnum/GL/Shader.h>

using namespace osp::active::shader;
using namespace Magnum;

void BillboardShader::draw_billboard(ActiveEnt e, ActiveScene& rScene,
    GL::Mesh& rMesh, ACompCamera const& camera, ACompTransform const& transform)
{
    auto& shaderInstance = rScene.reg_get<ACompBillboardShaderInstance>(e);
    BillboardShader& shader = *shaderInstance.m_shaderProgram;

    //Vector3 pos = transform.m_transformWorld.translation();

    Vector3 pos = {100.0, 0.0, 0.0};

    shader
        .bind_texture(*shaderInstance.m_texture)
        .set_transform_matrix(Matrix4{Matrix3{camera.m_inverse}})
        .set_proj_matrix(camera.m_projection)
        .set_position_world(pos)
        .draw(rMesh);
}

void BillboardShader::init()
{
    GL::Shader vert{GL::Version::GL430, GL::Shader::Type::Vertex};
    GL::Shader frag{GL::Version::GL430, GL::Shader::Type::Fragment};
    vert.addFile("OSPData/adera/Shaders/BillboardShader.vert");
    frag.addFile("OSPData/adera/Shaders/BillboardShader.frag");

    CORRADE_INTERNAL_ASSERT_OUTPUT(GL::Shader::compile({vert, frag}));
    attachShaders({vert, frag});
    CORRADE_INTERNAL_ASSERT_OUTPUT(link());

    // Set Sampler2D uniform position
    setUniform(
        static_cast<Int>(UniformPos::Texture),
        static_cast<Int>(TextureSlots::BillboardTexture));
}

BillboardShader& BillboardShader::set_proj_matrix(Magnum::Matrix4 const& matrix)
{
    setUniform(static_cast<Int>(UniformPos::ProjMat), matrix);
    return *this;
}

BillboardShader& BillboardShader::set_transform_matrix(Magnum::Matrix4 const& matrix)
{
    setUniform(static_cast<Int>(UniformPos::ViewTransformMat), matrix);
    return *this;
}

BillboardShader& BillboardShader::set_position_world(Magnum::Vector3 const& matrix)
{
    setUniform(static_cast<Int>(UniformPos::WorldPos), matrix);
    return *this;
}

BillboardShader& BillboardShader::bind_texture(Magnum::GL::Texture2D& rTex)
{
    rTex.bind(static_cast<Int>(TextureSlots::BillboardTexture));
    return *this;
}

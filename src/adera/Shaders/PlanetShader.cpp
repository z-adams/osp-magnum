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
#include "PlanetShader.h"

#include <Magnum/GL/Version.h>
#include <Magnum/GL/Shader.h>
#include <Corrade/Containers/Reference.h>
#include <Magnum/GL/Texture.h>

using adera::shader::PlanetShader;
using namespace osp::active;
using Magnum::Int;

void PlanetShader::draw_planet(ActiveEnt e, ActiveScene& rScene,
    Magnum::GL::Mesh& rMesh, ACompCamera const& camera, ACompTransform const& transform)
{
    auto& shaderInstance = rScene.reg_get<ACompPlanetShaderInstance>(e);
    PlanetShader& shader = *shaderInstance.m_shaderProgram;

    Magnum::Matrix4 entRelative = camera.m_inverse * transform.m_transformWorld;

    shader
        .bind_diffuse_cubemap(*shaderInstance.m_diffuseTex)
        .set_transform_matrix(entRelative)
        .set_proj_matrix(camera.m_projection)
        .set_normal_matrix(entRelative.normalMatrix())
        .draw(rMesh);
}

void PlanetShader::init()
{
    using namespace Magnum;

    GL::Shader vert{GL::Version::GL430, GL::Shader::Type::Vertex};
    GL::Shader frag{GL::Version::GL430, GL::Shader::Type::Fragment};
    vert.addFile("OSPData/adera/Shaders/PlanetShader.vert");
    frag.addFile("OSPData/adera/Shaders/PlanetShader.frag");

    CORRADE_INTERNAL_ASSERT_OUTPUT(GL::Shader::compile({vert, frag}));
    attachShaders({vert, frag});
    CORRADE_INTERNAL_ASSERT_OUTPUT(link());

    // Set TexSampler2D uniforms
    setUniform(
        static_cast<Int>(UniformPos::DiffuseTex),
        static_cast<Int>(TextureSlots::DiffuseCMUnit));
}

PlanetShader::PlanetShader()
{
    init();
}

PlanetShader& PlanetShader::set_proj_matrix(Magnum::Matrix4 const& matrix)
{
    setUniform(static_cast<Int>(UniformPos::ProjMat), matrix);
    return *this;
}

PlanetShader& PlanetShader::set_transform_matrix(Magnum::Matrix4 const& matrix)
{
    setUniform(static_cast<Int>(UniformPos::ModelTransformMat), matrix);
    return *this;
}

PlanetShader& PlanetShader::set_normal_matrix(Magnum::Matrix3 const& matrix)
{
    setUniform(static_cast<Int>(UniformPos::NormalMat), matrix);
    return *this;
}

PlanetShader& PlanetShader::bind_diffuse_cubemap(Magnum::GL::CubeMapTexture & rTex)
{
    rTex.bind(static_cast<Int>(TextureSlots::DiffuseCMUnit));
    return *this;
}

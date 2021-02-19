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
#include <Magnum/GL/Attribute.h>
#include <Magnum/GL/Texture.h>
#include <Magnum/GL/CubeMapTexture.h>
#include <Magnum/Math/Vector4.h>
#include <Magnum/Math/Vector3.h>
#include <Magnum/Math/Vector2.h>
#include <Magnum/Math/Matrix3.h>
#include <Magnum/Math/Matrix4.h>
#include <Magnum/Math/Color.h>
#include <Magnum/Shaders/Generic.h>

#include <osp/Resource/Resource.h>
#include <osp/Active/activetypes.h>
#include <osp/Active/ActiveScene.h>

namespace adera::shader
{

class PlanetShader : public Magnum::GL::AbstractShaderProgram
{
public:
    // Vertex attribs
    Magnum::Shaders::Generic3D::Position Position;
    Magnum::Shaders::Generic3D::Normal Normal;
    Magnum::Shaders::Generic3D::TextureCoordinates TextureCoordinates;

    // Outputs
    enum : Magnum::UnsignedInt
    {
        ColorOutput = 0
    };

    PlanetShader();

    struct ACompPlanetShaderInstance
    {
        // Parent shader
        osp::DependRes<PlanetShader> m_shaderProgram;

        // Uniform values
        osp::DependRes<Magnum::GL::CubeMapTexture> m_diffuseTex;

        ACompPlanetShaderInstance(
            osp::DependRes<PlanetShader> parent,
            osp::DependRes<Magnum::GL::CubeMapTexture> texture)
            : m_shaderProgram(parent)
            , m_diffuseTex(texture)
        {}
    };

    static void draw_planet(osp::active::ActiveEnt e,
        osp::active::ActiveScene& rScene,
        Magnum::GL::Mesh& rMesh,
        osp::active::ACompCamera const& camera,
        osp::active::ACompTransform const& transform);

private:
    // GL init
    void init();

    // Uniforms
    enum class UniformPos : Magnum::Int
    {
        ProjMat = 0,
        ModelTransformMat = 1,
        NormalMat = 2,
        DiffuseTex = 3
    };

    // Cubemap slots
    enum class TextureSlots : Magnum::Int
    {
        DiffuseCMUnit = 0
    };

    // Hide irrelevant calls
    using Magnum::GL::AbstractShaderProgram::drawTransformFeedback;
    using Magnum::GL::AbstractShaderProgram::dispatchCompute;

    // Private uniform setters
    PlanetShader& set_proj_matrix(Magnum::Matrix4 const& matrix);
    PlanetShader& set_transform_matrix(Magnum::Matrix4 const& matrix);
    PlanetShader& set_normal_matrix(Magnum::Matrix3 const& matrix);

    PlanetShader& bind_diffuse_cubemap(Magnum::GL::CubeMapTexture& rTex);
};

} // namespace adera::shader
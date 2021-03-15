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
#include <Magnum/GL/CubeMapTexture.h>
#include <Magnum/GL/Attribute.h>

#include "../types.h"
#include <osp/Resource/Resource.h>
#include <osp/Active/ActiveScene.h>

class SkyboxShader : public Magnum::GL::AbstractShaderProgram
{
public:
    typedef Magnum::GL::Attribute<0, Magnum::Vector3> Position;
    typedef Magnum::GL::Attribute<1, Magnum::Vector3> Normal;

    SkyboxShader() { init(); }

    struct ACompSkyboxShaderInstance
    {
        // Parent shader
        osp::DependRes<SkyboxShader> m_shaderProgram;

        // Uniform values
        osp::DependRes<Magnum::GL::CubeMapTexture> m_texture;

        ACompSkyboxShaderInstance(
            osp::DependRes<SkyboxShader> parent,
            osp::DependRes<Magnum::GL::CubeMapTexture> texture)
            : m_shaderProgram(parent)
            , m_texture(texture)
        {}
    };

    static void draw_sky(osp::active::ActiveEnt e,
        osp::active::ActiveScene& rScene,
        Magnum::GL::Mesh& rMesh,
        osp::active::ACompCamera const& camera,
        osp::active::ACompTransform const& transform);

private:
    void init();

    // Uniforms
    enum class UniformPos : Magnum::Int
    {
        ProjMat = 0,
        ModelTransformMat = 1,
        NormalMat = 2,
        Texture = 3
    };

    // Texture slots
    enum class TextureSlots : Magnum::Int
    {
        TextureCM = 0
    };

    // Hide irrelevant calls
    using Magnum::GL::AbstractShaderProgram::drawTransformFeedback;
    using Magnum::GL::AbstractShaderProgram::dispatchCompute;

    // Private uniform setters
    SkyboxShader& set_proj_matrix(Magnum::Matrix4 const& matrix);
    SkyboxShader& set_transform_matrix(Magnum::Matrix4 const& matrix);
    SkyboxShader& set_normal_matrix(Magnum::Matrix3 const& matrix);

    SkyboxShader& bind_diffuse_cubemap(Magnum::GL::CubeMapTexture& rTex);
};

namespace osp::active
{

class SysSkybox : public IDynamicSystem
{
public:
    static inline std::string smc_name = "Skybox";

    SysSkybox(ActiveScene &rScene);
    SysSkybox(SysSkybox const& copy) = delete;
    SysSkybox(SysSkybox&& move) = delete;
    ~SysSkybox() = default;

    static void update(ActiveScene& rScene);
    static void set_skybox(ActiveScene &rScene, std::string_view skyboxResName);

private:
    UpdateOrderHandle_t m_updateOrder;
};

}

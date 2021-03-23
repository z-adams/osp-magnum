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
#include "SysSkybox.h"

#include <Magnum/GL/Buffer.h>
#include <Magnum/GL/Mesh.h>
#include <Magnum/GL/Shader.h>
#include <Magnum/GL/Attribute.h>
#include <Magnum/GL/Version.h>
#include <Magnum/Trade/MeshData.h>
#include <Magnum/MeshTools/Interleave.h>
#include <Magnum/MeshTools/CompressIndices.h>
#include <Magnum/Primitives/Cube.h>
#include <Corrade/Containers/Reference.h>
#include <Corrade/Containers/ArrayViewStl.h>

#include <osp/Resource/Cubemap.h>
#include <osp/Resource/AssetImporter.h>
#include <osp/Active/SysDebugRender.h>

#include <adera/Machines/UserControl.h>

using namespace osp::active;
using namespace Magnum;

void SkyboxShader::draw_sky(ActiveEnt e, ActiveScene& rScene, GL::Mesh& rMesh,
    ACompCamera const& camera, ACompTransform const& transform)
{
    auto& shaderInstance = rScene.reg_get<ACompSkyboxShaderInstance>(e);
    SkyboxShader& shader = *shaderInstance.m_shaderProgram;

    Magnum::Matrix4 rotation = Matrix4{Matrix3{camera.m_inverse}};
    Magnum::Matrix4 entRelative = rotation;

    shader
        .bind_diffuse_cubemap(*shaderInstance.m_texture)
        .set_transform_matrix(entRelative)
        .set_proj_matrix(camera.m_projection)
        .set_normal_matrix(entRelative.normalMatrix())
        .draw(rMesh);
}

void SkyboxShader::init()
{
    GL::Shader vert{GL::Version::GL430, GL::Shader::Type::Vertex};
    GL::Shader frag{GL::Version::GL430, GL::Shader::Type::Fragment};
    vert.addFile("OSPData/adera/Shaders/SkyboxShader.vert");
    frag.addFile("OSPData/adera/Shaders/SkyboxShader.frag");

    CORRADE_INTERNAL_ASSERT_OUTPUT(GL::Shader::compile({vert, frag}));
    attachShaders({vert, frag});
    CORRADE_INTERNAL_ASSERT_OUTPUT(link());

    // Set TexSampler2D uniforms
    setUniform(
        static_cast<Int>(UniformPos::Texture),
        static_cast<Int>(TextureSlots::TextureCM));
}

SkyboxShader& SkyboxShader::set_proj_matrix(Matrix4 const & matrix)
{
    setUniform(static_cast<Int>(UniformPos::ProjMat), matrix);
    return *this;
}

SkyboxShader& SkyboxShader::set_transform_matrix(Matrix4 const & matrix)
{
    setUniform(static_cast<Int>(UniformPos::ModelTransformMat), matrix);
    return *this;
}

SkyboxShader& SkyboxShader::set_normal_matrix(Matrix3 const & matrix)
{
    setUniform(static_cast<Int>(UniformPos::NormalMat), matrix);
    return *this;
}

SkyboxShader& SkyboxShader::bind_diffuse_cubemap(GL::CubeMapTexture& rTex)
{
    rTex.bind(static_cast<Int>(TextureSlots::TextureCM));
    return *this;
}

SysSkybox::SysSkybox(ActiveScene& rScene)
    : m_updateOrder(rScene.get_update_order(), "skybox", "", "", SysSkybox::update)
{
    using namespace Magnum;

    Package& glResources = rScene.get_context_resources();

    DependRes<GL::Mesh> boxMesh = glResources.get<GL::Mesh>("skybox_mesh");
    if (boxMesh.empty())
    {
        /*std::array<Vector3, 8> vertices =
        {
            Vector3{-1.0, -1.0, -1.0},
            Vector3{1.0, -1.0, -1.0},
            Vector3{1.0, 1.0, -1.0},
            Vector3{-1.0, 1.0, -1.0},
            Vector3{-1.0, -1.0, 1.0},
            Vector3{1.0, -1.0, 1.0},
            Vector3{1.0, 1.0, 1.0},
            Vector3{-1.0, 1.0, 1.0},
        };

        GL::Buffer vertBuffer;
        vertBuffer.setData(std::move(vertices));

        std::array<UnsignedByte, 36> indices =
        {
            0, 1, 2,
            2, 3, 0,

            4, 5, 6,
            6, 7, 4,

            0, 5, 4,
            0, 1, 5,

            1, 6, 5,
            1, 2, 6,

            2, 3, 6,
            2, 3, 7,

            3, 0, 4,
            3, 4, 7
        };

        GL::Buffer indexBuffer;
        indexBuffer.setData(std::move(indices));

        Magnum::GL::Mesh mesh;
        mesh.setPrimitive(Magnum::MeshPrimitive::Triangles)
            .setCount(36)
            .addVertexBuffer(std::move(vertBuffer), 0,
                SkyboxShader::Position{})
            .setIndexBuffer(std::move(indexBuffer), 0,
                Magnum::MeshIndexType::UnsignedByte);*/

        Magnum::Trade::MeshData cube = Magnum::Primitives::cubeSolid();
        GL::Buffer vertices;
        vertices.setData(MeshTools::interleave(cube.positions3DAsArray(),
            cube.normalsAsArray()));
        std::pair<Containers::Array<char>, MeshIndexType> compressed =
            MeshTools::compressIndices(cube.indicesAsArray());
        GL::Buffer indices;
        indices.setData(compressed.first);

        Magnum::GL::Mesh mesh;
        mesh.setPrimitive(cube.primitive())
            .setCount(cube.indexCount())
            .addVertexBuffer(std::move(vertices), 0, SkyboxShader::Position{},
                SkyboxShader::Normal{})
            .setIndexBuffer(std::move(indices), 0, compressed.second);

        boxMesh = glResources.add<GL::Mesh>("skybox_mesh", std::move(mesh));
    }
}

void SysSkybox::set_skybox(ActiveScene& rScene, std::string_view skyboxResName)
{
    using ShaderInstance_t = SkyboxShader::ACompSkyboxShaderInstance;
    auto view = rScene.get_registry().view<ShaderInstance_t>();

    Package& glResources = rScene.get_context_resources();
    DependRes<Magnum::GL::CubeMapTexture> res =
        glResources.get<Magnum::GL::CubeMapTexture>(skyboxResName);

    if (res.empty())
    {
        Package& lzdb = rScene.get_application().debug_find_package("lzdb");
        res = osp::AssetImporter::compile_cubemap(skyboxResName, lzdb, glResources);
    }

    if (view.empty())
    {
        ActiveEnt boxEnt = rScene.hier_create_child(rScene.hier_get_root(), "Skybox");
        DependRes<SkyboxShader> shader = glResources.get<SkyboxShader>("skybox_shader");
        rScene.reg_emplace<ShaderInstance_t>(boxEnt, shader, std::move(res));
        DependRes<GL::Mesh> mesh = glResources.get<GL::Mesh>("skybox_mesh");
        rScene.reg_emplace<CompDrawableDebug>(boxEnt, mesh, SkyboxShader::draw_sky);
        rScene.reg_emplace<ACompTransform>(boxEnt);
        rScene.reg_emplace<CompBackgroundDebug>(boxEnt);
    }
    else
    {
        ActiveEnt sbEnt = *view.begin();
        ShaderInstance_t& skyInstance = view.get<ShaderInstance_t>(sbEnt);
        skyInstance.m_texture = std::move(res);
    }
}

void SysSkybox::update(ActiveScene& rScene)
{

}

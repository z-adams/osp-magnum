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
#include <Magnum/GL/DefaultFramebuffer.h>
#include <Magnum/GL/Renderer.h>
#include <Magnum/GL/Buffer.h>
#include <Magnum/GL/Mesh.h>
#include <Magnum/Mesh.h>
#include <Magnum/GL/DebugOutput.h>
#include <Corrade/Containers/ArrayViewStl.h>

#include "SysDebugRender.h"
#include "ActiveScene.h"
#include "adera/Shaders/Phong.h"
#include "adera/Shaders/PlumeShader.h"
#include "adera/Shaders/PlanetShader.h"
#include <osp/Active/SysSkybox.h>
#include <osp/Shaders/BillboardShader.h>
#include <Magnum/Shaders/VertexColor.h>


using namespace osp::active;

// for _1, _2, _3, ... std::bind arguments
using namespace std::placeholders;

// for the 0xrrggbb_rgbf and _deg literals
using namespace Magnum::Math::Literals;

const std::string SysDebugRender::smc_name = "DebugRender";

SysDebugRender::SysDebugRender(ActiveScene &rScene) :
        m_scene(rScene),
        m_renderDebugDraw(rScene.get_render_order(), "debug", "", "",
                          std::bind(&SysDebugRender::draw, this, _1))
{
    Package& glResources = m_scene.get_context_resources();

    using namespace adera::shader;

    glResources.add<Phong>("phong_shader",
        Phong{Magnum::Shaders::Phong::Flag::DiffuseTexture});

    glResources.add<PlumeShader>("plume_shader");

    glResources.add<PlanetShader>("planet_shader");

    glResources.add<SkyboxShader>("skybox_shader");

    // TMP: no shaderinstance
    glResources.add<Magnum::Shaders::VertexColor3D>("vertexcolor_shader");

    glResources.add<osp::active::shader::BillboardShader>("billboard_shader");
    /*using namespace Magnum;
    GL::Renderer::enable(GL::Renderer::Feature::DebugOutput);
    GL::Renderer::enable(GL::Renderer::Feature::DebugOutputSynchronous);
    GL::DebugOutput::setDefaultCallback();*/

    // Generate 1x1 quad for billboard rendering
    using namespace Magnum;
    if (glResources.get<GL::Mesh>("billboard_quad").empty())
    {
        using namespace osp::active::shader;
        std::array<float, 30> surfData
        {
            // Vert position        // UV coordinate
            -0.5f, -0.5f, 0.0f,     0.0f,  0.0f,
             0.5f, -0.5f, 0.0f,     0.0f,  1.0f,
             0.5f,  0.5f, 0.0f,     1.0f,  1.0f,

            -0.5f, -0.5f, 0.0f,     0.0f,  0.0f,
             0.5f,  0.5f, 0.0f,     1.0f,  1.0f,
            -0.5f,  0.5f, 0.0f,     0.0f,  1.0f
        };

        GL::Buffer surface(std::move(surfData), GL::BufferUsage::StaticDraw);
        GL::Mesh surfaceMesh;
        surfaceMesh
            .setPrimitive(Magnum::MeshPrimitive::Triangles)
            .setCount(6)
            .addVertexBuffer(std::move(surface), 0,
                BillboardShader::Position{}, BillboardShader::TextureCoordinate{});
        glResources.add<GL::Mesh>("billboard_quad", std::move(surfaceMesh));
    }

    // Generate single-vertex mesh for point rendering
    using namespace Magnum;
    if (glResources.get<GL::Mesh>("point").empty())
    {
        std::array<float, 3> point
        {
            0.0f, 0.0f, 0.0f
        };

        GL::Buffer buf(std::move(point), GL::BufferUsage::StaticDraw);
        GL::Mesh pointMesh;
        pointMesh
            .setPrimitive(Magnum::MeshPrimitive::Points)
            .setCount(1)
            .addVertexBuffer(std::move(buf), 0, GL::Attribute<0, Vector3>{});
        glResources.add<GL::Mesh>("point", std::move(pointMesh));
    }
}

void SysDebugRender::draw(ACompCamera const& camera)
{
    using Magnum::GL::Renderer;

    auto& reg = m_scene.get_registry();

    Renderer::enable(Renderer::Feature::SeamlessCubeMapTexture);

    // Get skybox
    using Skybox_t = SkyboxShader::ACompSkyboxShaderInstance;
    auto boxview = reg.view<CompDrawableDebug, ACompTransform, Skybox_t>();
    // Disable depth test
    Renderer::disable(Renderer::Feature::Blending);
    Renderer::disable(Renderer::Feature::DepthTest);
    //Renderer::enable(Renderer::Feature::DepthClamp);
    Renderer::enable(Renderer::Feature::FaceCulling);
    Renderer::setFaceCullingMode(Renderer::PolygonFacing::Front);
    // Draw skybox
    draw_group(boxview, camera);

    Renderer::enable(Renderer::Feature::DepthTest);
    Renderer::disable(Renderer::Feature::DepthClamp);
    Renderer::enable(Renderer::Feature::FaceCulling);
    Renderer::setFaceCullingMode(Renderer::PolygonFacing::Back);

    // Get opaque objects
    auto opaqueObjects = reg.view<CompDrawableDebug, ACompTransform>(
        entt::exclude<CompTransparentDebug, Skybox_t, CompBackgroundDebug>);
    // Configure blend mode for opaque rendering
    Renderer::disable(Renderer::Feature::Blending);
    // Draw opaque objects
    draw_group(opaqueObjects, camera);

    // Render sun test point
    auto backgroundView =
        reg.view<CompDrawableDebug, ACompTransform, CompBackgroundDebug, CompQueryObj>();
    for (ActiveEnt e : backgroundView)
    {
        auto& q = backgroundView.get<CompQueryObj>(e);
        glBeginQuery(GL_ANY_SAMPLES_PASSED, q.m_id);
    }
    draw_group(backgroundView, camera);
    glEndQuery(GL_ANY_SAMPLES_PASSED);


    // Get transparent objects
    auto transparentObjects = m_scene.get_registry()
        .view<CompDrawableDebug, CompVisibleDebug,
        CompTransparentDebug, ACompTransform>(entt::exclude<CompOverlayDebug>);

    // Configure blend mode for transparency
    Renderer::enable(Renderer::Feature::Blending);
    Renderer::setBlendFunction(
        Renderer::BlendFunction::SourceAlpha,
        Renderer::BlendFunction::OneMinusSourceAlpha);

    // Draw backfaces first
    Renderer::setFaceCullingMode(Renderer::PolygonFacing::Front);
    draw_group(transparentObjects, camera);

    // Then draw frontfaces
    Renderer::setFaceCullingMode(Renderer::PolygonFacing::Back);
    draw_group(transparentObjects, camera);

    // Get overlays
    auto overlayObjects = m_scene.get_registry()
        .view<CompDrawableDebug, CompVisibleDebug,
        CompTransparentDebug, ACompTransform, CompOverlayDebug>();

    Magnum::GL::defaultFramebuffer.clear(Magnum::GL::FramebufferClear::Depth);
    draw_group(overlayObjects, camera);
}


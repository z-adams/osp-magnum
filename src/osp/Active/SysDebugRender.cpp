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

#include <array>

#include <Magnum/GL/DefaultFramebuffer.h>
#include <Magnum/GL/Framebuffer.h>
#include <Magnum/GL/Renderer.h>
#include <Magnum/GL/Buffer.h>
#include <Magnum/GL/Mesh.h>
#include <Magnum/GL/TextureFormat.h>
#include <Magnum/GL/Renderbuffer.h>
#include <Magnum/GL/RenderbufferFormat.h>
#include <Magnum/Mesh.h>
#include <Magnum/GL/DebugOutput.h>
#include <Magnum/Shaders/MeshVisualizer.h>
#include <Magnum/Mesh.h>
#include <Corrade/Containers/ArrayViewStl.h>

#include "SysDebugRender.h"
#include "ActiveScene.h"
#include "adera/Shaders/Phong.h"
#include <osp/Shaders/Flat.h>
#include "adera/Shaders/PlumeShader.h"
#include "adera/Shaders/PlanetShader.h"
#include <osp/Active/SysSkybox.h>
#include <osp/Shaders/BillboardShader.h>
#include <osp/Shaders/RTPrepass.h>
#include <osp/Shaders/RT.h>
#include <osp/Shaders/CompositePass.h>
#include <osp/Shaders/PassthroughShader.h>
#include <Magnum/Shaders/VertexColor.h>
#include <osp/string_concat.h>

#include <osp/Shaders/RenderTexture.h>


using namespace Magnum;
using namespace osp;
using namespace osp::active;

// for _1, _2, _3, ... std::bind arguments
using namespace std::placeholders;

// for the 0xrrggbb_rgbf and _deg literals
using namespace Magnum::Math::Literals;
using namespace Magnum;


void SysDebugRender::add_functions(ActiveScene &rScene)
{
    /*rScene.debug_render_add(rScene.get_render_order(), "debug", "", "",
                            &SysDebugRender::draw);*/

                            // Initialize some GL resources (temporary)

    Package& glResources = rScene.get_context_resources();

    using namespace adera::shader;
    using Magnum::Shaders::MeshVisualizer3D;

    glResources.add<MeshVisualizer3D>("mesh_vis_shader",
        MeshVisualizer3D{MeshVisualizer3D::Flag::Wireframe | MeshVisualizer3D::Flag::NormalDirection});

    glResources.add<Phong>("phong_shader",
        Phong{Shaders::Phong::Flag::DiffuseTexture});

    glResources.add<osp::active::shader::Flat>("flat_shader",
        osp::active::shader::Flat{Shaders::Flat3D::Flag::Textured});

    glResources.add<PlumeShader>("plume_shader");

    glResources.add<PlanetShader>("planet_shader");

    glResources.add<SkyboxShader>("skybox_shader");

    glResources.add<RenderTexture>("render_texture");

    glResources.add<osp::active::shader::PrepassShader>("prepass_shader");

    glResources.add<osp::active::shader::RTShader>("RT_shader");

    glResources.add<osp::active::shader::CompositePass>("composite_shader");

    glResources.add<osp::active::shader::PassthroughShader>("passthrough_shader");

    // TMP: no shaderinstance
    glResources.add<Shaders::VertexColor3D>("vertexcolor_shader");

    glResources.add<osp::active::shader::BillboardShader>("billboard_shader");

    /*// Enable OpenGL debug output
    GL::Renderer::enable(GL::Renderer::Feature::DebugOutput);
    GL::Renderer::enable(GL::Renderer::Feature::DebugOutputSynchronous);
    GL::DebugOutput::setDefaultCallback();*/

    // Generate 1x1 quad for billboard rendering
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

    // Generate fullscreen tri for texture rendering
    using namespace Magnum; 
    if (glResources.get<GL::Mesh>("fullscreen_tri").empty())
    {
        Vector2 screenSize = Vector2{GL::defaultFramebuffer.viewport().size()};

        float aspectRatio = screenSize.x() / screenSize.y();

        std::array<float, 12> surfData
        {
            // Vert position    // UV coordinate
            -1.0f,  1.0f,       0.0f,  1.0f,
            -1.0f, -3.0f,       0.0f, -1.0f,
             3.0f,  1.0f,       2.0f,  1.0f
        };

        GL::Buffer surface(std::move(surfData), GL::BufferUsage::StaticDraw);
        GL::Mesh surfaceMesh;
        surfaceMesh
            .setPrimitive(Magnum::MeshPrimitive::Triangles)
            .setCount(3)
            .addVertexBuffer(std::move(surface), 0,
                RenderTexture::Position{}, RenderTexture::TextureCoordinates{});
        glResources.add<GL::Mesh>("fullscreen_tri", std::move(surfaceMesh));
    }

    // Generate single-vertex mesh for point rendering
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

/*void SysDebugRender::draw(ACompCamera& camera)
{
    GL::AbstractFramebuffer* framebuffer = (camera.m_renderTarget.empty()) ?
        reinterpret_cast<GL::AbstractFramebuffer*>(&GL::defaultFramebuffer)
        : reinterpret_cast<GL::AbstractFramebuffer*>(&(*camera.m_renderTarget));

    framebuffer->bind();

    Renderer::enable(Renderer::Feature::SeamlessCubeMapTexture);

    // Get skybox
    using Skybox_t = SkyboxShader::ACompSkyboxShaderInstance;
    auto boxview = reg.view<CompDrawableDebug, ACompTransform, Skybox_t, CompBackgroundDebug>();
    // Disable depth test
    Renderer::disable(Renderer::Feature::Blending);
    Renderer::disable(Renderer::Feature::DepthTest);
    //Renderer::enable(Renderer::Feature::DepthClamp);
    Renderer::enable(Renderer::Feature::FaceCulling);
    Renderer::setFaceCullingMode(Renderer::PolygonFacing::Front);
    // Draw skybox
    draw_group(rScene, boxview, camera);

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
    draw_group(rScene, opaqueObjects, camera);

    // Render sun test point
    auto backgroundView =
        reg.view<CompDrawableDebug, ACompTransform, CompBackgroundDebug, CompQueryObj>();
    for (ActiveEnt e : backgroundView)
    {
        auto& q = backgroundView.get<CompQueryObj>(e);
        glBeginQuery(GL_ANY_SAMPLES_PASSED, q.m_id);
    }
    draw_group(rScene, backgroundView, camera);
    glEndQuery(GL_ANY_SAMPLES_PASSED);


    // Get transparent objects
    auto transparentObjects = rScene.get_registry()
        .view<CompDrawableDebug, CompVisibleDebug,
        CompTransparentDebug, ACompTransform>(entt::exclude<CompOverlayDebug>);

    // Configure blend mode for transparency
    Renderer::enable(Renderer::Feature::Blending);
    Renderer::setBlendFunction(
        Renderer::BlendFunction::SourceAlpha,
        Renderer::BlendFunction::OneMinusSourceAlpha);

    // Draw backfaces first
    Renderer::setFaceCullingMode(Renderer::PolygonFacing::Front);
    draw_group(rScene, transparentObjects, camera);

    // Then draw frontfaces
    Renderer::setFaceCullingMode(Renderer::PolygonFacing::Back);
    draw_group(rScene, transparentObjects, camera);

    // Get overlays
    auto overlayObjects = rScene.get_registry()
        .view<CompDrawableDebug, CompVisibleDebug,
        CompTransparentDebug, ACompTransform, CompOverlayDebug>();

    GL::defaultFramebuffer.clear(GL::FramebufferClear::Depth);
    draw_group(rScene, overlayObjects, camera);
}*/

DependRes<GL::Framebuffer> SysDebugRender::create_framebuffer(ActiveScene& rScene,
    std::string_view name)
{
    auto& glResources = rScene.get_context_resources();

    Vector2i viewSize = GL::defaultFramebuffer.viewport().size();

    GL::Texture2D color;
    color.setStorage(1, GL::TextureFormat::RGB8, viewSize);
    osp::DependRes<GL::Texture2D> colorRes = glResources.add<GL::Texture2D>(
        osp::string_concat(name, "_color"), std::move(color));

    GL::Renderbuffer depthStencil;
    depthStencil.setStorage(GL::RenderbufferFormat::Depth24Stencil8, viewSize);
    osp::DependRes<GL::Renderbuffer> depthStencilRes = glResources.add<GL::Renderbuffer>(
            osp::string_concat(name, "_depthStencil"), std::move(depthStencil));

    GL::Framebuffer fbo(Range2Di{{0, 0}, viewSize});
    fbo.attachTexture(GL::Framebuffer::ColorAttachment{0}, *colorRes, 0);
    fbo.attachRenderbuffer(GL::Framebuffer::BufferAttachment::DepthStencil, *depthStencilRes);

    return glResources.add<GL::Framebuffer>(name, std::move(fbo));
}

void SysDebugRender::display_framebuffer(ActiveScene& rScene, Magnum::GL::Texture2D& rTexture)
{
    using namespace Magnum;
    
    auto& glResources = rScene.get_context_resources();

    DependRes<GL::Mesh> surface = glResources.get<GL::Mesh>("fullscreen_tri");
    DependRes<RenderTexture> shader = glResources.get<RenderTexture>("render_texture");

    shader->render_texure(*surface, rTexture);
}

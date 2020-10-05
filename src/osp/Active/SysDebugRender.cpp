#include <Magnum/GL/DefaultFramebuffer.h>
#include <Magnum/GL/Renderer.h>

#include "SysDebugRender.h"
#include "ActiveScene.h"


using namespace osp::active;

// for _1, _2, _3, ... std::bind arguments
using namespace std::placeholders;

// for the 0xrrggbb_rgbf and _deg literals
using namespace Magnum::Math::Literals;


SysDebugRender::SysDebugRender(ActiveScene &rScene) :
        m_scene(rScene),
        m_renderDebugDraw(rScene.get_render_order(), "debug", "", "",
                          std::bind(&SysDebugRender::draw, this, _1))
{
}

void SysDebugRender::draw(ACompCamera const& camera)
{
    using Magnum::GL::Renderer;
    using Magnum::Shaders::Phong;

    Renderer::enable(Renderer::Feature::DepthTest);
    Renderer::enable(Renderer::Feature::FaceCulling);

    // Configure blend mode for opaque rendering
    Renderer::disable(Renderer::Feature::Blending);

    // Get opaque objects
    auto opaqueObjects = m_scene.get_registry()
        .view<CompDrawableDebug, CompVisibleDebug,
        ACompTransform>(entt::exclude<CompTransparentDebug>);

    // TMP update opaque textures until objects own their own shaders
    // This doesn't even work because they all reference the same instance
    for (auto entity : opaqueObjects)
    {
        CompDrawableDebug& drawable = opaqueObjects.get<CompDrawableDebug>(entity);
        Phong& shader = *std::get<Phong*>(drawable.m_shader);
        shader
            .bindDiffuseTexture(*drawable.m_textures[0])
            .setAmbientColor(0x111111_rgbf)
            .setLightPosition({10.0f, 15.0f, 5.0f});
    }

    // Draw opaque objects
    draw_group(opaqueObjects, camera);

    // Configure blend mode for transparency
    Renderer::enable(Renderer::Feature::Blending);
    Renderer::setBlendFunction(
        Renderer::BlendFunction::SourceAlpha,
        Renderer::BlendFunction::OneMinusSourceAlpha);

    // Get transparent objects
    auto transparentObjects = m_scene.get_registry()
        .view<CompDrawableDebug, CompVisibleDebug,
        CompTransparentDebug, ACompTransform>();

    // Draw backfaces first
    Renderer::setFaceCullingMode(Renderer::PolygonFacing::Front);
    draw_group(transparentObjects, camera);

    // Then draw frontfaces
    Renderer::setFaceCullingMode(Renderer::PolygonFacing::Back);
    draw_group(transparentObjects, camera);
}

void SysDebugRender::draw_element(CompDrawableDebug& drawable, ACompCamera const& camera,
    ACompTransform const& transform)
{
    using Magnum::Shaders::Phong;

    Matrix4 entRelative = camera.m_inverse * transform.m_transformWorld;

    if (std::holds_alternative<Phong*>(drawable.m_shader))
    {
        (*std::get<Phong*>(drawable.m_shader))
            .bindDiffuseTexture(*drawable.m_textures[0]);
        (*std::get<Phong*>(drawable.m_shader))
            .setTransformationMatrix(entRelative)
            .setProjectionMatrix(camera.m_projection)
            .setNormalMatrix(entRelative.normalMatrix())
            .draw(*(drawable.m_mesh));
    }
    else if (std::holds_alternative<PlumeShaderInstance>(drawable.m_shader))
    {
        std::get<PlumeShaderInstance>(drawable.m_shader).draw(*drawable.m_mesh, camera, transform);
    }
}
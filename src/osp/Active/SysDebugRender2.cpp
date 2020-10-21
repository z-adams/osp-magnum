#include <Magnum/GL/DefaultFramebuffer.h>
#include <Magnum/GL/Renderer.h>

#include "SysDebugRender2.h"
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

    Renderer::disable(Renderer::Feature::DepthTest);
    Renderer::enable(Renderer::Feature::FaceCulling);
    Renderer::setClearColor(Magnum::Color4{Magnum::Color3{0.05f}, 1.0f});

    auto orbits = m_scene.get_registry()
        .view<CompDrawableDebug, ACompTransform, CompPass1Debug>();
    auto bodies = m_scene.get_registry()
        .view<CompDrawableDebug, ACompTransform>(entt::exclude<CompPass1Debug>);

    Matrix4 entRelative;

    Renderer::enable(Renderer::Feature::Blending);
    Renderer::setBlendFunction(Renderer::BlendFunction::SourceAlpha, Renderer::BlendFunction::OneMinusSourceAlpha);
    Renderer::setPointSize(1.0f);
    for(auto entity: orbits)
    {
        CompDrawableDebug& drawable = orbits.get<CompDrawableDebug>(entity);
        ACompTransform& transform = orbits.get<ACompTransform>(entity);

        entRelative = camera.m_inverse * transform.m_transformWorld;

        (*drawable.m_shader)
                .setTransformationProjectionMatrix(camera.m_projection * entRelative)
                .draw(*(drawable.m_mesh));
    }
    Renderer::disable(Renderer::Feature::Blending);
    Renderer::setPointSize(2.0f);
    for (auto entity : bodies)
    {
        /*if (entity == static_cast<ActiveEnt>(1))
        {
            Renderer::setPointSize(4.0f);
        }
        else
        {
            Renderer::setPointSize(2.0f);
        }*/
        CompDrawableDebug& drawable = bodies.get<CompDrawableDebug>(entity);
        ACompTransform& transform = bodies.get<ACompTransform>(entity);

        entRelative = camera.m_inverse * transform.m_transformWorld;

        (*drawable.m_shader)
            .setTransformationProjectionMatrix(camera.m_projection * entRelative)
            .draw(*(drawable.m_mesh));
    }
}

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

    Renderer::enable(Renderer::Feature::DepthTest);

    auto drawGroup = m_scene.get_registry().view<CompDrawableDebug, ACompTransform>();

    Matrix4 entRelative;
    Renderer::setPointSize(4.0f);
    Renderer::setLineWidth(2.0f);
    for(auto entity: drawGroup)
    {
        CompDrawableDebug& drawable = drawGroup.get<CompDrawableDebug>(entity);
        ACompTransform& transform = drawGroup.get<ACompTransform>(entity);

        entRelative = camera.m_inverse * transform.m_transformWorld;

        drawable.m_mesh->setPrimitive(Magnum::GL::MeshPrimitive::Points);

        (*drawable.m_shader)
                .setColor(0xFFFFFF_rgbf)
                .setTransformationProjectionMatrix(camera.m_projection * entRelative)
                .draw(*(drawable.m_mesh));

        drawable.m_mesh->setPrimitive(Magnum::GL::MeshPrimitive::Lines);

        (*drawable.m_shader)
            .setColor(0x00FF00_rgbf)
            .draw(*(drawable.m_mesh));

    }
}

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
#include "SysPrepass.h"
#include <osp/Shaders/RTPrepass.h>
#include <Magnum/GL/Renderer.h>
#include <Magnum/GL/Framebuffer.h>
#include <Magnum/GL/Buffer.h>

#include <osp/Active/SysDebugRender.h>
#include "adera/Shaders/PlanetShader.h"

using namespace Magnum;
using namespace osp::active;
using namespace osp::active::shader;
using namespace std::placeholders;

void PrepassExecutor::add_functions(ActiveScene& rScene)
{
    /*rScene.debug_render_add(rScene.get_render_order(), "prepass", "", "debug",
        &PrepassExecutor::execute_prepass);*/
    //SysDebugRender::create_framebuffer(rScene, "prepass_fbo");
}

void PrepassExecutor::execute_prepass(ActiveScene& rScene, ACompCamera const& camera,
    GL::Framebuffer& gBuffer)
{
    using PlanetShader_t = adera::shader::PlanetShader::ACompPlanetShaderInstance;

    auto& reg = rScene.get_registry();
    auto view = reg.view<CompDrawableDebug, ACompTransform>(
        entt::exclude<CompTransparentDebug, CompBackgroundDebug, CompOverlayDebug, PlanetShader_t>);

    using Magnum::GL::Renderer;

    Renderer::disable(Renderer::Feature::Blending);
    Renderer::enable(Renderer::Feature::DepthTest);
    Renderer::enable(Renderer::Feature::FaceCulling);
    Renderer::setFaceCullingMode(Renderer::PolygonFacing::Back);
    Renderer::disable(Renderer::Feature::Multisampling);

    auto& ctxRes = rScene.get_context_resources();

    DependRes<PrepassShader> shader = ctxRes.get<PrepassShader>("prepass_shader");
    for (auto [e, drawable, compXform] : view.each())
    {
        DependRes<GL::Mesh> mesh = drawable.m_mesh;

        Vector3 cameraPos = camera.m_inverse.inverted().translation();

        (*shader)
            .set_model_matrix(compXform.m_transformWorld)
            .set_view_matrix(camera.m_inverse)
            .set_proj_matrix(camera.m_projection)
            .set_camera_pos_world(cameraPos)
            .bind_gbuffer(gBuffer)
            .draw(*mesh);
    }
    glMemoryBarrier(GL_TEXTURE_FETCH_BARRIER_BIT);
    Renderer::enable(Renderer::Feature::Multisampling);
}

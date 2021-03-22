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
#include "SysSunflare.h"
#include <osp/Shaders/BillboardShader.h>
#include <osp/Resource/AssetImporter.h>
#include <osp/Active/SysDebugRender.h>
#include <Magnum/Trade/ImageData.h>
#include <Magnum/Shaders/VertexColor.h>

using namespace adera::active;
using namespace osp::active;

SysSunflare::SysSunflare(osp::active::ActiveScene& rScene)
    : m_updateOrder(rScene.get_update_order(), "sunflare", "physics", "",
        [](ActiveScene& rScene) { update(rScene); })
{

}

void SysSunflare::update(ActiveScene& rScene)
{
    using Shader_t = osp::active::shader::BillboardShader;
    auto& reg = rScene.get_registry();
    for (auto [e, flare, visibility] : reg.view<ACompSunflare, CompVisibleDebug>().each())
    {
        auto& query = rScene.reg_get<CompQueryObj>(flare.m_occlusionTest);
        uint32_t visible = 0;
        glGetQueryObjectuiv(query.m_id, GL_QUERY_RESULT, &visible);
        if (visible == GL_TRUE)
        {
            visibility.m_state = true;
        }
        else
        {
            visibility.m_state = false;
        }
    }
}

void SysSunflare::add_flare(ActiveScene& rScene, ActiveEnt e, std::string_view textureName)
{
    using namespace Magnum;
    using namespace osp::active;

    auto& ctxRess = rScene.get_context_resources();

    osp::DependRes<GL::Mesh> sunMesh = ctxRess.get<GL::Mesh>("billboard_quad");

    using osp::active::shader::BillboardShader;
    osp::DependRes<BillboardShader> sunShader = ctxRess.get<BillboardShader>("billboard_shader");

    osp::DependRes<GL::Texture2D> sunTex =
        ctxRess.get<GL::Texture2D>(textureName);
    if (sunTex.empty())
    {
        auto& pkg = rScene.get_application().debug_find_package("lzdb");
        auto sunImg = pkg.get<Trade::ImageData2D>(textureName);
        sunTex = osp::AssetImporter::compile_tex(sunImg, ctxRess);
    }
    rScene.reg_emplace<BillboardShader::ACompBillboardShaderInstance>(e, sunShader, sunTex);
    rScene.reg_emplace<CompDrawableDebug>(e, std::move(sunMesh),
        &BillboardShader::draw_billboard);
    rScene.reg_emplace<CompTransparentDebug>(e, true);
    rScene.reg_emplace<CompVisibleDebug>(e, true);
    rScene.reg_emplace<CompOverlayDebug>(e);
    auto& xfm = rScene.reg_emplace<ACompTransform>(e);
    xfm.m_transform = Matrix4::from(Matrix3{}, Vector3{100.0f, 0.0f, 0.0f});

    // Add child point for occlusion testing
    ActiveEnt testPoint = rScene.hier_create_child(rScene.hier_get_root());
    rScene.reg_emplace<ACompSunflare>(e, testPoint);

    osp::DependRes<GL::Mesh> pointMesh = ctxRess.get<GL::Mesh>("point");
    osp::DependRes<Shaders::VertexColor3D> vertShader =
        ctxRess.get<Shaders::VertexColor3D>("vertexcolor_shader");

    auto drawfnc = [](ActiveEnt ent, ActiveScene& rScene, GL::Mesh& mesh,
        ACompCamera const& camera, ACompTransform const& xform)
    {
        osp::DependRes<Shaders::VertexColor3D> shader =
            rScene.get_context_resources().get<Shaders::VertexColor3D>("vertexcolor_shader");

        Matrix4 cameraNoTransl = Matrix4{Matrix3{camera.m_inverse}};
        Matrix4 transform = camera.m_projection * cameraNoTransl * xform.m_transformWorld;

        (*shader)
            .setTransformationProjectionMatrix(transform)
            .draw(mesh);
    };

    rScene.reg_emplace<CompBackgroundDebug>(testPoint);
    rScene.reg_emplace<CompDrawableDebug>(testPoint, std::move(pointMesh), drawfnc);
    auto& xform = rScene.reg_emplace<ACompTransform>(testPoint);
    xform.m_transform = Matrix4::from(Matrix3{}, Vector3{1000.0f, 0.0f, 0.0f});

    // Gen query object
    auto& q = rScene.reg_emplace<CompQueryObj>(testPoint);
}

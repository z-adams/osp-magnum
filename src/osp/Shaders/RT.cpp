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
#include "RT.h"

#include <Magnum/GL/Shader.h>
#include <Magnum/GL/Version.h>
#include <Magnum/GL/ImageFormat.h>
#include <Corrade/Containers/ArrayViewStl.h>
#include <Magnum/Trade/MeshData.h>

#include <osp/Active/SysDebugRender.h>
#include <adera/Shaders/PlanetShader.h>

using namespace osp::active::shader;
using namespace Magnum;

void RTShader::init()
{
    GL::Shader prog{GL::Version::GL430, GL::Shader::Type::Compute};
    prog.addFile("OSPData/adera/Shaders/RT.comp");

    CORRADE_INTERNAL_ASSERT_OUTPUT(prog.compile());
    attachShader(prog);
    CORRADE_INTERNAL_ASSERT_OUTPUT(link());

    setUniform(
        static_cast<Int>(UniformPos::OutputImg),
        static_cast<Int>(ImageSlots::OutputMap));
}

RTShader& RTShader::bind_objects_list(GL::Buffer& objects)
{
    objects.bind(GL::Buffer::Target::Uniform, static_cast<Int>(UniformPos::ObjectBlock));
    return *this;
}

RTShader& RTShader::bind_light_list(GL::Buffer& lights)
{
    lights.bind(GL::Buffer::Target::Uniform, static_cast<Int>(UniformPos::LightBlock));
    return *this;
}

RTShader& RTShader::bind_output_img(GL::Texture2D& rTex)
{
    rTex.bindImage(static_cast<Int>(ImageSlots::OutputMap), 0,
        GL::ImageAccess::WriteOnly, GL::ImageFormat::R8);
    return *this;
}

RTShader& RTShader::bind_gbuffer(GL::Buffer& rGBuf)
{
    rGBuf.bind(GL::Buffer::Target::ShaderStorage, static_cast<Int>(UniformPos::GBuffer));
    return *this;
}

RTShader& RTShader::bind_triangle_buffer(GL::Buffer& rTriangles)
{
    rTriangles.bind(GL::Buffer::Target::ShaderStorage, static_cast<Int>(UniformPos::TriBuffer));
    return *this;
}

void RTShader::raytrace(ActiveScene& rScene, ACompCamera const& camera,
    GL::Buffer& gBuffer, GL::Texture2D& target)
{
    std::vector<ObjectData> objects;
    std::vector<DirectionalLight> lights = {DirectionalLight{{1.0f, 0.0f, 0.0f}, 1.0f}};
    std::vector<Triangle> tris;

    using adera::shader::PlanetShader;
    auto& pkg = rScene.get_application().debug_find_package("lzdb");
    auto geometry = rScene.get_registry().view<CompDrawableDebug, ACompTransform>(
        entt::exclude<PlanetShader, CompBackgroundDebug,
        CompOverlayDebug, CompTransparentDebug>);
    for (auto [e, drawable, transform] : geometry.each())
    {
        DependRes<Trade::MeshData> mesh = pkg.get<Trade::MeshData>(drawable.m_mesh.name());

        size_t firstTriIndex = tris.size();

        if (mesh->isIndexed())
        {
            auto indices = Corrade::Containers::arrayCast<const UnsignedInt>(mesh->indexData());
            auto vertices = Corrade::Containers::arrayCast<const Vector3>(mesh->vertexData());

            for (size_t i = 0; i < indices.size(); i += 3)
            {
                tris.emplace_back(
                    vertices[indices[i + 0]],
                    vertices[indices[i + 1]],
                    vertices[indices[i + 2]]
                );
            }
        }
        else
        {
            auto vertices = Corrade::Containers::arrayCast<const Vector3>(mesh->vertexData());
            for (size_t i = 0; i < vertices.size(); i += 3)
            {
                tris.emplace_back(
                    vertices[i + 0],
                    vertices[i + 1],
                    vertices[i + 2]
                );
            }
        }

        size_t numTriangles = tris.size() - firstTriIndex;

        Box boundingBox; // get from somewhere
        Box aabb = box_to_AABB(boundingBox, Matrix3{transform.m_transformWorld});
        aabb.translate(transform.m_transformWorld.translation());

        ObjectData obj
        {
            aabb.m_max,
            firstTriIndex,
            aabb.m_min,
            numTriangles
        };
    }

    if (m_objectBuffer.id() == 0)
    {
        m_objectBuffer.setData(objects);
    }
    else
    {
        m_objectBuffer.setSubData(0, objects);
    }

    if (m_lightBuffer.id() == 0)
    {
        m_lightBuffer.setData(lights);
    }
    /*else
    {
        m_objectBuffer.setSubData(0, lights);
    }*/

    if (m_triangleBuffer.id() == 0)
    {
        m_triangleBuffer.setData(tris);
    }
    /*else
    {
        m_objectBuffer.setSubData(0, tris);
    }*/

    bind_objects_list(m_objectBuffer);
    bind_light_list(m_lightBuffer);
    bind_output_img(target);
    bind_gbuffer(gBuffer);
    bind_triangle_buffer(m_triangleBuffer);

    constexpr Vector3ui dimensions{1280, 720, 0};
    dispatchCompute(dimensions);
}

Box osp::active::shader::box_to_AABB(Box& box, Magnum::Matrix3 const& rot)
{
    Box output{{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}};

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            float a = rot[i][j] * box.m_min[j];
            float b = rot[i][j] * box.m_max[j];
            output.m_min[i] += (a < b) ? a : b;
            output.m_max[i] += (a < b) ? b : a;
        }
    }

    return output;
}

using namespace osp::active;

void osp::active::SysRaytracer::add_functions(ActiveScene & rScene)
{
    rScene.debug_render_add(rScene.get_render_order(), "RT", "debug", "",
        &raytrace);
}

void SysRaytracer::raytrace(ActiveScene& rScene, ACompCamera const& camera)
{
    auto& gfxRes = rScene.get_context_resources();
    DependRes<RTShader> shader = gfxRes.get<RTShader>("RT_shader");
    DependRes<GL::Buffer> gbuf = gfxRes.get<GL::Buffer>("gbuffer");
    DependRes<GL::Texture2D> color = gfxRes.get<GL::Texture2D>("output");
    shader->raytrace(rScene, camera, *gbuf, *color);
}

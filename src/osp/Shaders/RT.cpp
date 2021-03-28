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
#include <Magnum/GL/Buffer.h>
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
    setUniform(
        static_cast<Int>(UniformPos::GRayDepth),
        static_cast<Int>(TextureSlots::RayDepth));
    setUniform(
        static_cast<Int>(UniformPos::GNormalUV),
        static_cast<Int>(TextureSlots::NormalUV));
}

RTShader& RTShader::set_uniform_counts(uint32_t nObjects, uint32_t nLights)
{
    setUniform(static_cast<Int>(UniformPos::UniformCounts),
        Vector4ui{nObjects, nLights, 0, 0});
    return *this;
}

RTShader& RTShader::bind_objects_list(GL::Buffer& objects)
{
    objects.bind(GL::Buffer::Target::Uniform, static_cast<Int>(BufferPos::ObjectsBuf));
    return *this;
}

RTShader& RTShader::bind_light_list(GL::Buffer& lights)
{
    lights.bind(GL::Buffer::Target::Uniform, static_cast<Int>(BufferPos::LightsBuf));
    return *this;
}

RTShader& RTShader::bind_output_img(GL::Texture2D& rTex)
{
    rTex.bindImage(static_cast<Int>(ImageSlots::OutputMap), 0,
        GL::ImageAccess::WriteOnly, GL::ImageFormat::RGBA8);
    return *this;
}

RTShader& RTShader::bind_gbuffer(GL::Texture2D& rRayDepth, GL::Texture2D& rNormalUV)
{
    rRayDepth.bind(static_cast<Int>(TextureSlots::RayDepth));
    rNormalUV.bind(static_cast<Int>(TextureSlots::NormalUV));
    return *this;
}

RTShader& RTShader::bind_triangle_buffer(GL::Buffer& rTriangles)
{
    rTriangles.bind(GL::Buffer::Target::ShaderStorage, static_cast<Int>(BufferPos::TriBuf));
    return *this;
}

RTShader& RTShader::set_camera_pos(Vector3 const& pos)
{
    setUniform(static_cast<Int>(UniformPos::CameraPosWorld), pos);
    return *this;
}

RTShader& RTShader::set_camera_rot(Matrix3 const& rot)
{
    setUniform(static_cast<Int>(UniformPos::CameraRot), rot);
    return *this;
}

void RTShader::raytrace(ActiveScene& rScene, ACompCamera const& camera,
    GL::Texture2D& rGRayDepth, GL::Texture2D& rGNormUV, GL::Texture2D& target)
{
    std::array<ObjectData, 64> objects;
    std::array<DirectionalLight, 1> lights{DirectionalLight{{1.0f, 0.0f, 0.0f}, 1.0f}};
    std::vector<Triangle> tris;

    using PlanetShader_t = adera::shader::PlanetShader::ACompPlanetShaderInstance;
    auto& pkg = rScene.get_application().debug_find_package("lzdb");
    auto geometry = rScene.get_registry().view<CompDrawableDebug, ACompTransform>(
        entt::exclude<PlanetShader_t, CompBackgroundDebug,
        CompOverlayDebug, CompTransparentDebug>);
    size_t objIndex = 0;
    for (auto [e, drawable, transform] : geometry.each())
    {
        DependRes<Trade::MeshData> mesh = pkg.get<Trade::MeshData>(drawable.m_mesh.name());

        size_t firstTriIndex = tris.size();

        if (mesh->isIndexed())
        {
            auto indices = Corrade::Containers::arrayCast<const UnsignedShort>(mesh->indexData());
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

        Vector3 mins{1e9f, 1e9f, 1e9f};
        Vector3 maxs{-1e9f, -1e9f, -1e9f};
        for (auto& tri : tris)
        {
            mins.x() = std::min(tri.m_v0.x(), mins.x());
            mins.y() = std::min(tri.m_v0.y(), mins.y());
            mins.z() = std::min(tri.m_v0.z(), mins.y());
            maxs.x() = std::max(tri.m_v0.x(), maxs.x());
            maxs.y() = std::max(tri.m_v0.y(), maxs.y());
            maxs.z() = std::max(tri.m_v0.z(), maxs.z());

            mins.x() = std::min(tri.m_v1.x(), mins.x());
            mins.y() = std::min(tri.m_v1.y(), mins.y());
            mins.z() = std::min(tri.m_v1.z(), mins.z());
            maxs.x() = std::max(tri.m_v1.x(), maxs.x());
            maxs.y() = std::max(tri.m_v1.y(), maxs.y());
            maxs.z() = std::max(tri.m_v1.z(), maxs.z());

            mins.x() = std::min(tri.m_v2.x(), mins.x());
            mins.y() = std::min(tri.m_v2.y(), mins.y());
            mins.z() = std::min(tri.m_v2.z(), mins.z());
            maxs.x() = std::max(tri.m_v2.x(), maxs.x());
            maxs.y() = std::max(tri.m_v2.y(), maxs.y());
            maxs.z() = std::max(tri.m_v2.z(), maxs.z());
        }
        Box boundingBox; // get from somewhere
        boundingBox.m_min = mins;
        boundingBox.m_max = maxs;
        Box aabb = box_to_AABB(boundingBox, Matrix3{transform.m_transformWorld});
        aabb.translate(transform.m_transformWorld.translation());

        ObjectData obj
        {
            aabb.m_max,
            firstTriIndex,
            aabb.m_min,
            numTriangles
        };

        objects[objIndex] = obj;
        objIndex++;
    }

    set_uniform_counts(objIndex, 1);
    m_objectBuffer.setData(objects);

    m_lightBuffer.setData(lights);
    m_triangleBuffer.setData(tris);

    bind_objects_list(m_objectBuffer);
    bind_light_list(m_lightBuffer);
    bind_output_img(target);
    bind_gbuffer(rGRayDepth, rGNormUV);
    bind_triangle_buffer(m_triangleBuffer);
    set_camera_pos(camera.m_inverse.inverted().translation());
    set_camera_rot(Matrix3{camera.m_inverse.inverted()});

    constexpr Vector3ui dimensions{1280/8, 720/8, 1};
    dispatchCompute(dimensions);
    glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
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
    /*rScene.debug_render_add(rScene.get_render_order(), "RT", "debug", "",
        &raytrace);*/
}

void SysRaytracer::raytrace(ActiveScene& rScene, ACompCamera const& camera, GL::Texture2D& color)
{
    auto& gfxRes = rScene.get_context_resources();
    DependRes<RTShader> shader = gfxRes.get<RTShader>("RT_shader");
    DependRes<GL::Texture2D> rayDepth = gfxRes.get<GL::Texture2D>("gBuffer_ray_depth");
    DependRes<GL::Texture2D> normUV = gfxRes.get<GL::Texture2D>("gBuffer_normal_uv");
    shader->raytrace(rScene, camera, *rayDepth, *normUV, color);
}

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
#pragma once

#include <Magnum/GL/AbstractShaderProgram.h>
#include <Magnum/GL/Framebuffer.h>
#include <Magnum/GL/Texture.h>
#include <Magnum/GL/Buffer.h>
#include <Magnum/Tags.h>

#include <osp/Active/ActiveScene.h>
#include <osp/Resource/Resource.h>

namespace osp::active::shader
{

#pragma pack(push, 1)
struct ObjectData
{
    Magnum::Vector3 m_AABBMaxs;
    Magnum::UnsignedInt m_firstTriIndex;
    Magnum::Vector3 m_AABBMins;
    Magnum::UnsignedInt m_numTriangles;
};
#pragma pack(pop)

struct DirectionalLight
{
    Magnum::Vector3 m_direction;
    Magnum::Float m_brightness;
};

#pragma pack(push, 1)
struct GBufferPixel
{
    Magnum::Vector3 m_castRay;
    Magnum::Float m_sampleDepth;
    Magnum::Vector2 m_normalXY;
    Magnum::Vector2 m_hitUV;
};
#pragma pack(pop)

struct Triangle
{
    Magnum::Vector3 m_v0;
    Magnum::Vector3 m_v1;
    Magnum::Vector3 m_v2;

    Triangle(Magnum::Vector3 v0, Magnum::Vector3 v1, Magnum::Vector3 v2)
        : m_v0(std::move(v0)), m_v1(std::move(v1)), m_v2(std::move(v2))
    {}
};

struct Box
{
    Magnum::Vector3 m_min;
    Magnum::Vector3 m_max;

    void translate(Magnum::Vector3 r)
    {
        m_min += r;
        m_max += r;
    }
};

// Jim Arvo, Graphics Gems (1990)
Box box_to_AABB(Box& box, Magnum::Matrix3 const& rot);

class RTShader : public Magnum::GL::AbstractShaderProgram
{
public:
    RTShader() { init(); }

    void raytrace(ActiveScene& rScene, ACompCamera const& camera,
        Magnum::GL::Buffer& gBuffer, Magnum::GL::Texture2D& target);

private:
    void init();

    // Uniforms
    enum class UniformPos : Magnum::Int
    {
        ObjectBlock = 0,
        LightBlock = 1,
        OutputImg = 2,
        GBuffer = 3,
        TriBuffer = 4
    };

    // Buffer bindings
    enum class BufferPos : Magnum::Int
    {
        ObjectsBuf = 0,
        LightsBuf = 1,
        GBuf = 3,
        TriBuf = 4
    };

    // ImageSlots
    enum class ImageSlots : Magnum::Int
    {
        OutputMap = 0
    };

    // Hide irrelevant calls
    using Magnum::GL::AbstractShaderProgram::drawTransformFeedback;
    using Magnum::GL::AbstractShaderProgram::draw;

    // Uniform bindings
    RTShader& bind_objects_list(Magnum::GL::Buffer& objects);
    RTShader& bind_light_list(Magnum::GL::Buffer& lights);
    RTShader& bind_output_img(Magnum::GL::Texture2D& rTex);
    RTShader& bind_gbuffer(Magnum::GL::Buffer& rGBuf);
    RTShader& bind_triangle_buffer(Magnum::GL::Buffer& rTriangles);

    // Data buffers (TMP)
    Magnum::GL::Buffer m_objectBuffer{Magnum::NoCreate};
    Magnum::GL::Buffer m_lightBuffer{Magnum::NoCreate};
    Magnum::GL::Buffer m_gBuffer{Magnum::NoCreate};
    Magnum::GL::Buffer m_triangleBuffer{Magnum::NoCreate};
};

}

namespace osp::active
{

class SysRaytracer
{
public:
    static void add_functions(ActiveScene& rScene);
    static void raytrace(ActiveScene& rScene, ACompCamera const& camera);
};

}

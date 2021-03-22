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
#include <Magnum/GL/Texture.h>
#include <Magnum/GL/Mesh.h>

#include <osp/Resource/Resource.h>
#include <osp/Active/ActiveScene.h>

namespace osp::active::shader
{

class BillboardShader : public Magnum::GL::AbstractShaderProgram
{
public:
    typedef Magnum::GL::Attribute<0, Magnum::Vector3> Position;
    typedef Magnum::GL::Attribute<1, Magnum::Vector2> TextureCoordinate;

    BillboardShader() { init(); }

    struct ACompBillboardShaderInstance
    {
        // Parent shader
        osp::DependRes<BillboardShader> m_shaderProgram;

        // Uniform values
        osp::DependRes<Magnum::GL::Texture2D> m_texture;

        ACompBillboardShaderInstance(
            osp::DependRes<BillboardShader> parent,
            osp::DependRes<Magnum::GL::Texture2D> texture)
            : m_shaderProgram(parent)
            , m_texture(texture)
        {}
    };

    static void draw_billboard(osp::active::ActiveEnt e,
        osp::active::ActiveScene& rScene,
        Magnum::GL::Mesh& rMesh,
        osp::active::ACompCamera const& camera,
        osp::active::ACompTransform const& transform);
private:
    void init();

    // Uniforms
    enum class UniformPos : Magnum::Int
    {
        ProjMat = 0,
        ViewTransformMat = 1,
        WorldPos = 2,
        Texture = 3
    };

    // Texture Slots
    enum class TextureSlots : Magnum::Int
    {
        BillboardTexture = 0
    };

    // Hide irrelevant calls
    using Magnum::GL::AbstractShaderProgram::drawTransformFeedback;
    using Magnum::GL::AbstractShaderProgram::dispatchCompute;

    // Private uniform setters
    BillboardShader& set_proj_matrix(Magnum::Matrix4 const& matrix);
    BillboardShader& set_transform_matrix(Magnum::Matrix4 const& matrix);
    BillboardShader& set_position_world(Magnum::Vector3 const& matrix);

    BillboardShader& bind_texture(Magnum::GL::Texture2D& rTex);
};

} // namespace osp::active::shader

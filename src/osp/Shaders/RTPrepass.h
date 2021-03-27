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
#include <Magnum/GL/Buffer.h>

#include <osp/Active/ActiveScene.h>

namespace osp::active::shader
{

class PrepassShader : Magnum::GL::AbstractShaderProgram
{
    friend class PrepassExecutor;
public:

    PrepassShader() { init(); }
    PrepassShader(PrepassShader const& copy) = delete;
    ~PrepassShader() = default;

private:
    void init();

    // Uniforms
    enum class UniformPos : Magnum::Int
    {
        modelViewMatrix = 0,
        projMatrix = 1,
        cameraPos = 2
    };

    // Buffer binding points
    enum class BufferPos : Magnum::Int
    {
        gbuffer = 0
    };

    // Output targets
    enum class Outputs : Magnum::Int
    {
        gCastRay_Depth = 0,
        gNormalXY_HitUV = 1
    };

    // Hide irrelevant calls
    using Magnum::GL::AbstractShaderProgram::drawTransformFeedback;
    using Magnum::GL::AbstractShaderProgram::dispatchCompute;

    // Private uniform setters
    PrepassShader& set_transform_matrix(Magnum::Matrix4 const& matrix);
    PrepassShader& set_proj_matrix(Magnum::Matrix4 const& matrix);
    PrepassShader& set_camera_pos_world(Magnum::Vector3 const& pos);
    PrepassShader& bind_gbuffer(Magnum::GL::Framebuffer& buffer);
};

}

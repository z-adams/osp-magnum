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
#include "Flat.h"
#include <Magnum/Math/Matrix4.h>

using namespace osp::active::shader;

void Flat::draw_entity(ActiveEnt e,
    ActiveScene& rScene, 
    Magnum::GL::Mesh& rMesh,
    ACompCamera const& camera,
    ACompTransform const& transform)
{
    auto& shaderInstance = rScene.reg_get<ACompFlatInstance>(e);
    Flat& shader = *shaderInstance.m_shaderProgram;

    Magnum::Matrix4 mvp =
        camera.m_projection * camera.m_inverse * transform.m_transformWorld;

    shader
        .bindTexture(*shaderInstance.m_texture)
        .setTransformationProjectionMatrix(mvp)
        .draw(rMesh);
}

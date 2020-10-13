#pragma once
#include <Magnum/GL/Mesh.h>
#include <Magnum/GL/AbstractShaderProgram.h>
#include <Magnum/Math/Matrix4.h>
#include "osp/Resource/Resource.h"
#include "osp/Active/activetypes.h"
#include <iostream>

/**
 * class ShaderInstance - shader instance subclass
 *
 * Since Magnum's AbstractShaderProgram represents the GL program itself and 
 * not a convenient shader instance which encapsulates its own uniform values, 
 * this class serves as a base class to implement such a wrapper for custom 
 * shaders. It uses CRTP to call the base class's update_uniforms() function
 * which loads its saved member uniform values into the referenced shader and
 * then uses the shader to draw the specified mesh.
 */
template <typename IMPL_T, typename SHADER_T>
class ShaderInstance
{
public:
    void draw(
        Magnum::GL::Mesh& mesh,
        osp::active::ACompCamera const& camera,
        osp::active::ACompTransform const& transform);

protected:
    ShaderInstance(osp::DependRes<SHADER_T> shader) : m_shader(shader) {}
    
    ShaderInstance(ShaderInstance&& other) noexcept;
    ShaderInstance& operator=(ShaderInstance&& other) noexcept;

    ShaderInstance(ShaderInstance const& other) = delete;
    ShaderInstance& operator=(ShaderInstance const& other) = delete;

    virtual void update_uniforms() = 0;

    osp::DependRes<SHADER_T> m_shader;
};

template <typename IMPL_T, typename SHADER_T>
void ShaderInstance<IMPL_T, SHADER_T>::draw(
    Magnum::GL::Mesh& mesh,
    osp::active::ACompCamera const& camera,
    osp::active::ACompTransform const& transform)
{
    update_uniforms();

    Magnum::Matrix4 entRelative = camera.m_inverse * transform.m_transformWorld;

    (*m_shader)
        .setTransformationMatrix(entRelative)
        .setProjectionMatrix(camera.m_projection)
        .setNormalMatrix(transform.m_transformWorld.normalMatrix())
        .draw(mesh);
}

template<typename IMPL_T, typename SHADER_T>
ShaderInstance<IMPL_T, SHADER_T>::ShaderInstance(ShaderInstance&& other) noexcept
{
    m_shader = std::move(other.m_shader);
}

template<typename IMPL_T, typename SHADER_T>
ShaderInstance<IMPL_T, SHADER_T>& ShaderInstance<IMPL_T, SHADER_T>::operator=(
    ShaderInstance&& other) noexcept
{
    m_shader = std::move(other.m_shader);
    return *this;
}
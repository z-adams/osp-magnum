#include "PBRShader.h"

#include <Magnum/GL/Version.h>
#include <Magnum/GL/Shader.h>
#include <Corrade/Containers/Reference.h>
#include <Magnum/GL/Texture.h>
#include <utility>

using namespace Magnum;
using osp::DependRes;
using Magnum::GL::Texture2D;

PBRShader::PBRShader()
{
    GL::Shader vert{GL::Version::GL430, GL::Shader::Type::Vertex};
    GL::Shader frag{GL::Version::GL430, GL::Shader::Type::Fragment};
    vert.addFile("OSPData/adera/Shaders/PBRShader.vert");
    frag.addFile("OSPData/adera/Shaders/PBRShader.frag");

    CORRADE_INTERNAL_ASSERT_OUTPUT(GL::Shader::compile({vert, frag}));
    attachShaders({vert, frag});
    CORRADE_INTERNAL_ASSERT_OUTPUT(link());

    // Set TexSampler2D uniforms
    setUniform(
        static_cast<Int>(UniformPos::AlbedoMap),
        static_cast<Int>(TextureSlot::AlbedoTexUnit));
    setUniform(
        static_cast<Int>(UniformPos::MetalRoughMap),
        static_cast<Int>(TextureSlot::MetalRoughTexUnit));
    setUniform(
        static_cast<Int>(UniformPos::NormalMap),
        static_cast<Int>(TextureSlot::NormalTexUnit));
}

PBRShader& PBRShader::setProjectionMatrix(const Matrix4& matrix)
{
    setUniform(static_cast<Int>(UniformPos::ProjMat), matrix);
    return *this;
}

PBRShader& PBRShader::setTransformationMatrix(const Matrix4& matrix)
{
    Vector3 cameraPos = -matrix.translation();
    setUniform(static_cast<Int>(UniformPos::CameraPos), cameraPos);
    setUniform(static_cast<Int>(UniformPos::ModelTransformMat), matrix);
    return *this;
}

PBRShader& PBRShader::setNormalMatrix(const Matrix3x3& matrix)
{
    setUniform(static_cast<Int>(UniformPos::NormalMat), matrix);
    return *this;
}

PBRShaderInstance::PBRShaderInstance(PBRShaderInstance&& other) noexcept :
    m_albedoTex(std::move(other.m_albedoTex)), m_metalRoughTex(std::move(other.m_metalRoughTex)),
    m_normalTex(std::move(other.m_normalTex)), m_sunDirection(std::move(other.m_sunDirection)),
    m_cameraPos(std::move(other.m_cameraPos)), ShaderInstance(std::move(other))
{}

PBRShaderInstance& PBRShaderInstance::operator=(PBRShaderInstance&& other) noexcept
{
    m_albedoTex = std::move(other.m_albedoTex);
    m_metalRoughTex = std::move(other.m_metalRoughTex);
    m_normalTex = std::move(other.m_normalTex);
    m_sunDirection = std::move(other.m_sunDirection);
    m_cameraPos = std::move(other.m_cameraPos);
    m_shader = std::move(other.m_shader);
    return *this;
}

PBRShaderInstance& PBRShaderInstance::set_albedo_tex(DependRes<Texture2D> tex)
{
    m_albedoTex = std::move(tex);
    return *this;
}

PBRShaderInstance& PBRShaderInstance::set_metal_rough_tex(DependRes<Texture2D> tex)
{
    m_metalRoughTex = std::move(tex);
    return *this;
}

PBRShaderInstance& PBRShaderInstance::set_normal_tex(DependRes<Texture2D> tex)
{
    m_metalRoughTex = std::move(tex);
    return *this;
}

PBRShaderInstance& PBRShaderInstance::set_sun_dir(Vector3 const& dir)
{
    m_sunDirection = dir;
    return *this;
}

PBRShaderInstance& PBRShaderInstance::set_camera_pos(Vector3 const& pos)
{
    m_cameraPos = pos;
    return *this;
}

void PBRShaderInstance::update_uniforms()
{
    m_shader->setUniform(
        static_cast<Int>(PBRShader::UniformPos::SunDirection),
        m_sunDirection);
    m_shader->setUniform(
        static_cast<Int>(PBRShader::UniformPos::CameraPos),
        m_cameraPos);
    m_albedoTex->bind(static_cast<Int>(PBRShader::TextureSlot::AlbedoTexUnit));
    m_metalRoughTex->bind(static_cast<Int>(PBRShader::TextureSlot::MetalRoughTexUnit));
    //m_normalTex->bind(static_cast<Int>(PBRShader::TextureSlot::NormalTexUnit));
}

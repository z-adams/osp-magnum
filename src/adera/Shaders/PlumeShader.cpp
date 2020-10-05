#include "PlumeShader.h"

#include <Magnum/GL/Version.h>
#include <Magnum/GL/Shader.h>
#include <Corrade/Containers/Reference.h>
#include <Magnum/GL/Texture.h>
#include <utility>

using namespace Magnum;

PlumeShader::PlumeShader()
{
    GL::Shader vert{GL::Version::GL430, GL::Shader::Type::Vertex};
    GL::Shader frag{GL::Version::GL430, GL::Shader::Type::Fragment};
    vert.addFile("OSPData/adera/Shaders/PlumeShader.vert");
    frag.addFile("OSPData/adera/Shaders/PlumeShader.frag");

    CORRADE_INTERNAL_ASSERT_OUTPUT(GL::Shader::compile({vert, frag}));
    attachShaders({vert, frag});
    CORRADE_INTERNAL_ASSERT_OUTPUT(link());

    // Set TexSampler2D uniforms
    setUniform(
        static_cast<Int>(UniformPos::NozzleNoiseTex),
        static_cast<Int>(TextureSlot::NozzleNoiseTexUnit));
    setUniform(
        static_cast<Int>(UniformPos::CombustionNoiseTex),
        static_cast<Int>(TextureSlot::CombustionNoiseTexUnit));
}

PlumeShader& PlumeShader::setProjectionMatrix(const Matrix4& matrix)
{
    setUniform(static_cast<Int>(UniformPos::ProjMat), matrix);
    return *this;
}

PlumeShader& PlumeShader::setTransformationMatrix(const Matrix4& matrix)
{
    setUniform(static_cast<Int>(UniformPos::ModelTransformMat), matrix);
    return *this;
}

PlumeShader& PlumeShader::setNormalMatrix(const Matrix3x3& matrix)
{
    setUniform(static_cast<Int>(UniformPos::NormalMat), matrix);
    return *this;
}

PlumeShader& PlumeShader::setMeshZBounds(float topZ, float bottomZ)
{
    setUniform(static_cast<Int>(UniformPos::MeshTopZ), topZ);
    setUniform(static_cast<Int>(UniformPos::MeshBottomZ), bottomZ);
    return *this;
}

PlumeShader& PlumeShader::bindNozzleNoiseTexture(Magnum::GL::Texture2D& tex)
{
    tex.bind(static_cast<Int>(TextureSlot::NozzleNoiseTexUnit));
    return *this;
}

PlumeShader& PlumeShader::bindCombustionNoiseTexture(Magnum::GL::Texture2D& tex)
{
    tex.bind(static_cast<Int>(TextureSlot::CombustionNoiseTexUnit));
    return *this;
}

PlumeShader& PlumeShader::setBaseColor(const Magnum::Color4 color)
{
    setUniform(static_cast<Int>(UniformPos::BaseColor), color);
    return *this;
}

PlumeShader& PlumeShader::setFlowVelocity(const float vel)
{
    setUniform(static_cast<Int>(UniformPos::FlowVelocity), vel);
    return *this;
}

PlumeShader& PlumeShader::updateTime(const float currentTime)
{
    setUniform(static_cast<Int>(UniformPos::Time), currentTime);
    return *this;
}

PlumeShader& PlumeShader::setPower(const float power)
{
    setUniform(static_cast<Int>(UniformPos::Power), power);
    return *this;
}

PlumeShaderInstance& PlumeShaderInstance::set_mesh_Z_bounds(float topZ, float bottomZ)
{
    m_maxZ = topZ;
    m_minZ = bottomZ;
    return *this;
}

PlumeShaderInstance& PlumeShaderInstance::set_nozzle_noise_tex(Magnum::GL::Texture2D& tex)
{
    m_nozzleTex = &tex;
    return *this;
}

PlumeShaderInstance& PlumeShaderInstance::set_combustion_noise_tex(Magnum::GL::Texture2D& tex)
{
    m_combustionTex = &tex;
    return *this;
}

PlumeShaderInstance& PlumeShaderInstance::set_base_color(const Magnum::Color4 color)
{
    m_color = color;
    return *this;
}

PlumeShaderInstance& PlumeShaderInstance::set_flow_velocity(const float vel)
{
    m_flowVelocity = vel;
    return *this;
}

PlumeShaderInstance& PlumeShaderInstance::update_time(const float currentTime)
{
    m_currentTime = currentTime;
    return *this;
}

PlumeShaderInstance& PlumeShaderInstance::set_power(const float power)
{
    m_powerLevel = power;
    return *this;
}

void PlumeShaderInstance::update_uniforms()
{
    m_shader->setMeshZBounds(m_maxZ, m_minZ);
    m_shader->bindNozzleNoiseTexture(*m_nozzleTex);
    m_shader->bindCombustionNoiseTexture(*m_combustionTex);
    m_shader->setBaseColor(m_color);
    m_shader->setFlowVelocity(m_flowVelocity);
    m_shader->updateTime(m_currentTime);
    m_shader->setPower(m_powerLevel);
}

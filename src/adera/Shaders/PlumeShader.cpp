#include "PlumeShader.h"

#include <Magnum/GL/Version.h>
#include <Magnum/GL/Shader.h>
#include <Corrade/Containers/Reference.h>
#include <Magnum/GL/Texture.h>

using namespace Magnum;

void PlumeShader::init()
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

PlumeShader::PlumeShader()
{
    init();
}

PlumeShader::PlumeShader(PlumeEffectData const& data)
{
    init();

    setMeshZBounds(data.zMax, data.zMin);
    setBaseColor(data.color);
    setFlowVelocity(data.flowVelocity);
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

#pragma once
#include <Magnum/GL/AbstractShaderProgram.h>
#include <Magnum/GL/Attribute.h>
#include <Magnum/Math/Vector4.h>
#include <Magnum/Math/Vector3.h>
#include <Magnum/Math/Vector2.h>
#include <Magnum/Math/Matrix3.h>
#include <Magnum/Math/Matrix4.h>
#include <Magnum/Math/Color.h>
#include <Magnum/Shaders/Generic.h>

#include "adera/Plume.h"

class PlumeShader : public Magnum::GL::AbstractShaderProgram
{
public:
    // Vertex attribs
    Magnum::Shaders::Generic3D::Position Position;
    Magnum::Shaders::Generic3D::Normal Normal;
    Magnum::Shaders::Generic3D::TextureCoordinates TextureCoordinates;

    // Outputs
    enum : Magnum::UnsignedInt
    {
        ColorOutput = 0
    };

    PlumeShader();
    PlumeShader(PlumeEffectData const& data);

    PlumeShader& setProjectionMatrix(const Magnum::Matrix4& matrix);
    PlumeShader& setTransformationMatrix(const Magnum::Matrix4& matrix);
    PlumeShader& setNormalMatrix(const Magnum::Matrix3x3& matrix);

    PlumeShader& setMeshZBounds(float topZ, float bottomZ);
    PlumeShader& bindNozzleNoiseTexture(Magnum::GL::Texture2D& tex);
    PlumeShader& bindCombustionNoiseTexture(Magnum::GL::Texture2D& tex);
    PlumeShader& setBaseColor(const Magnum::Color4 color);
    PlumeShader& setFlowVelocity(const float vel);
    PlumeShader& updateTime(const float currentTime);
    PlumeShader& setPower(const float power);


private:
    // GL init
    void init();

    // Uniforms
    enum class UniformPos : Magnum::Int
    {
        ProjMat = 0,
        ModelTransformMat = 1,
        NormalMat = 2,
        MeshTopZ = 3,
        MeshBottomZ = 4,
        NozzleNoiseTex = 5,
        CombustionNoiseTex = 6,
        BaseColor = 7,
        FlowVelocity = 8,
        Time = 9,
        Power = 10
    };

    // Texture2D slots
    enum class TextureSlot : Magnum::Int
    {
        NozzleNoiseTexUnit = 0,
        CombustionNoiseTexUnit = 1
    };

    // Hide irrelevant calls
    using Magnum::GL::AbstractShaderProgram::drawTransformFeedback;
    using Magnum::GL::AbstractShaderProgram::dispatchCompute;
};
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
#include "osp/Active/activetypes.h"
#include "osp/Resource/ShaderInstance.h"

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

class PlumeShaderInstance : public ShaderInstance<PlumeShaderInstance, PlumeShader>
{
private:
    friend class ShaderInstance<PlumeShaderInstance, PlumeShader>;
    using PSI = PlumeShaderInstance;
    using Base = ShaderInstance<PlumeShaderInstance, PlumeShader>;
public:
    PlumeShaderInstance(osp::DependRes<PlumeShader> shader) : ShaderInstance(shader)
    {
        m_minZ = 0.0f;
        m_maxZ = 0.0f;
        m_nozzleTex = nullptr;
        m_combustionTex = nullptr;
        m_color = Magnum::Color4{1.0f};
        m_flowVelocity = 0.0f;
        m_currentTime = 0.0f;
        m_powerLevel = 0.0f;
    }

    PlumeShaderInstance(PlumeEffectData const& data, osp::DependRes<PlumeShader> shader)
        : ShaderInstance(shader)
    {
        m_minZ = data.zMin;
        m_maxZ = data.zMax;
        m_color = data.color;
        m_flowVelocity = data.flowVelocity;

        m_nozzleTex = nullptr;
        m_combustionTex = nullptr;
        m_currentTime = 0.0f;
        m_powerLevel = 0.0f;
    }

    PlumeShaderInstance(PlumeShaderInstance&& other) noexcept = default;
    PSI& operator=(PSI&& other) noexcept = default;

    PlumeShaderInstance(PSI const& other) = delete;
    PSI& operator=(PSI const& other) = delete;

    PSI& set_mesh_Z_bounds(float topZ, float bottomZ);
    PSI& set_nozzle_noise_tex(Magnum::GL::Texture2D& tex);
    PSI& set_combustion_noise_tex(Magnum::GL::Texture2D& tex);
    PSI& set_base_color(const Magnum::Color4 color);
    PSI& set_flow_velocity(const float vel);
    PSI& update_time(const float currentTime);
    PSI& set_power(const float power);

private:
    void update_uniforms() override;

    // Uniform values
    float m_minZ;
    float m_maxZ;
    Magnum::GL::Texture2D* m_nozzleTex;
    Magnum::GL::Texture2D* m_combustionTex;
    Magnum::Color4 m_color;
    float m_flowVelocity;
    float m_currentTime;
    float m_powerLevel;
};
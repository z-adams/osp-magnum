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
#include <Magnum/GL/Texture.h>

#include "adera/Plume.h"
#include "osp/Active/activetypes.h"
#include "osp/Resource/Resource.h"
#include "osp/Resource/ShaderInstance.h"

class PBRShader : public Magnum::GL::AbstractShaderProgram
{
    friend class PBRShaderInstance;
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

    PBRShader();

    PBRShader& setProjectionMatrix(const Magnum::Matrix4& matrix);
    PBRShader& setTransformationMatrix(const Magnum::Matrix4& matrix);
    PBRShader& setNormalMatrix(const Magnum::Matrix3x3& matrix);

private:
    // Uniforms
    enum class UniformPos : Magnum::Int
    {
        ProjMat = 0,
        ModelTransformMat = 1,
        NormalMat = 2,
        SunDirection = 3,
        CameraPos = 4,
        AlbedoMap = 5,
        MetalRoughMap = 6,
        NormalMap = 7,
    };

    // Texture2D slots
    enum class TextureSlot : Magnum::Int
    {
        AlbedoTexUnit = 0,
        MetalRoughTexUnit = 1,
        NormalTexUnit = 2
    };

    // Hide irrelevant calls
    using Magnum::GL::AbstractShaderProgram::drawTransformFeedback;
    using Magnum::GL::AbstractShaderProgram::dispatchCompute;
};

class PBRShaderInstance : public ShaderInstance<PBRShaderInstance, PBRShader>
{
private:
    friend class ShaderInstance<PBRShaderInstance, PBRShader>;
    using PBRS = PBRShaderInstance;
    using Base = ShaderInstance<PBRShaderInstance, PBRShader>;
public:
    PBRShaderInstance(osp::DependRes<PBRShader> shader) :
        m_albedoTex(), m_metalRoughTex(), m_normalTex(),
        m_sunDirection(1, 0, 0), m_cameraPos(0, 1, 0),
        ShaderInstance(shader)
    { }

    PBRShaderInstance(PBRShaderInstance&& other) noexcept;
    PBRS& operator=(PBRS&& other) noexcept;

    PBRShaderInstance(PBRS const& other) = delete;
    PBRS& operator=(PBRS const& other) = delete;

    PBRS& set_albedo_tex(osp::DependRes<Magnum::GL::Texture2D> tex);
    PBRS& set_metal_rough_tex(osp::DependRes<Magnum::GL::Texture2D> tex);
    PBRS& set_normal_tex(osp::DependRes<Magnum::GL::Texture2D> tex);
    PBRS& set_sun_dir(Magnum::Vector3 const& dir);
    PBRS& set_camera_pos(Magnum::Vector3 const& pos);

private:
    void update_uniforms() override;

    // Uniform values
    osp::DependRes<Magnum::GL::Texture2D> m_albedoTex;
    osp::DependRes<Magnum::GL::Texture2D> m_metalRoughTex;
    osp::DependRes<Magnum::GL::Texture2D> m_normalTex;
    Magnum::Vector3 m_sunDirection;
    Magnum::Vector3 m_cameraPos;
};
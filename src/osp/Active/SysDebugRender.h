#pragma once

#include <variant>
#include <Magnum/Math/Color.h>
#include <Magnum/GL/Mesh.h>
#include <Magnum/Shaders/Phong.h>

#include "../types.h"
#include "activetypes.h"
#include "adera/Shaders/PlumeShader.h"

namespace osp::active
{

struct CompDrawableDebug
{
    Magnum::GL::Mesh* m_mesh;
    std::vector<Magnum::GL::Texture2D*> m_textures;
    std::variant<Magnum::Shaders::Phong*, PlumeShaderInstance> m_shader;
    Magnum::Color4 m_color;
};

struct CompTransparentDebug
{
    bool state = false;
};

struct CompVisibleDebug
{
    bool state = true;
};

class SysDebugRender : public IDynamicSystem
{
public:
    SysDebugRender(ActiveScene &rScene);
    ~SysDebugRender() = default;

    void draw(ACompCamera const& camera);

private:
    template <typename T>
    void draw_group(T& collection, ACompCamera const& camera);

    void draw_element(CompDrawableDebug& drawable, ACompCamera const& camera,
        ACompTransform const& transform);

    ActiveScene &m_scene;

    RenderOrderHandle m_renderDebugDraw;
};

template<typename T>
inline void SysDebugRender::draw_group(T& collection, ACompCamera const& camera)
{
    using Magnum::Shaders::Phong;

    for (auto entity : collection)
    {
        CompDrawableDebug& drawable = collection.get<CompDrawableDebug>(entity);
        ACompTransform& transform = collection.get<ACompTransform>(entity);
        CompVisibleDebug& visible = collection.get<CompVisibleDebug>(entity);

        if (!visible.state) { continue; }

        draw_element(drawable, camera, transform);
    }
}

}

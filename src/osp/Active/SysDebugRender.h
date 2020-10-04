#pragma once

#include <Magnum/Math/Color.h>
#include <Magnum/GL/Mesh.h>
#include <Magnum/Shaders/Phong.h>

#include "../types.h"
#include "activetypes.h"

namespace osp::active
{

struct CompDrawableDebug
{
    Magnum::GL::Mesh* m_mesh;
    std::vector<Magnum::GL::Texture2D*> m_textures;
    Magnum::Shaders::Phong* m_shader;
    Magnum::Color4 m_color;
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
    ActiveScene &m_scene;

    RenderOrderHandle m_renderDebugDraw;
};

}

#pragma once

#include <Magnum/Math/Color.h>
#include <Magnum/GL/Mesh.h>
#include <Magnum/Shaders/Phong.h>
#include <Magnum/Shaders/Flat.h>

#include "../types.h"
#include "activetypes.h"

namespace osp::active
{

struct CompDrawableDebug
{
    Magnum::GL::Mesh* m_mesh;
    std::vector<Magnum::GL::Texture2D*> m_textures;
    Magnum::Shaders::Flat3D* m_shader;
    Magnum::Color4 m_color;
};

struct CompPass1Debug
{
    bool m_dummy;
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

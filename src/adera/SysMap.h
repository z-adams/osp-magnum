#pragma once
#include "osp/Active/activetypes.h"
#include "osp/Active/ActiveScene.h"
#include "osp/Universe.h"
#include <Magnum/GL/Mesh.h>
#include <Magnum/GL/Buffer.h>
#include <Magnum/Shaders/VertexColor.h>
#include <Magnum/Math/Color.h>

namespace osp::active
{

struct PathVertex
{
    Magnum::Vector3 m_pos;
    Magnum::Color4 m_col;
};

struct OrbitPathData
{
    active::ActiveEnt m_drawEnt;
    Magnum::GL::Buffer m_indexBuf;
    Magnum::GL::Buffer m_vertexBuf;
    Magnum::GL::Mesh m_meshData;
    Magnum::Color3 m_color;
    unsigned m_lastVertIdx;
};

class SysMap : public IDynamicSystem
{
public:
    SysMap(ActiveScene &scene, universe::Universe& universe);
    ~SysMap() = default;

    void update_map();

private:
    static constexpr int m_orbitSamples = 500;

    void create_graphics_data(osp::universe::Satellite ent,
        Magnum::Color3 color = Magnum::Color3{0.25f});
    void set_orbit_circle(osp::universe::Satellite ent, float radius);
    void set_sun_sphere(osp::universe::Satellite ent, float radius);
    static Vector3 universe_to_render_space(Vector3s v3s);

    universe::Universe& m_universe;
    active::ActiveScene& m_scene;
    UpdateOrderHandle m_updateMap;


    std::map<universe::Satellite,
        std::pair<active::ActiveEnt, OrbitPathData>> m_mapping;
    Magnum::Shaders::VertexColor3D m_shader;
    Magnum::GL::Mesh m_pointMesh;
};

}  // osp::active


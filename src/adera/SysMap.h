#pragma once
#include "osp/Active/activetypes.h"
#include "osp/Active/ActiveScene.h"
#include "osp/Universe.h"
#include <Magnum/GL/Mesh.h>
#include <Magnum/GL/Buffer.h>
#include <Magnum/Shaders/Flat.h>
#include <Magnum/Math/Color.h>

namespace osp::active
{

class SysMap : public IDynamicSystem
{
public:
    SysMap(ActiveScene &scene, universe::Universe& universe);
    ~SysMap() = default;

    void update_map();

private:
    static constexpr int m_orbitSamples = 100;

    void create_graphics_data(osp::universe::Satellite ent,
        Magnum::Color3 color = Magnum::Color3{0.25f});
    void set_orbit_circle(osp::universe::Satellite ent, float radius);
    static Vector3 universe_to_render_space(Vector3s v3s);

    universe::Universe& m_universe;
    active::ActiveScene& m_scene;
    UpdateOrderHandle m_updateMap;

    struct OrbitPathData
    {
        active::ActiveEnt m_drawEnt;
        Magnum::GL::Buffer m_bufData;
        Magnum::GL::Mesh m_meshData;
    };

    std::map<universe::Satellite,
        std::pair<active::ActiveEnt, OrbitPathData>> m_mapping;
    Magnum::Shaders::Flat3D m_shader;
    Magnum::GL::Mesh m_pointMesh;
};

}  // osp::active


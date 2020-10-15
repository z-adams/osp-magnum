#include "SysMap.h"
#include "osp/Active/SysDebugRender2.h"
#include <Magnum/Mesh.h>
#include <Magnum/GL/Texture.h>
#include <Corrade/Containers/ArrayViewStl.h>
#include "planet-a/Satellites/SatPlanet.h"

using namespace osp::active;
using namespace osp::universe;
using namespace Magnum;
using Magnum::GL::Texture2D;
using namespace Magnum::Math::Literals;
using namespace planeta::universe;

osp::active::SysMap::SysMap(ActiveScene &scene, Universe& universe) :
    m_universe(universe), m_scene(scene),
    m_updateMap(scene.get_update_order(), "map_screen", "physics", "debug",
        std::bind(&SysMap::update_map, this)),
    m_pointMesh(Magnum::MeshPrimitive::Points),
    m_shader()
{
    Magnum::GL::Buffer buf;
    buf.setData({0.0f, 0.0f, 0.0f});
    m_pointMesh
        .setCount(1)
        .addVertexBuffer(std::move(buf), 0, Shaders::Flat3D::Position{});

    // Create sun radius visualizer
    create_graphics_data(static_cast<Satellite>(1), 0xFFC800_rgbf);
    set_sun_sphere(static_cast<Satellite>(1), 6.9634e8 / 1e6);
}

void SysMap::update_map()
{
    auto& reg = m_universe.get_reg();
    auto view = reg.view<UCompTransformTraj, UCompType, UCompPlanet>();

    for (universe::Satellite sat : view)
    {
        UCompTransformTraj& tt = m_universe.get_reg().get<UCompTransformTraj>(sat);
        std::string name = tt.m_name;
        
        if (static_cast<int>(sat) == 1) { continue; }

        auto &posTraj = view.get<universe::UCompTransformTraj>(sat);
        auto &type = view.get<universe::UCompType>(sat);
        auto &pos = posTraj.m_position;

        if (m_mapping.find(sat) == m_mapping.end())
        {
            create_graphics_data(sat, posTraj.m_color);

            float radius = universe_to_render_space(pos).length();
            set_orbit_circle(sat, radius);
        }

        ACompTransform& transform = m_scene.reg_get<ACompTransform>(m_mapping[sat].first);
        transform.m_transformWorld = Matrix4::translation(universe_to_render_space(pos));
    }
}

void osp::active::SysMap::create_graphics_data(Satellite sat, Color3 color)
{
    using namespace Magnum::GL;

    // Add point mesh
    active::ActiveEnt bodyEnt = m_scene.get_registry().create();
    m_scene.reg_emplace<ACompTransform>(bodyEnt);
    m_scene.reg_emplace<CompDrawableDebug>(bodyEnt,
        &m_pointMesh, std::vector<Texture2D*>{}, &m_shader, 0xFFFFFF_rgbf);

    // Initialize orbit data
    active::ActiveEnt pathEnt = m_scene.get_registry().create();

    OrbitPathData data{pathEnt, Buffer(), Mesh()};

    std::vector<Vector3> bufData(m_orbitSamples, {0.0f, 0.0f, 0.0f});
    data.m_bufData.setData(bufData, BufferUsage::StreamDraw);
    data.m_meshData
        .setPrimitive(Magnum::MeshPrimitive::LineLoop)
        .setCount(m_orbitSamples)
        .addVertexBuffer(data.m_bufData, 0, Shaders::Flat3D::Position{});

    auto& obj = m_mapping.emplace(sat, std::make_pair(bodyEnt, std::move(data)));

    // Add orbit comps
    m_scene.reg_emplace<ACompTransform>(pathEnt);
    m_scene.reg_emplace<CompDrawableDebug>(pathEnt,
        &obj.first->second.second.m_meshData, std::vector<Texture2D*>{}, &m_shader, color);
    m_scene.reg_emplace<CompPass1Debug>(pathEnt);
}

void SysMap::set_orbit_circle(Satellite ent, float radius)
{
    std::vector<Vector3> points(m_orbitSamples);
    constexpr float dA = (2.0 * 3.14159265359) / m_orbitSamples;
    for (int i = 0; i < m_orbitSamples; i++)
    {
        points[i] = {radius*cos(i*dA), radius*sin(i*dA), 0.0f};
    }
    auto& pair = m_mapping[ent];
    pair.second.m_bufData.setData(points);
}

void SysMap::set_sun_sphere(Satellite ent, float radius)
{
    std::vector<Vector3> points(m_orbitSamples);
    int index = 0;
    // Account for overlapping indices
    constexpr int samples = m_orbitSamples - 2;
    constexpr double PI = 3.14159265359;
    constexpr float PI_2 = PI / 2.0;
    constexpr float dA = 3.0 * (2.0 * PI) / samples;
    // Draw XY ring
    constexpr int steps1 = samples / 3 + 1;
    for (int i = 0; i < steps1; i++, index++)
    {
        points[index] = {radius * cos(i * dA), radius * sin(i * dA), 0.0f};
    }

    // Draw first 1/4 of XZ ring
    constexpr int steps2 = (samples / 3) / 4;
    for (int i = 0; i <= steps2; i++, index++)
    {
        points[index] = {radius * cos(i * dA), 0.0f, radius * sin(i * dA)};
    }
    // Now we're at (0, 0, 1)
    // Draw YZ ring
    constexpr int steps3 = steps1;
    for (int i = 0; i < steps3; i++, index++)
    {
        points[index] = {0.0f, radius * sin(i * dA), radius * cos(i * dA)};
    }
    // Back at (0, 0, 1)
    // Draw remaining 3/4 of XZ ring
    constexpr int steps4 = steps2 * 3;
    for (int i = 0; i <= steps4; i++, index++)
    {
        points[index] = {radius * sin(-i * dA), 0.0f, radius * cos(i * dA)};
    }
    auto& pair = m_mapping[ent];
    pair.second.m_bufData.setData(points);
}

Vector3 osp::active::SysMap::universe_to_render_space(Vector3s v3s)
{
    constexpr int64_t units_per_m = 1024ll;
    float x = static_cast<double>(v3s.x() / units_per_m) / 1e6;
    float y = static_cast<double>(v3s.y() / units_per_m) / 1e6;
    float z = static_cast<double>(v3s.z() / units_per_m) / 1e6;
    return Vector3{x, y, z};
}

#include "SysMap.h"
#include "osp/Active/SysDebugRender2.h"
#include <Magnum/Mesh.h>
#include <Magnum/GL/Texture.h>
#include <Corrade/Containers/Array.h>
#include <Corrade/Containers/ArrayViewStl.h>
#include "planet-a/Satellites/SatPlanet.h"
#include "osp/Trajectories/NBody.h"

using namespace osp::active;
using namespace osp::universe;
using namespace Magnum;
using Magnum::GL::Texture2D;
using namespace Magnum::Math::Literals;
using namespace planeta::universe;
using namespace Corrade::Containers;

osp::active::SysMap::SysMap(ActiveScene &scene, Universe& universe) :
    m_universe(universe), m_scene(scene),
    m_updateMap(scene.get_update_order(), "map_screen", "physics", "debug",
        std::bind(&SysMap::update_map, this)),
    m_mapData(scene, 6'000, 1'000)
{
    // Create sun point
    m_mapData.add_point(static_cast<Satellite>(1));
}

void SysMap::update_map()
{
    auto& reg = m_universe.get_reg();
    auto view = reg.view<UCompTransformTraj, UCompType, UCompPlanet>();

    for (Satellite sat : view)
    {
        if (m_mapData.add_point(sat))
        {

        }

        auto pos = m_universe.get_reg().get<UCompTransformTraj>(sat).m_position;
        m_mapData.get_point_pos(sat) = universe_to_render_space(pos);
    }

    /*for (universe::Satellite sat : view)
    {
        UCompTransformTraj& tt = m_universe.get_reg().get<UCompTransformTraj>(sat);
        std::string name = tt.m_name;
        
        if (static_cast<int>(sat) == 1) { continue; }

        auto &posTraj = view.get<universe::UCompTransformTraj>(sat);
        auto &type = view.get<universe::UCompType>(sat);
        auto &pos = posTraj.m_position;

        if (reg.has<TCompAsteroid>(sat))
        {
            if (m_pointsOnly.find(sat) == m_pointsOnly.end())
            {
                // Add point mesh
                active::ActiveEnt bodyEnt = m_scene.get_registry().create();
                m_scene.reg_emplace<ACompTransform>(bodyEnt);
                m_scene.reg_emplace<CompDrawableDebug>(bodyEnt,
                    &m_pointMesh, std::vector<Texture2D*>{}, &m_shader, 0xFFFFFF_rgbf);
                m_pointsOnly.emplace(sat, bodyEnt);
            }

            Vector3 newPos = universe_to_render_space(pos);
            ACompTransform& transform = m_scene.reg_get<ACompTransform>(m_pointsOnly[sat]);
            transform.m_transformWorld = Matrix4::translation(newPos);
            continue;
        }

        if (m_mapping.find(sat) == m_mapping.end())
        {
            create_graphics_data(sat, posTraj.m_color);

            float radius = universe_to_render_space(pos).length();
            set_orbit_circle(sat, radius);
        }

        auto& mapElem = m_mapping[sat];
        auto& pathData = mapElem.second;
        ArrayView<ColorVertex> pathDataBuf = arrayCast<ColorVertex>(pathData.m_vertexBuf
            .map(0, m_orbitSamples*sizeof(ColorVertex),
                GL::Buffer::MapFlag::Write|GL::Buffer::MapFlag::Read));
        ArrayView<UnsignedInt> pathIdxBuf = arrayCast<UnsignedInt>(pathData.m_indexBuf
            .map(0, m_orbitSamples * sizeof(UnsignedInt),
                GL::Buffer::MapFlag::Write));

        for (ColorVertex& pv : pathDataBuf)
        {
            pv.m_col.a() = Magnum::Math::clamp(pv.m_col.a() - 0.003f, 0.0f, 1.0f);
        }

        *for (UnsignedInt& idx : pathIdxBuf)
        {
            idx++;
            if (idx >= m_orbitSamples) { idx = 0; }
        }*

        Vector3 newPos = universe_to_render_space(pos);
        ACompTransform& transform = m_scene.reg_get<ACompTransform>(mapElem.first);
        transform.m_transformWorld = Matrix4::translation(newPos);

        unsigned lastIdx = pathData.m_lastVertIdx;
        pathData.m_lastVertIdx++;
        if (pathData.m_lastVertIdx >= m_orbitSamples) { pathData.m_lastVertIdx = 0; }

        pathDataBuf[pathData.m_lastVertIdx] = {newPos, Color4{pathData.m_color, 0.0f}};
        pathDataBuf[lastIdx].m_col.a() = 1.0f;
        
        pathIdxBuf[pathData.m_lastVertIdx] = pathData.m_lastVertIdx; //0;

        pathData.m_vertexBuf.unmap();
        pathData.m_indexBuf.unmap();
    }*/

    m_mapData.update();
}

/*void osp::active::SysMap::create_graphics_data(Satellite sat, Color3 color)
{
    using namespace Magnum::GL;

    auto& traj = m_universe.get_reg().get<UCompTransformTraj>(sat);
    Vector3 startPos = universe_to_render_space(traj.m_position);

    // Add point mesh
    active::ActiveEnt bodyEnt = m_scene.get_registry().create();
    m_scene.reg_emplace<ACompTransform>(bodyEnt);
    m_scene.reg_emplace<CompDrawableDebug>(bodyEnt,
        &m_pointMesh, std::vector<Texture2D*>{}, &m_shader, 0xFFFFFF_rgbf);

    // Initialize orbit data
    active::ActiveEnt pathEnt = m_scene.get_registry().create();

    OrbitPathData data{pathEnt, Buffer(), Buffer(), Mesh(), color, 0};

    std::vector<ColorVertex> vertBufData(m_orbitSamples, {startPos, Color4{color, 0.0f}});
    std::vector<UnsignedInt> indBufData(m_orbitSamples);
    for (UnsignedInt i = 0; i < m_orbitSamples; i++)
    {
        indBufData[i] = i;
    }
    data.m_vertexBuf.setData(vertBufData, BufferUsage::StreamDraw);
    data.m_indexBuf.setData(indBufData, BufferUsage::StreamDraw);
    data.m_meshData
        .setPrimitive(Magnum::MeshPrimitive::LineStrip)
        .setCount(m_orbitSamples)
        .addVertexBuffer(data.m_vertexBuf, 0,
            Shaders::VertexColor3D::Position{},
            Shaders::VertexColor3D::Color4{})
        .setIndexBuffer(data.m_indexBuf, 0, GL::MeshIndexType::UnsignedInt);

    auto& obj = m_mapping.emplace(sat, std::make_pair(bodyEnt, std::move(data)));

    // Add orbit comps
    m_scene.reg_emplace<ACompTransform>(pathEnt);
    m_scene.reg_emplace<CompDrawableDebug>(pathEnt,
        &obj.first->second.second.m_meshData, std::vector<Texture2D*>{}, &m_shader, color);
    m_scene.reg_emplace<CompPass1Debug>(pathEnt);
}

void SysMap::set_orbit_circle(Satellite ent, float radius)
{
    // Estimate angle covered
    double vel = m_universe.get_reg().get<osp::universe::TCompVel>(ent)
        .m_vel.length() / (1024.0 * 1e6);
    float timeToExhaustPoints = m_orbitSamples * TrajNBody::m_timestep;

    // s = r * theta -> theta = s / r
    float theta = (vel * timeToExhaustPoints) / radius;

    std::vector<ColorVertex> points(m_orbitSamples);
    float dA = theta / m_orbitSamples;
    for (int i = 0; i < m_orbitSamples; i++)
    {
        Vector3 pos = {radius*cos(i*dA), radius*sin(i*dA), 0.0f};
        points[i] = {pos, Color4{0.0f}};
    }
    auto& pair = m_mapping[ent];
    pair.second.m_vertexBuf.setData(points);
}

void SysMap::set_sun_sphere(Satellite ent, float radius)
{
    std::vector<ColorVertex> points(m_orbitSamples, {Vector3{0.0f}, Color4{0xFFC800_rgbf, 1.0f}});
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
        points[index].m_pos = {radius * cos(i * dA), radius * sin(i * dA), 0.0f};
    }

    // Draw first 1/4 of XZ ring
    constexpr int steps2 = (samples / 3) / 4;
    for (int i = 0; i <= steps2; i++, index++)
    {
        points[index].m_pos = {radius * cos(i * dA), 0.0f, radius * sin(i * dA)};
    }
    // Now we're at (0, 0, 1)
    // Draw YZ ring
    constexpr int steps3 = steps1;
    for (int i = 0; i < steps3; i++, index++)
    {
        points[index].m_pos = {0.0f, radius * sin(i * dA), radius * cos(i * dA)};
    }
    // Back at (0, 0, 1)
    // Draw remaining 3/4 of XZ ring
    constexpr int steps4 = steps2 * 3;
    for (int i = 0; i <= steps4; i++, index++)
    {
        points[index].m_pos = {radius * sin(-i * dA), 0.0f, radius * cos(i * dA)};
    }
    auto& pair = m_mapping[ent];
    pair.second.m_vertexBuf.setData(points);

    std::vector<UnsignedInt> indices;
    indices.reserve(m_orbitSamples);
    for (UnsignedInt i = 0; i < m_orbitSamples; i++)
    {
        indices.push_back(i);
    }
    pair.second.m_indexBuf.setData(indices);
}*/

Vector3 osp::active::SysMap::universe_to_render_space(Vector3s v3s)
{
    constexpr int64_t units_per_m = 1024ll;
    float x = static_cast<double>(v3s.x() / units_per_m) / 1e6;
    float y = static_cast<double>(v3s.y() / units_per_m) / 1e6;
    float z = static_cast<double>(v3s.z() / units_per_m) / 1e6;
    return Vector3{x, y, z};
}

MapRenderData::MapRenderData(ActiveScene& scene, unsigned maxPoints, unsigned maxPathVertices)
    : m_maxPoints(maxPoints), m_maxPathVerts(maxPathVertices),
    m_nextPathIndex(0), m_nextPointIndex(0), m_shader()
{
    using namespace Magnum::GL;

    // Allocate point data
    m_points = std::vector<ColorVertex>(m_maxPoints, {Vector3(0.0f), Color4(1.0f)});
    m_pointBuffer.setData(m_points, BufferUsage::StreamDraw);
    m_pointMesh
        .setPrimitive(GL::MeshPrimitive::Points)
        .setCount(0)
        .addVertexBuffer(m_pointBuffer, 0,
            Shaders::VertexColor3D::Position{},
            Shaders::VertexColor3D::Color4{});

    // Allocate path data
    std::vector<ColorVertex> pathVerts(m_maxPathVerts, {Vector3(0.0f), Color4{1.0f}});
    m_vertexBuffer.setData(pathVerts, BufferUsage::StreamDraw);
    std::vector<GLuint> pathIndices(m_maxPathVerts, 0);
    m_indexBuffer.setData(pathIndices, BufferUsage::StreamDraw);
    m_mesh
        .setPrimitive(GL::MeshPrimitive::LineStrip)
        .setCount(0)
        .addVertexBuffer(m_vertexBuffer, 0,
            Shaders::VertexColor3D::Position{},
            Shaders::VertexColor3D::Color4{})
        .setIndexBuffer(m_indexBuffer, 0, GL::MeshIndexType::UnsignedInt);


    // Create entities and components
    m_pointsEnt = scene.get_registry().create();
    scene.reg_emplace<ACompTransform>(m_pointsEnt);
    scene.reg_emplace<CompDrawableDebug>(m_pointsEnt,
        &m_pointMesh, std::vector<Texture2D*>{}, &m_shader);

    m_pathsEnt = scene.get_registry().create();
    scene.reg_emplace<ACompTransform>(m_pathsEnt);
    scene.reg_emplace<CompDrawableDebug>(m_pathsEnt,
        &m_mesh, std::vector<Texture2D*>{}, &m_shader);
    scene.reg_emplace<CompPass1Debug>(m_pathsEnt);

}

bool MapRenderData::add_point(Satellite object, Vector3 pos)
{
    if (m_pointMapping.find(object) != m_pointMapping.end()) { return false; }

    m_pointMapping.emplace(object, m_nextPointIndex);
    m_pointMesh.setCount(m_nextPointIndex + 1);
    m_points[m_nextPointIndex].m_pos = pos;
    m_nextPointIndex++;

    return true;
}

bool MapRenderData::add_path(universe::Satellite object, unsigned vertices)
{
    if (m_pathMapping.find(object) != m_pathMapping.end()) { return false; }

    std::vector<ColorVertex> vertBufData(vertices);
    std::vector<UnsignedInt> indBufData(vertices);
    for (UnsignedInt i = 0; i < vertices; i++)
    {
        indBufData[i] = 0;
    }

    m_vertexBuffer.setSubData(m_nextPathIndex, vertBufData);
    m_indexBuffer.setSubData(m_nextPathIndex, indBufData);
    m_mesh.setCount(m_nextPathIndex + vertices);
    m_nextPathIndex += vertices;
    
    return true;
}

Vector3& MapRenderData::get_point_pos(Satellite object)
{
    return m_points[m_pointMapping.at(object)].m_pos;
}

void MapRenderData::update()
{
    m_pointBuffer.setData(m_points, GL::BufferUsage::DynamicDraw);
}

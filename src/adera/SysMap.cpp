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
    m_mapData(scene, 6'000, 35 * m_orbitSamples)
{
}

void SysMap::update_map()
{
    auto& reg = m_universe.get_reg();
    auto view = reg.view<UCompTransformTraj, UCompPlanet>();
    auto pathView = reg.view<UCompTransformTraj, UCompPlanet>(entt::exclude<TCompAsteroid>);

    for (Satellite sat : view)
    {
        auto pos = reg.get<UCompTransformTraj>(sat).m_position;
        Vector3 renderSpace = universe_to_render_space(pos);
        m_mapData.get_point_pos(sat) = renderSpace;
        if (!reg.has<TCompAsteroid>(sat))
        {
            m_mapData.push_path_pos(sat, renderSpace);
        }
    }

    m_mapData.update();
}

void SysMap::check_and_initialize_objects()
{
    auto& reg = m_universe.get_reg();
    auto view = reg.view<UCompTransformTraj, UCompPlanet>();
    auto pathView = reg.view<UCompTransformTraj, UCompPlanet>(entt::exclude<TCompAsteroid>);

    for (Satellite sat : view)
    {
        m_mapData.add_point(sat);
    }

    for (Satellite sat : pathView)
    {
        auto& tt = pathView.get<UCompTransformTraj>(sat);
        Vector3 initPos = universe_to_render_space(tt.m_position);
        m_mapData.add_path(sat, m_orbitSamples, tt.m_color, initPos);
    }
}

/*void SysMap::set_orbit_circle(Satellite ent, float radius)
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

bool MapRenderData::add_path(universe::Satellite object, unsigned nVertices,
    Color3 color, Vector3 initPos)
{
    if (m_pathMapping.find(object) != m_pathMapping.end()) { return false; }

    PathSegData pathData;
    pathData.m_startIdx = m_nextPathIndex;
    pathData.m_endIdx = m_nextPathIndex + nVertices - 1;
    pathData.m_nextIdx = 0;

    // Account for the primitive restart index at the end of each segment
    unsigned nIndices = nVertices + 1;

    std::vector<ColorVertex> vertBufData(nVertices, {initPos, Color4{color, 1.0f}});
    std::vector<GLuint> indBufData(nIndices, pathData.m_startIdx);
    indBufData.back() = PRIMITIVE_RESTART;

    m_vertexBuffer.setSubData(m_nextPathIndex * sizeof(ColorVertex), vertBufData);
    m_indexBuffer.setSubData(m_nextPathIndex * sizeof(GLuint), indBufData);
    m_mesh.setCount(m_nextPathIndex + nIndices);
    m_nextPathIndex += nIndices;

    m_pathMapping.emplace(object, std::move(pathData));

    return true;
}

Vector3& MapRenderData::get_point_pos(Satellite object)
{
    return m_points[m_pointMapping.at(object)].m_pos;
}

void MapRenderData::push_path_pos(Satellite object, Vector3 pos)
{
    PathSegData& pathData = m_pathMapping.at(object);
    GLuint begin = pathData.m_startIdx;
    GLuint nElems = pathData.m_endIdx - begin + 1;
    GLuint& index = pathData.m_nextIdx;

    ArrayView<ColorVertex> pathDataBuf = arrayCast<ColorVertex>(m_vertexBuffer
        .map(begin * sizeof(ColorVertex),
            nElems * sizeof(ColorVertex),
            GL::Buffer::MapFlag::Write | GL::Buffer::MapFlag::Read));
    ArrayView<GLuint> pathIdxBuf = arrayCast<GLuint>(m_indexBuffer
        .map(begin * sizeof(GLuint),
            nElems * sizeof(GLuint),
            GL::Buffer::MapFlag::Write | GL::Buffer::MapFlag::Read));

    ColorVertex& vert = pathDataBuf[index];
    vert.m_pos = pos;
    for (GLuint& i : pathIdxBuf)
    {
        i++;
        if (i > pathData.m_endIdx) { i = begin; }
    }
    for (ColorVertex& v : pathDataBuf)
    {
        v.m_col.a() = Magnum::Math::clamp(v.m_col.a() - 0.002f, 0.0f, 1.0f);
    }
    vert.m_col.a() = 1.0f;
    pathIdxBuf[index] = begin;
    index++;
    if (index >= nElems) { index = 0; }

    m_vertexBuffer.unmap();
    m_indexBuffer.unmap();
}

void MapRenderData::update()
{
    m_pointBuffer.setData(m_points, GL::BufferUsage::DynamicDraw);
}

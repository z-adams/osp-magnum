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

struct ColorVertex
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

class MapRenderData
{
public:
    MapRenderData(ActiveScene& scene, unsigned maxPoints, unsigned maxPathVertices);
    ~MapRenderData() = default;
    MapRenderData(MapRenderData const& copy) = delete;
    MapRenderData(MapRenderData&& move) = default;

    bool add_point(universe::Satellite object, Vector3 pos = Vector3{0.0f});
    bool add_path(universe::Satellite object, unsigned vertices);

    Vector3& get_point_pos(universe::Satellite object);

    void update();
private:

    // "Primitive restart" index which signifies to break a line strip at that vertex
    static constexpr GLuint PRIMITIVE_RESTART = std::numeric_limits<GLuint>::max();
    const GLuint m_maxPoints;
    const GLuint m_maxPathVerts;

    // Path object data
    active::ActiveEnt m_pathsEnt;
    GLuint m_nextPathIndex;
    Magnum::GL::Buffer m_indexBuffer;
    Magnum::GL::Buffer m_vertexBuffer;
    Magnum::GL::Mesh m_mesh;

    // Point object data
    active::ActiveEnt m_pointsEnt;
    GLuint m_nextPointIndex;
    std::vector<ColorVertex> m_points;
    Magnum::GL::Buffer m_pointBuffer;
    Magnum::GL::Mesh m_pointMesh;
    
    // Shader (shared for now, may need custom)
    Magnum::Shaders::VertexColor3D m_shader;

    struct PathSegData
    {
        GLuint m_startIdx;
        GLuint m_endIdx;
    };
    // Mapping from satellites to paths
    std::map<universe::Satellite, PathSegData> m_pathMapping;

    // Mapping from satellites to point sprites
    std::map<universe::Satellite, GLuint> m_pointMapping;
};

class SysMap : public IDynamicSystem
{
public:
    SysMap(ActiveScene &scene, universe::Universe& universe);
    ~SysMap() = default;

    void update_map();
    void check_and_initialize_objects();

    static Vector3 universe_to_render_space(Vector3s v3s);
private:
    static constexpr int m_orbitSamples = 500;

    /*void create_graphics_data(osp::universe::Satellite ent,
        Magnum::Color3 color = Magnum::Color3{0.25f});
    void set_orbit_circle(osp::universe::Satellite ent, float radius);
    void set_sun_sphere(osp::universe::Satellite ent, float radius);*/

    universe::Universe& m_universe;
    active::ActiveScene& m_scene;
    UpdateOrderHandle m_updateMap;

    MapRenderData m_mapData;
};

}  // osp::active


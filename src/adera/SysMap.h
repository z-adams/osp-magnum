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

class MapPath
{
    friend class MapRenderData;
public:
    MapPath() : MapPath(nullptr, 0) {}

    MapPath(MapPath const& copy) = delete;
    MapPath& operator=(MapPath const& copy) = delete;

    MapPath(MapPath&& move) noexcept;
    MapPath& operator=(MapPath&& move) noexcept;

    ~MapPath();
private:
    MapPath(MapRenderData* owner, size_t index) : m_index(index), m_owner(owner) {}

    MapRenderData* m_owner{nullptr};
    size_t m_index{0};
};

struct MCompPath
{
    MapPath m_path;
};

class MapRenderData
{
public:
    // "Primitive restart" index which signifies to break a line strip at that vertex
    static constexpr GLuint PRIMITIVE_RESTART = std::numeric_limits<GLuint>::max();

    MapRenderData(ActiveScene& scene, unsigned maxPoints, unsigned maxPathVertices);
    ~MapRenderData() = default;
    MapRenderData(MapRenderData const& copy) = delete;
    MapRenderData(MapRenderData&& move) = default;

    bool add_point(universe::Satellite object, Vector3 pos = Vector3{0.0f});
    MapPath add_path(unsigned vertices, Magnum::Color3 color = Magnum::Color3{1.0f},
        Vector3 initPos = Vector3{0.0f});

    size_t alloc_path();
    void free_path(MapPath const& path);

    Vector3& get_point_pos(universe::Satellite object);
    void push_path_pos(MapPath const& path, Vector3 pos);
    void write_path_data(MapPath const& path, std::vector<Vector3> const& data);

    void update();
private:
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

    struct PathSegInfo
    {
        GLuint m_startIdx;
        GLuint m_endIdx;
        GLuint m_nextIdx;
    };
    // Mapping from satellites to paths
    std::vector<bool> m_pathUsed;
    std::vector<PathSegInfo> m_pathPool;
    //std::map<universe::Satellite, PathSegInfo> m_pathMapping;

    // Mapping from satellites to point sprites
    std::map<universe::Satellite, GLuint> m_pointMapping;
};

class SysMap : public IDynamicSystem
{
public:
    SysMap(ActiveScene &scene, universe::Universe& universe);
    ~SysMap();

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


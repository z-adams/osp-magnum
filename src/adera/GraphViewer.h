#pragma once
#include <vector>

#include "osp/types.h"
#include "osp/Active/activetypes.h"
#include "osp/Active/ActiveScene.h"
#include "adera/MassSpringLattice.h"

#include <Magnum/GL/Buffer.h>
#include <Magnum/GL/Mesh.h>
#include <Magnum/Shaders/MeshVisualizer.h>
#include <Magnum/Shaders/Flat.h>

struct Node
{
    Magnum::Vector3 m_position;
};

struct Edge
{
    unsigned m_nodeA;
    unsigned m_nodeB;
};

struct Graph
{
    std::vector<Node> m_nodes;
    std::vector<Edge> m_edges;
    Magnum::GL::Buffer m_vertBuffer;
    Magnum::GL::Mesh m_mesh;
};

struct CompGraphDebug
{
    size_t m_index;
};

class SysGraphViewer : public osp::active::IDynamicSystem
{
public:
    static constexpr GLuint PRIMITIVE_RESTART = std::numeric_limits<GLuint>::max();

    SysGraphViewer(osp::active::ActiveScene& scene);
    ~SysGraphViewer() = default;

    void update_view();
    void update_graph(Graph& g, std::vector<LatticeMass> points);
    void check_and_initialize_objects();

private:
    osp::active::ActiveScene& m_scene;
    osp::active::UpdateOrderHandle m_updateGraphView;

    Magnum::Shaders::Flat3D m_shader;

    std::vector<Graph> m_graphs;
};

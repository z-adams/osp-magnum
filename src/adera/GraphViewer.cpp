#include "GraphViewer.h"
#include "osp/Active/SysDebugRender2.h"
#include <Magnum/Math/Color.h>
#include <Corrade/Containers/Array.h>
#include <Corrade/Containers/ArrayViewStl.h>

using namespace osp::active;
using namespace Magnum::Math::Literals;

SysGraphViewer::SysGraphViewer(ActiveScene& scene)
    :m_scene(scene),
    m_updateGraphView(scene.get_update_order(), "graph_view", "physics", "debug",
        std::bind(&SysGraphViewer::update_view, this)), m_shader()
{}

void SysGraphViewer::update_view()
{
    auto view = m_scene.get_registry().view<LCompSoftBody, CompGraphDebug>();
    for (ActiveEnt e : view)
    {
        auto& softbody = view.get<LCompSoftBody>(e);
        auto& graph = m_graphs[view.get<CompGraphDebug>(e).m_index];

        update_graph(graph, softbody.m_masses);
    }
}

void SysGraphViewer::update_graph(Graph& g, std::vector<LatticeMass> points)
{
    using namespace Corrade::Containers;
    using Magnum::Vector3;
    using namespace Magnum;

    ArrayView<Vector3> vertBuf = arrayCast<Magnum::Vector3>(g.m_vertBuffer
        .map(0, g.m_nodes.size() * sizeof(Vector3),
            GL::Buffer::MapFlag::Write | GL::Buffer::MapFlag::Read));

    for (size_t i = 0; i < g.m_nodes.size(); i++)
    {
        g.m_nodes[i].m_position = points[i].m_pos;
        vertBuf[i] = points[i].m_pos;
    }

    g.m_vertBuffer.unmap();
}

void SysGraphViewer::check_and_initialize_objects()
{
    auto view = m_scene.get_registry().view<LCompSoftBody>();
    for (ActiveEnt e : view)
    {
        auto& softbody = view.get<LCompSoftBody>(e);

        Graph graph;

        for (auto const& m : softbody.m_masses)
        {
            graph.m_nodes.push_back({m.m_pos});
        }
        for (auto const& s : softbody.m_springs)
        {
            graph.m_edges.push_back({s.m_massA, s.m_massB});
        }

        using namespace Magnum::GL;
        
        // Allocate GPU vert data
        graph.m_vertBuffer.setData(graph.m_nodes, BufferUsage::StreamDraw);

        // Allocate GPU edge/index data
        std::vector<GLuint> indexBuffer;
        for (size_t i = 0; i < graph.m_edges.size(); i++)
        {
            indexBuffer.push_back(graph.m_edges[i].m_nodeA);
            indexBuffer.push_back(graph.m_edges[i].m_nodeB);
        }
        Buffer edgeBuffer(indexBuffer, BufferUsage::StaticDraw);

        graph.m_mesh
            .setPrimitive(MeshPrimitive::Lines)
            //.setCount(graph.m_nodes.size())
            .setCount(indexBuffer.size())
            .addVertexBuffer(graph.m_vertBuffer, 0, Magnum::Shaders::Flat3D::Position{})
            .setIndexBuffer(std::move(edgeBuffer), 0, MeshIndexType::UnsignedInt);

        m_scene.reg_emplace<CompGraphDebug>(e, m_graphs.size());
        m_graphs.push_back(std::move(graph));

        CompDrawableDebug drawable;
        drawable.m_color = 0x000011_rgbf;
        drawable.m_mesh = &m_graphs.back().m_mesh;
        drawable.m_shader = &m_shader;
        drawable.m_textures = {};
        m_scene.reg_emplace<CompDrawableDebug>(e, std::move(drawable));
    }
}

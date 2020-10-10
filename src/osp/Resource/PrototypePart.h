#pragma once

#include <string>
#include <array>
#include <vector>
#include <variant>

#include "../types.h"

namespace osp
{

enum class ObjectType
{
    NONE,       // Normal old object
    MESH,       // It has a mesh
    COLLIDER//,   // It's a collider
    //ATTACHMENT  //
};

enum class ECollisionShape : uint8_t
{
    NONE,
    COMBINED,
    SPHERE,
    BOX,
    CAPSULE,
    CYLINDER,
    //MESH,
    CONVEX_HULL,
    TERRAIN
};

float col_shape_volume(ECollisionShape shape, Vector3 scale);

//const uint32_t gc_OBJ_MESH      = 1 << 2;
//const uint32_t gc_OBJ_COLLIDER  = 1 << 3;


struct DrawableData
{

    // index to a PrototypePart.m_meshDataUsed
    unsigned m_mesh;
    std::vector<unsigned> m_textures;
    //std::string m_mesh;
    //unsigned m_material;
};

struct ColliderData
{
    ECollisionShape m_type;
    unsigned m_meshData;
};

using config_node_t = std::variant<double, int, std::string>;

struct PrototypeMachine
{
    std::string m_type;
    std::map<std::string, config_node_t> m_config;
};

struct PrototypeObject
{
    unsigned m_parentIndex;
    unsigned m_childCount;
    //unsigned m_mesh;

    uint32_t m_bitmask;

    std::string m_name;



    //Magnum::Matrix4 m_transform;
    // maybe not use magnum types here
    Magnum::Vector3 m_translation;
    Magnum::Quaternion m_rotation;
    Magnum::Vector3 m_scale;

    ObjectType m_type;

    std::variant<DrawableData, ColliderData> m_objectData;

    // Put more OSP-specific data in here
    std::vector<PrototypeMachine> m_machines;

};

/**
 * Describes everything on how to construct a part, loaded directly from a file
 * or something
 */
class PrototypePart
{

public:
    PrototypePart();

    constexpr std::vector<PrototypeObject>& get_objects()
    { return m_objects; }
    constexpr std::vector<PrototypeObject> const& get_objects() const
    { return m_objects; }

    constexpr float& get_mass()
    { return m_mass; }
    constexpr float get_mass() const
    { return m_mass; }


    constexpr std::vector<std::string>& get_strings()
    { return m_strings; }

private:
    //std::string name; use path

    std::vector<PrototypeObject> m_objects;

    //std::vector<DependRes<MeshData3D> > m_meshDataUsed;

    // TODO: more OSP information
    float m_mass;

    // std::vector <machines>

    std::vector<std::string> m_strings;

};

}

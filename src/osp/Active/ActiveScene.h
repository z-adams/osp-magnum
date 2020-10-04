#pragma once

#include <utility>
#include <vector>

#include "../OSPApplication.h"
#include "../UserInputHandler.h"

#include "../types.h"
#include "activetypes.h"
#include "physics.h"

#include "SysDebugRender.h"
#include "SysNewton.h"
#include "SysMachine.h"
//#include "SysVehicle.h"
#include "SysWire.h"
#include "adera/SysExhaustPlume.h"

namespace osp::active
{


/**
 * Component for transformation (in meters)
 */
struct ACompTransform
{
    //Matrix4 m_transformPrev;
    Matrix4 m_transform;
    Matrix4 m_transformWorld;
    bool m_enableFloatingOrigin;
};

struct ACompHierarchy
{
    std::string m_name;

    //unsigned m_childIndex;
    unsigned m_level{0};
    ActiveEnt m_parent{entt::null};
    ActiveEnt m_siblingNext{entt::null};
    ActiveEnt m_siblingPrev{entt::null};

    // as a parent
    unsigned m_childCount{0};
    ActiveEnt m_childFirst{entt::null};

    // transform relative to parent

};

/**
 * Component that represents a camera
 */
struct ACompCamera
{
    float m_near, m_far;
    Magnum::Deg m_fov;
    Vector2 m_viewport;

    Matrix4 m_projection;
    Matrix4 m_inverse;


    void calculate_projection();
};

/**
 * The 3D Game Engine scene. ActiveScene implements a scene graph hierarchy and
 * contains a bunch of other systems.
 */
class ActiveScene
{

public:
    ActiveScene(UserInputHandler &userInput, OSPApplication& app);
    ~ActiveScene();

    OSPApplication& get_application() { return m_app; };

    /**
     * @return Root entity of the entire scene graph
     */
    constexpr ActiveEnt hier_get_root() { return m_root; }

    /**
     * Create a new entity, and add a ACompHierarchy to it
     * @param parent [in] Entity to assign as parent
     * @param name   [in] Name of entity
     * @return New entity created
     */
    ActiveEnt hier_create_child(ActiveEnt parent,
                                std::string const& name = "Innocent Entity");

    /**
     * (NOT YET IMPLEMENTED) set parent-child relationship between two nodes
     * containing a ACompHierarchy
     * @param parent
     * @param child
     */
    void hier_set_parent_child(ActiveEnt parent, ActiveEnt child);

    /**
     * Destroy an entity and all its descendents
     * @param ent
     */
    void hier_destroy(ActiveEnt ent);

    /**
     * Cut an entity out of the hierarchy
     * @param ent
     */
    void hier_cut(ActiveEnt ent);

    /**
     * Traverse the scene hierarchy
     * 
     * Calls the specified callable on each entity of the scene hierarchy
     * @param root The entity whose sub-tree to traverse
     * @param callable A function that accepts an ActiveEnt as an argument and
     *                 returns false if traversal should stop, true otherwise
     */
    template <typename FUNC_T>
    void hierarchy_traverse(ActiveEnt root, FUNC_T callable);

    /**
     * @return Internal entt::registry
     */
    constexpr entt::registry& get_registry() { return m_registry; }

    /**
     * Shorthand for get_registry().get<T>()
     * @tparam T Component to get
     * @return Reference to component
     */
    template<class T>
    constexpr T& reg_get(ActiveEnt ent) { return m_registry.get<T>(ent); };

    /**
     * Shorthand for get_registry().emplace<T>()
     * @tparam T Component to emplace
     */
    template<class T, typename... Args>
    constexpr T& reg_emplace(ActiveEnt ent, Args&& ... args)
    {
        return m_registry.emplace<T>(ent, std::forward<Args>(args)...);
    }

    /**
     * Update everything in the update order, including all systems and stuff
     */
    void update();

    /**
     * Update the m_transformWorld of entities with ACompTransform and
     * ACompHierarchy. Intended for physics interpolation
     */
    void update_hierarchy_transforms();

    /**
     * Request a floating origin mass translation. Multiple calls to this are
     * accumulated and are applied on the next physics update
     * @param pAmount Meters to translate
     */
    void floating_origin_translate(Vector3 const& pAmount);

    /**
     * @return Accumulated total of floating_origin_translate
     */
    constexpr Vector3 const& floating_origin_get_total() { return m_floatingOriginTranslate; }

    /**
     * Attempt ro perform translations on this frame. Will do nothing if the
     * floating origin total (m_floatingOriginTranslate) is 0
     */
    void floating_origin_translate_begin();

    /**
     * @return True if a floating origin translation is being performed this
     *         frame
     */
    constexpr bool floating_origin_in_progress() { return m_floatingOriginInProgress; }

    /**
     * Calculate transformations relative to camera, and draw every
     * CompDrawableDebug
     * @param camera [in] Entity containing a ACompCamera
     */
    void draw(ActiveEnt camera);

    constexpr UserInputHandler& get_user_input() { return m_userInput; }

    constexpr UpdateOrder& get_update_order() { return m_updateOrder; }

    constexpr RenderOrder& get_render_order() { return m_renderOrder; }

    // TODO
    constexpr float get_time_delta_fixed() { return 1.0f / 60.0f; }

    /**
     * Get one of the system members
     * @tparam T either SysPhysics, SysWire, or SysDebugObject
     */
    template<class T>
    constexpr T& get_system();

    /**
     *
     * @tparam T
     */
    template<class T, typename... Args>
    void system_machine_add(std::string const& name, Args &&... args);

    /**
     *
     * @tparam T
     */
    template<class T, typename... Args>
    T& dynamic_system_add(std::string const& name, Args &&... args);

    /**
     * Get a registered SysMachine by name. This accesses a map.
     * @param name [in] Name used as a key
     * @return Iterator to specified SysMachine
     */
    MapSysMachine::iterator system_machine_find(std::string const& name);

    bool system_machine_it_valid(MapSysMachine::iterator it);

private:

    void on_hierarchy_construct(entt::registry& reg, ActiveEnt ent);
    void on_hierarchy_destruct(entt::registry& reg, ActiveEnt ent);

    OSPApplication& m_app;

    //std::vector<std::vector<ActiveEnt> > m_hierLevels;
    entt::basic_registry<ActiveEnt> m_registry;
    ActiveEnt m_root;
    bool m_hierarchyDirty;

    Vector3 m_floatingOriginTranslate;
    bool m_floatingOriginInProgress;

    float m_timescale;


    UserInputHandler &m_userInput;
    //std::vector<std::reference_wrapper<ISysMachine>> m_update_sensor;
    //std::vector<std::reference_wrapper<ISysMachine>> m_update_physics;

    UpdateOrder m_updateOrder;
    RenderOrder m_renderOrder;

    MapSysMachine m_sysMachines;
    MapDynamicSys m_dynamicSys;

    // TODO: base class and a list for Systems (or not)
    SysDebugRender m_render;
    SysPhysics m_physics;
    SysWire m_wire;
    SysExhaustPlume m_exhaustPlume;
    //SysVehicle m_vehicles;

    //SysDebugObject m_debugObj;
    //std::tuple<SysPhysics, SysWire, SysDebugObject> m_systems;

};

// move these to another file eventually

template<class T, typename... Args>
void ActiveScene::system_machine_add(std::string const& name,
                                             Args &&... args)
{
    m_sysMachines.emplace(name, std::make_unique<T>(*this, args...));
}

template<class T, typename... Args>
T& ActiveScene::dynamic_system_add(std::string const& name,
                                             Args &&... args)
{
    auto ptr = std::make_unique<T>(*this, args...);
    T &refReturn = *(ptr.get());

    auto pair = m_dynamicSys.emplace(name, std::move(ptr));

    return refReturn;
}

// TODO: There's probably a better way to do these:

template<>
constexpr SysPhysics& ActiveScene::get_system<SysPhysics>()
{
    return m_physics;
}

template<>
constexpr SysWire& ActiveScene::get_system<SysWire>()
{
    return m_wire;
}

//template<>
///constexpr SysDebugObject& ActiveScene::get_system<SysDebugObject>()
//{
//    return m_debugObj;
//}

template<typename FUNC_T>
void ActiveScene::hierarchy_traverse(ActiveEnt root, FUNC_T callable)
{
    using osp::active::ACompHierarchy;

    std::vector<ActiveEnt> parentNextSibling;
    ActiveEnt currentEnt = root;

    parentNextSibling.reserve(16);
    while (true)
    {
        ACompHierarchy &hier = reg_get<ACompHierarchy>(currentEnt);

        if (!callable(currentEnt)) { return; }

        if (hier.m_childCount)
        {
            // entity has some children
            currentEnt = hier.m_childFirst;


            // save next sibling for later if it exists
            if (hier.m_siblingNext != entt::null)
            {
                parentNextSibling.push_back(hier.m_siblingNext);
            }
        } else if (hier.m_siblingNext != entt::null)
        {
            // no children, move to next sibling
            currentEnt = hier.m_siblingNext;
        } else if (parentNextSibling.size())
        {
            // last sibling, and not done yet
            // is last sibling, move to parent's (or ancestor's) next sibling
            currentEnt = parentNextSibling.back();
            parentNextSibling.pop_back();
        } else
        {
            break;
        }
    }
}

}

#include <iostream>

#include "ActiveScene.h"

using namespace osp;
using namespace osp::active;

void ACompCamera::calculate_projection()
{

    m_projection = Matrix4::perspectiveProjection(
            m_fov, m_viewport.x() / m_viewport.y(),
            m_near, m_far);
}


ActiveScene::ActiveScene(UserInputHandler &userInput, OSPApplication &app) :
        m_app(app),
        m_hierarchyDirty(false),
        m_userInput(userInput),
        m_render(*this),
        m_physics(*this),
        m_wire(*this),
        m_exhaustPlume(*this)
{
    m_registry.on_construct<ACompHierarchy>()
                    .connect<&ActiveScene::on_hierarchy_construct>(*this);

    m_registry.on_destroy<ACompHierarchy>()
                    .connect<&ActiveScene::on_hierarchy_destruct>(*this);

    // "There is no need to store groups around for they are extremely cheap to
    //  construct, even though they can be copied without problems and reused
    //  freely. A group performs an initialization step the very first time
    //  it's requested and this could be quite costly. To avoid it, consider
    //  creating the group when no components have been assigned yet."
    m_registry.group<ACompHierarchy, ACompTransform>();

    // Create the root entity

    m_root = m_registry.create();
    ACompHierarchy& hierarchy = m_registry.emplace<ACompHierarchy>(m_root);
    hierarchy.m_name = "Root Entity";

}

ActiveScene::~ActiveScene()
{
    // destruct dynamic systems before registry
    m_dynamicSys.clear();
    m_registry.clear();
}


entt::entity ActiveScene::hier_create_child(entt::entity parent,
                                            std::string const& name)
{
    entt::entity child = m_registry.create();
    ACompHierarchy& childHierarchy = m_registry.emplace<ACompHierarchy>(child);
    //ACompTransform& transform = m_registry.emplace<ACompTransform>(ent);
    childHierarchy.m_name = name;

    hier_set_parent_child(parent, child);

    return child;
}

void ActiveScene::hier_set_parent_child(entt::entity parent, entt::entity child)
{
    ACompHierarchy& childHierarchy = m_registry.get<ACompHierarchy>(child);
    //ACompTransform& transform = m_registry.emplace<ACompTransform>(ent);

    ACompHierarchy& parentHierarchy = m_registry.get<ACompHierarchy>(parent);

    // if child has an existing parent, cut first
    if (m_registry.valid(childHierarchy.m_parent))
    {
        hier_cut(child);
    }

    // set new child's parent
    childHierarchy.m_parent = parent;
    childHierarchy.m_level = parentHierarchy.m_level + 1;

    // If has siblings (not first child)
    if (parentHierarchy.m_childCount)
    {
        entt::entity const sibling = parentHierarchy.m_childFirst;
        ACompHierarchy& siblingHierarchy
                = m_registry.get<ACompHierarchy>(sibling);

        // Set new child and former first child as siblings
        siblingHierarchy.m_siblingPrev = child;
        childHierarchy.m_siblingNext = sibling;
    }

    // Set parent's first child to new child just created
    parentHierarchy.m_childFirst = child;
    parentHierarchy.m_childCount ++; // increase child count
}

void ActiveScene::hier_destroy(ActiveEnt ent)
{
    auto &entHier = m_registry.get<ACompHierarchy>(ent);

    // recurse into children. this can be optimized more but i'm lazy
    while (entHier.m_childCount)
    {
        hier_destroy(entHier.m_childFirst);
    }

    hier_cut(ent);
    m_registry.destroy(ent);
}

void ActiveScene::hier_cut(ActiveEnt ent)
{
    auto &entHier = m_registry.get<ACompHierarchy>(ent);

    // Set sibling's sibling's to each other

    if (m_registry.valid(entHier.m_siblingNext))
    {
        m_registry.get<ACompHierarchy>(entHier.m_siblingNext).m_siblingPrev
                = entHier.m_siblingPrev;
    }

    if (m_registry.valid(entHier.m_siblingPrev))
    {
        m_registry.get<ACompHierarchy>(entHier.m_siblingPrev).m_siblingNext
                = entHier.m_siblingNext;
    }

    // deal with parent

    auto &parentHier = m_registry.get<ACompHierarchy>(entHier.m_parent);
    parentHier.m_childCount --;

    if (parentHier.m_childFirst == ent)
    {
        parentHier.m_childFirst = entHier.m_siblingNext;
    }

    entHier.m_parent = entHier.m_siblingNext = entHier.m_siblingPrev
            = entt::null;
}

void ActiveScene::on_hierarchy_construct(entt::registry& reg, entt::entity ent)
{
    //std::cout << "hierarchy entity constructed\n";

    m_hierarchyDirty = true;
}

void ActiveScene::on_hierarchy_destruct(entt::registry& reg, entt::entity ent)
{
    std::cout << "hierarchy entity destructed\n";

}

void ActiveScene::update()
{
    // Update machine systems
    //for (ISysMachine &sysMachine : m_update_sensor)
    //{
    //    sysMachine.update_sensor();
    //}

    //m_wire.update_propigate();

    //for (ISysMachine &sysMachine : m_update_physics)
    //{
    //    sysMachine.update_physics(1.0f/60.0f);
    //}
    m_updateOrder.call();
}

void ActiveScene::floating_origin_translate_begin()
{

    // check if floating origin translation is requested
    m_floatingOriginInProgress = !(m_floatingOriginTranslate.isZero());
    if (m_floatingOriginInProgress)
    {
        std::cout << "Floating origin translation\n";
    }
}

void ActiveScene::update_hierarchy_transforms()
{
    // skip top level
    /*for(auto it = m_hierLevels.begin() + 1; it != m_hierLevels.end(); it ++)
    {
        for (entt::entity entity : *it)
        {

        }
    }*/
    auto group = m_registry.group<ACompHierarchy, ACompTransform>();

    if (m_hierarchyDirty)
    {
        // Sort by level, so that objects closer to the root are iterated first
        group.sort<ACompHierarchy>([](ACompHierarchy const& lhs,
                                     ACompHierarchy const& rhs)
        {
            return lhs.m_level < rhs.m_level;
        }, entt::insertion_sort());
        //group.sortable();
        m_hierarchyDirty = false;
    }

    //std::cout << "size: " << group.size() << "\n";

    //bool translateAll = !(m_floatingOriginTranslate.isZero());

    for(auto entity: group)
    {
        //std::cout << "nice: " << group.get<ACompHierarchy>(entity).m_name << "\n";

        ACompHierarchy& hierarchy = group.get<ACompHierarchy>(entity);
        ACompTransform& transform = group.get<ACompTransform>(entity);

        if (hierarchy.m_parent == m_root)
        {
            // top level object, parent is root

            if (m_floatingOriginInProgress && transform.m_enableFloatingOrigin)
            {
                // Do floating origin translation if enabled
                Vector3& translation = transform.m_transform[3].xyz();
                translation += m_floatingOriginTranslate;
            }

            transform.m_transformWorld = transform.m_transform;

        }
        else
        {
            ACompTransform& parentTransform
                    = group.get<ACompTransform>(hierarchy.m_parent);

            // set transform relative to parent
            transform.m_transformWorld = parentTransform.m_transformWorld
                                          * transform.m_transform;

        }
    }

    if (m_floatingOriginInProgress)
    {
        // everything was translated already, set back to zero
        m_floatingOriginTranslate = Vector3(0.0f, 0.0f, 0.0f);
        m_floatingOriginInProgress = false;
    }
}

void ActiveScene::floating_origin_translate(Vector3 const& amount)
{
    m_floatingOriginTranslate += amount;
    //m_floatingOriginDirty = true;
}

void ActiveScene::draw(entt::entity camera)
{
    //Matrix4 cameraProject;
    //Matrix4 cameraInverse;

    // TODO: check if camera has the right components
    ACompCamera& cameraComp = reg_get<ACompCamera>(camera);
    ACompTransform& cameraTransform = reg_get<ACompTransform>(camera);

    //cameraProject = cameraComp.m_projection;
    cameraComp.m_inverse = cameraTransform.m_transformWorld.inverted();

    m_renderOrder.call(cameraComp);
}

MapSysMachine::iterator ActiveScene::system_machine_find(const std::string &name)
{
//    MapSysMachine::iterator sysIt = m_sysMachines.find(name);
//    if (sysIt == m_sysMachines.end())
//    {
//        return nullptr;
//    }
//    else
//    {
//        return sysIt->second.get();
//    }
    return m_sysMachines.find(name);
}

bool ActiveScene::system_machine_it_valid(MapSysMachine::iterator it)
{
    return it != m_sysMachines.end();
}



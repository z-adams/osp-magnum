#pragma once

#include <memory>

#include <osp/Active/activetypes.h>
#include <osp/UserInputHandler.h>
#include <osp/types.h>
#include <osp/Universe.h>

namespace osp
{

class AbstractDebugObject
{
public:
    // DebugObject(ActiveScene &scene, ActiveEnt ent);
    virtual ~AbstractDebugObject() = default;


    //virtual void set_entity(ActiveEnt m_ent);
};

template <class Derived>
class DebugObject : public AbstractDebugObject
{
    friend Derived;
public:
    DebugObject(active::ActiveScene &scene, active::ActiveEnt ent) :
            m_scene(scene),
            m_ent(ent) {};
    virtual ~DebugObject() = default;

private:
    active::ActiveScene &m_scene;
    active::ActiveEnt m_ent;
};

struct CompDebugObject
{
    CompDebugObject(std::unique_ptr<AbstractDebugObject> ptr) :
        m_obj(std::move(ptr)) {}
    CompDebugObject(CompDebugObject&& move) = default;
    ~CompDebugObject() = default;
    CompDebugObject& operator=(CompDebugObject&& move) = default;
    std::unique_ptr<AbstractDebugObject> m_obj;
};


class DebugCameraController : public DebugObject<DebugCameraController>
{

public:
    DebugCameraController(active::ActiveScene &scene, active::ActiveEnt ent);
    ~DebugCameraController() = default;
    void update_vehicle_mod_pre();
    void update_physics_post();
    void view_orbit(universe::Satellite ent) { m_orbiting = ent; }
private:

    universe::Satellite m_orbiting;
    Vector3 m_orbitPos;
    float m_orbitDistance;

    active::UpdateOrderHandle m_updateVehicleModPre;
    active::UpdateOrderHandle m_updatePhysicsPost;

    UserInputHandler &m_userInput;
    // Mouse inputs
    MouseMovementHandle m_mouseMotion;
    ScrollInputHandle m_scrollInput;
    ButtonControlHandle m_rmb;
    // Keyboard inputs
    ButtonControlHandle m_switch;
    ButtonControlHandle m_switch_back;
};


//class SysDebugObject
//{
//public:
//    SysDebugObject(ActiveScene &scene);

//private:
//    ActiveScene &m_scene;
//};

}

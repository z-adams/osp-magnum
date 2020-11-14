#include "DebugObject2.h"

#include <osp/Active/ActiveScene.h>
#include <osp/Active/SysVehicle.h>
#include <osp/Active/physics.h>
#include <planet-a/Satellites/SatPlanet.h>

#include <adera/Machines/UserControl.h>
#include <osp/Active/SysDebugRender2.h>
#include <adera/SysMap.h>

using adera::active::machines::MachineUserControl;

using namespace osp;
using namespace osp::active;

using namespace Magnum::Math::Literals;

//DebugObject::DebugObject(ActiveScene &scene, ActiveEnt ent) :
//    m_scene(scene),
//    m_ent(ent)
//{

//}

//SysDebugObject::SysDebugObject(ActiveScene &scene) :
//        m_scene(scene)
//{

//}


DebugCameraController::DebugCameraController(active::ActiveScene &scene,
                                             active::ActiveEnt ent) :
        DebugObject(scene, ent),
        m_orbiting(static_cast<universe::Satellite>(1)),
        m_orbitPos(0, 0, 1),
        m_updatePhysicsPost(scene.get_update_order(), "dbg_cam", "physics", "",
                std::bind(&DebugCameraController::update_physics_post, this)),
        m_updateVehicleModPre(scene.get_update_order(), "dbg_cam_vmod", "", "vehicle_modification",
                std::bind(&DebugCameraController::update_vehicle_mod_pre, this)),
        m_userInput(scene.get_user_input()),
        m_mouseMotion(m_userInput.mouse_get()),
        m_scrollInput(m_userInput.scroll_get()),
        m_switch(m_userInput.config_get("game_switch")),
        m_switch_back(m_userInput.config_get("game_switch_back")),
        m_rmb(m_userInput.config_get("ui_rmb"))
{
    m_orbitDistance = 20.0f;
}

void DebugCameraController::update_vehicle_mod_pre()
{
    if (!m_scene.get_application().get_universe().get_reg().valid(m_orbiting))
    {
        return;
    }
}

void DebugCameraController::update_physics_post()
{
    auto& universeReg = m_scene.get_application().get_universe().get_reg();
    bool targetValid = universeReg.valid(m_orbiting);

    if (m_switch.triggered() || m_switch_back.triggered())
    {
        std::cout << "switch to new target\n";

        auto view = universeReg.view<planeta::universe::UCompPlanet>();
        auto it = view.find(m_orbiting);
        std::cout << "\nCurrent: " << static_cast<int>(*it) << "\n";
        std::cout << "View 1st: " << static_cast<int>(*(--view.end())) << "\n";

        if (m_switch.triggered())
        {
            if (it == view.end() || it == --view.end())
            {
                // no vehicle selected, or last vehicle is selected (loop around)
                m_orbiting = view.front();
            }
            else
            {
                // pick the next vehicle
                m_orbiting = *(++it);
                std::cout << "prev\n";
            }
        }
        else if (m_switch_back.triggered())
        {
            if (it == view.end() || it == view.begin())
            {
                // no vehicle selected, or last vehicle is selected (loop around)
                m_orbiting = view.back();
            }
            else
            {
                // pick the next vehicle
                m_orbiting = *(--it);
                std::cout << "next\n";
            }
        }
        //m_scene.dynamic_system_get<SysMap>("Map").set_focus(m_orbiting);
        targetValid = universeReg.valid(m_orbiting);
    }

    if (!targetValid)
    {
        return;
    }

    Matrix4 &xform = m_scene.reg_get<active::ACompTransform>(m_ent).m_transform;
    Vector3s const& v3sPos = universeReg.get<universe::UCompTransformTraj>(m_orbiting).m_position;
    Vector3 tgtPos = SysMap::universe_to_render_space(v3sPos);

    //float keyRotYaw = static_cast<float>(m_rt.trigger_hold() - m_lf.trigger_hold());
    //float keyRotPitch = static_cast<float>(m_dn.trigger_hold() - m_up.trigger_hold());

    Quaternion mcFish(Magnum::Math::IdentityInit);
    if (m_rmb.trigger_hold())// || keyRotYaw || keyRotPitch)
    {
        // 180 degrees per second
        auto keyRotDelta = 180.0_degf * m_scene.get_time_delta_fixed();

        float yaw = 0.0f; // keyRotYaw * static_cast<float>(keyRotDelta);
        float pitch = 0.0f; // keyRotPitch * static_cast<float>(keyRotDelta);
        if (m_rmb.trigger_hold())
        {
            yaw += static_cast<float>(-m_mouseMotion.dxSmooth());
            pitch += static_cast<float>(-m_mouseMotion.dySmooth());
        }

        // 100 degrees per step
        constexpr auto rotRate = 1.0_degf;

        // rotate it
        mcFish = Quaternion::rotation(yaw * rotRate, xform.up())
            * Quaternion::rotation(pitch * rotRate, xform.right());
    }

    Vector3 posRelative = xform.translation() - tgtPos;

    // set camera orbit distance
    constexpr float distSensitivity = 5e4f;
    m_orbitDistance += distSensitivity * static_cast<float>(-m_scrollInput.dy());
    
    // Clamp orbit distance to avoid producing a degenerate m_orbitPos vector
    constexpr float minDist = 2000.0f;
    if (m_orbitDistance < minDist) { m_orbitDistance = minDist; }

    m_orbitPos = m_orbitPos.normalized() * m_orbitDistance;
    m_orbitPos = mcFish.transformVector(m_orbitPos);

    xform.translation() = tgtPos + m_orbitPos;

    // look at target
    xform = Matrix4::lookAt(xform.translation(), tgtPos, xform[1].xyz());
}

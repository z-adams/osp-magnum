#include <iostream>

#include <osp/Active/ActiveScene.h>
#include <osp/Active/physics.h>


#include "Rocket.h"
#include "osp/Resource/AssetImporter.h"
#include "adera/SysExhaustPlume.h"
#include <Magnum/Trade/MeshData.h>

using namespace adera::active::machines;
using namespace osp::active;
using namespace osp;

MachineRocket::MachineRocket() :
        Machine(true),
        m_wiGimbal(this, "Gimbal"),
        m_wiIgnition(this, "Ignition"),
        m_wiThrottle(this, "Throttle"),
        m_rigidBody(entt::null)
{
    //m_enable = true;
}

MachineRocket::MachineRocket(MachineRocket&& move) :
        Machine(std::move(move)),
        m_wiGimbal(this, std::move(move.m_wiGimbal)),
        m_wiIgnition(this, std::move(move.m_wiIgnition)),
        m_wiThrottle(this, std::move(move.m_wiThrottle))
{
    //m_enable = true;
}


MachineRocket& MachineRocket::operator=(MachineRocket&& move)
{
    m_enable = move.m_enable;
    // TODO
    return *this;
}

void MachineRocket::propagate_output(WireOutput* output)
{

}

WireInput* MachineRocket::request_input(WireInPort port)
{
    return existing_inputs()[port];
}

WireOutput* MachineRocket::request_output(WireOutPort port)
{
    return existing_outputs()[port];
}

std::vector<WireInput*> MachineRocket::existing_inputs()
{
    return {&m_wiGimbal, &m_wiIgnition, &m_wiThrottle};
}

std::vector<WireOutput*> MachineRocket::existing_outputs()
{
    return {};
}

SysMachineRocket::SysMachineRocket(ActiveScene &scene) :
    SysMachine<SysMachineRocket, MachineRocket>(scene),
    m_physics(scene.get_system<SysPhysics>()),
    m_updatePhysics(scene.get_update_order(), "mach_rocket", "wire", "physics",
                    std::bind(&SysMachineRocket::update_physics, this))
{

}

//void SysMachineRocket::update_sensor()
//{
//}

void SysMachineRocket::update_physics()
{
    auto view = m_scene.get_registry().view<MachineRocket>();

    for (ActiveEnt ent : view)
    {
        auto &machine = view.get<MachineRocket>(ent);

        //if (!machine.m_enable)
        //{
        //    continue;
        //}

        //std::cout << "updating a rocket\n";

        ACompRigidBody *compRb;
        ACompTransform *compTf;

        if (m_scene.get_registry().valid(machine.m_rigidBody))
        {
            // Try to get the ACompRigidBody if valid
            compRb = m_scene.get_registry()
                            .try_get<ACompRigidBody>(machine.m_rigidBody);
            compTf = m_scene.get_registry()
                            .try_get<ACompTransform>(machine.m_rigidBody);
            if (!compRb || !compTf)
            {
                machine.m_rigidBody = entt::null;
                continue;
            }
        }
        else
        {
            // rocket's rigid body not set yet
            auto body = m_physics.find_rigidbody_ancestor(ent);

            if (body.second == nullptr)
            {
                std::cout << "no rigid body!\n";
                continue;
            }

            machine.m_rigidBody = body.first;
            compRb = body.second;
            compTf = m_scene.get_registry()
                    .try_get<ACompTransform>(body.first);
        }

        if (WireData *ignition = machine.m_wiIgnition.connected_value())
        {

        }

        using wiretype::Percent;

        if (WireData *throttle = machine.m_wiThrottle.connected_value())
        {
            Percent *percent = std::get_if<Percent>(throttle);

            float thrust = 10.0f; // temporary

            //std::cout << percent->m_value << "\n";

            m_physics.body_apply_force(*compRb, compTf->m_transform.backward()
                    * (percent->m_value * thrust));
        }



        // this is suppose to be gimbal, but for now it applies torque

        using wiretype::AttitudeControl;

        if (WireData *gimbal = machine.m_wiGimbal.connected_value())
        {
            AttitudeControl *attCtrl = std::get_if<AttitudeControl>(gimbal);

            Vector3 localTorque = compTf->m_transform
                    .transformVector(attCtrl->m_attitude);

            localTorque *= 3.0f; // arbitrary

            m_physics.body_apply_torque(*compRb, localTorque);
        }


    }
}

void SysMachineRocket::attach_plume_effect(ActiveEnt ent)
{
    using Magnum::Shaders::Phong;
    using Magnum::GL::Mesh;
    using Magnum::GL::Texture2D;
    using Magnum::Trade::ImageData2D;

    ActiveEnt plumeNode;

    auto findPlumeHandle = [&](ActiveEnt ent)
    {
        ACompHierarchy& node = m_scene.reg_get<ACompHierarchy>(ent);
        if (node.m_name.compare(0, 9, "fx_plume_") == 0)
        {
            plumeNode = ent;
            return false;  // terminate search
        }
        return true;
    };

    m_scene.hierarchy_traverse(ent, findPlumeHandle);

    if (plumeNode == entt::null)
    {
        std::cout << "ERROR: could not find plume anchor for MachineRocket "
            << static_cast<int>(ent) << "\n";
        return;
    }
    std::cout << "MachineRocket "
        << static_cast<int>(ent) << "'s associated plume: "
        << static_cast<int>(plumeNode) << "\n";

    // Get plume effect data
    Package& pkg = m_scene.get_application().debug_get_packges()[0];
    std::string plumeAnchorName = m_scene.reg_get<ACompHierarchy>(plumeNode).m_name;
    std::string effectName = plumeAnchorName.substr(3, plumeAnchorName.length() - 3);
    DependRes<PlumeEffectData> plumeEffect = pkg.get<PlumeEffectData>(effectName);
    if (plumeEffect.empty())
    {
        std::cout << "ERROR: couldn't find plume effect " << effectName << "!\n";
        return;
    }

    // Get plume mesh
    DependRes<Mesh> plumeMesh = pkg.get<Mesh>(plumeEffect->meshName);
    if (plumeMesh.empty())
    {
        plumeMesh = AssetImporter::compile_mesh(plumeEffect->meshName, pkg);
    }
    // Get plume tex (TEMPORARY)
    DependRes<Texture2D> n1024 = pkg.get<Texture2D>("OSPData/adera/noise1024.png");

    PlumeShader shader(*plumeEffect);
    std::vector<Texture2D*> textures{&(*n1024), &(*n1024)};

    ACompExhaustPlume& plumeComp = m_scene.reg_emplace<ACompExhaustPlume>(plumeNode,
        ent, std::move(shader));

    m_scene.reg_emplace<CompDrawableDebug>(plumeNode, &(*plumeMesh),
        textures, &plumeComp.m_shader);
    m_scene.reg_emplace<CompVisibleDebug>(plumeNode, false);
    m_scene.reg_emplace<CompTransparentDebug>(plumeNode, true);
}

Machine& SysMachineRocket::instantiate(ActiveEnt ent)
{
    attach_plume_effect(ent);
    return m_scene.reg_emplace<MachineRocket>(ent);
}

Machine& SysMachineRocket::get(ActiveEnt ent)
{
    return m_scene.reg_get<MachineRocket>(ent);//emplace(ent);
}

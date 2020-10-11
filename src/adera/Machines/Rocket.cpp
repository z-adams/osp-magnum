#include <iostream>

#include <osp/Active/ActiveScene.h>
#include <osp/Active/physics.h>


#include "Rocket.h"
#include "osp/Resource/AssetImporter.h"
#include "osp/Resource/blueprints.h"
#include "adera/SysExhaustPlume.h"
#include <Magnum/Trade/MeshData.h>

using namespace adera::active::machines;
using namespace osp::active;
using namespace osp;

MachineRocket::MachineRocket(Parameters params, fuel_list_t& resources) :
        Machine(true),
        m_wiGimbal(this, "Gimbal"),
        m_wiIgnition(this, "Ignition"),
        m_wiThrottle(this, "Throttle"),
        m_rigidBody(entt::null),
        m_params(params),
        m_resourceLines(std::move(resources)),
        m_lastPowerOutput(0.0f)
{
}

MachineRocket::MachineRocket(MachineRocket&& move) :
        Machine(std::move(move)),
        m_wiGimbal(this, std::move(move.m_wiGimbal)),
        m_wiIgnition(this, std::move(move.m_wiIgnition)),
        m_wiThrottle(this, std::move(move.m_wiThrottle)),
        m_params(std::move(move.m_params)),
        m_resourceLines(std::move(move.m_resourceLines)),
        m_lastPowerOutput(std::move(move.m_lastPowerOutput))
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
    /*std::vector<WireInput*> inputs;
    inputs.reserve(3 + m_resourceLines.size());

    inputs.insert(inputs.begin(), {&m_wiGimbal, &m_wiIgnition, &m_wiThrottle});
    for (auto& resource : m_resourceLines)
    {
        inputs.push_back(&resource.m_lineIn);
    }
    return inputs;*/
    return {&m_wiGimbal, &m_wiIgnition, &m_wiThrottle};
}

std::vector<WireOutput*> MachineRocket::existing_outputs()
{
    return {};
}

SysMachineRocket::SysMachineRocket(ActiveScene &scene) :
    SysMachine<SysMachineRocket, MachineRocket>(scene),
    m_physics(scene.get_system<SysPhysics>()),
    m_updatePhysics(scene.get_update_order(), "mach_rocket", "controls", "physics",
                    std::bind(&SysMachineRocket::update_physics, this))
{

}

//void SysMachineRocket::update_sensor()
//{
//}

void SysMachineRocket::update_physics()
{
    auto view = m_scene.get_registry().view<MachineRocket, ACompTransform>();

    for (ActiveEnt ent : view)
    {
        auto &machine = view.get<MachineRocket>(ent);

        machine.m_lastPowerOutput = 0.0f;  // Will be set later if engine is on

        // Check for nonzero throttle, continue otherwise

        WireData *throttle = machine.m_wiThrottle.connected_value();
        wiretype::Percent* throtPercent = nullptr;
        if (throttle)
        {
            using wiretype::Percent;
            throtPercent = std::get_if<Percent>(throttle);
            if (!(throtPercent->m_value > 0.0f))
            {
                continue;
            }
        }
        else
        {
            continue;
        }


        // Check for adequate resource inputs

        bool fail = false;
        for (auto& resource : machine.m_resourceLines)
        {
            MachineContainer* src = m_scene.reg_try_get<MachineContainer>(resource.m_sourceEnt);
            if (!src)
            {
                std::cout << "Error: no source found\n";
                fail = true;
                break;
            }
            if (!src->check_contents().m_quantity > 0)
            {
                fail = true;
                break;
            }
        }
        if (fail) { continue; }

        // Perform physics calculation

        auto physComps = m_physics.try_get_or_find_rigidbody_parent(ent, machine.m_rigidBody);
        if (!physComps.first || !physComps.second) { continue; }

        ACompRigidBody *compRb = physComps.first;
        ACompTransform *compTf = physComps.second;

        if (WireData *ignition = machine.m_wiIgnition.connected_value())
        {

        }

        float thrustMag = machine.m_params.m_maxThrust * throtPercent->m_value;

        Matrix4 relTransform{};
        m_physics.find_rigidbody_ancestor(ent, &relTransform);

        Vector3 thrustDir = relTransform.transformVector(Vector3{0.0f, 0.0f, 1.0f});
        Vector3 thrust = thrustMag * thrustDir;
        Vector3 location = relTransform.translation();

        Vector3 worldThrust = compTf->m_transform.transformVector(thrust);
        m_physics.body_apply_force(*compRb, worldThrust);

        Vector3 torque = Magnum::Math::cross(location, thrust);
        Vector3 worldTorque = compTf->m_transform.transformVector(torque);
        m_physics.body_apply_torque(*compRb, worldTorque);

        compRb->m_rigidbodyDirty = true;

        // Perform resource consumption calculation

        float massFlowRateTot = thrustMag / (9.81f * machine.m_params.m_specImpulse);
        for (auto const& resource : machine.m_resourceLines)
        {
            float massFlowRate = massFlowRateTot * resource.m_massRateFraction;
            float massFlow = massFlowRate * m_scene.get_time_delta_fixed();
            uint64_t required = resource.m_type->resource_quantity(massFlow);
            MachineContainer* src = m_scene.reg_try_get<MachineContainer>(resource.m_sourceEnt);
            uint64_t consumed = src->request_contents(required);
            std::cout << "consumed " << consumed << " units of fuel, "
                << src->check_contents().m_quantity << " remaining\n";
        }

        // Set output power level (for plume effect)
        // TODO: later, take into account low fuel pressure, bad mixture, etc.
        machine.m_lastPowerOutput = throtPercent->m_value;
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

    // Get plume shader (TEMPORARY)
    DependRes<PlumeShader> ps = pkg.get<PlumeShader>("plume_shader");

    PlumeShaderInstance shader(*plumeEffect, ps);
    shader.set_combustion_noise_tex(*n1024);
    shader.set_nozzle_noise_tex(*n1024);

    std::vector<Texture2D*> textures{&(*n1024), &(*n1024)};

    ACompExhaustPlume& plumeComp = m_scene.reg_emplace<ACompExhaustPlume>(plumeNode,
        ent);

    m_scene.reg_emplace<CompDrawableDebug>(plumeNode, &(*plumeMesh),
        textures, std::move(shader));
    m_scene.reg_emplace<CompVisibleDebug>(plumeNode, false);
    m_scene.reg_emplace<CompTransparentDebug>(plumeNode, true);
}

Machine& SysMachineRocket::instantiate(ActiveEnt ent, PrototypeMachine config,
    BlueprintMachine settings)
{
    // Read engine config
    MachineRocket::Parameters params;
    params.m_maxThrust = std::get<double>(config.m_config["thrust"]);
    params.m_specImpulse = std::get<double>(config.m_config["Isp"]);

    MachineRocket::fuel_list_t inputs;
    Package& pkg = m_scene.get_application().debug_get_packges()[0];
    DependRes<ShipResourceType> fuel = pkg.get<ShipResourceType>("fuel");
    if (fuel)
    {
        ActiveEnt fuelSource = static_cast<ActiveEnt>(13);
        MachineRocket::ResourceInput input = {std::move(fuel), 1.0f, fuelSource};
        inputs.push_back(std::move(input));
    }

    attach_plume_effect(ent);
    return m_scene.reg_emplace<MachineRocket>(ent, params, inputs);
}

Machine& SysMachineRocket::get(ActiveEnt ent)
{
    return m_scene.reg_get<MachineRocket>(ent);//emplace(ent);
}

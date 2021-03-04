/**
 * Open Space Program
 * Copyright © 2019-2020 Open Space Program Project
 *
 * MIT License
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include <iostream>

#include <osp/Active/ActiveScene.h>
#include <osp/Active/physics.h>
#include <osp/Active/SysDebugRender.h>

#include "Rocket.h"
#include "osp/Resource/AssetImporter.h"
#include "adera/Shaders/Phong.h"
#include "adera/Shaders/PlumeShader.h"
#include "osp/Resource/blueprints.h"
#include "osp/PhysicsConstants.h"
#include "adera/SysExhaustPlume.h"
#include "adera/Plume.h"
#include <Magnum/Trade/MeshData.h>
#include <Magnum/Math/Color.h>
#include <Magnum/Math/Matrix4.h>

using namespace adera::active::machines;
using namespace osp::active;
using namespace osp;

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
    std::vector<WireInput*> inputs;
    inputs.reserve(3 + m_resourceLines.size());

    inputs.insert(inputs.begin(), {&m_wiGimbal, &m_wiIgnition, &m_wiThrottle});
    for (auto& resource : m_resourceLines)
    {
        inputs.push_back(&resource.m_source);
    }
    return inputs;
}

std::vector<WireOutput*> MachineRocket::existing_outputs()
{
    return {};
}

SysMachineRocket::SysMachineRocket(ActiveScene &rScene)
    : SysMachine<SysMachineRocket, MachineRocket>(rScene)
    , m_updatePhysics(rScene.get_update_order(), "mach_rocket", "controls", "physics",
        [this](ActiveScene& rScene) { this->update_physics(rScene); })
{

}

//void SysMachineRocket::update_sensor()
//{
//}

void SysMachineRocket::update_physics(ActiveScene& rScene)
{
    auto view = m_scene.get_registry().view<MachineRocket, ACompTransform>();

    for (ActiveEnt ent : view)
    {
        auto &machine = view.get<MachineRocket>(ent);
        if (!machine.m_enable)
        {
            continue;
        }

        machine.m_powerOutput = 0.0f;  // Will be set later if engine is on

        // Check for nonzero throttle, continue otherwise
        WireData *pThrottle = machine.m_wiThrottle.connected_value();
        wiretype::Percent* pThrotPercent = nullptr;
        if (pThrottle != nullptr)
        {
            using wiretype::Percent;
            pThrotPercent = std::get_if<Percent>(pThrottle);
            if ((pThrotPercent == nullptr) || !(pThrotPercent->m_value > 0.0f))
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
            if (auto const* pipe = resource.m_source.get_if<wiretype::Pipe>();
                pipe != nullptr)
            {
                
                if (auto const* src = rScene.reg_try_get<MachineContainer>(pipe->m_source);
                    src != nullptr)
                {
                    uint64_t required = resource_units_required(rScene, machine,
                        pThrotPercent->m_value, resource);
                    if (src->check_contents().m_quantity > required)
                    {
                        continue;
                    }
                }
            }
            fail = true;
            break;
        }
        if (fail)
        {
            continue;
        }

        // Perform physics calculation

        // Get rigidbody ancestor and its transformation component
        auto const* pRbAncestor =
            SysPhysics_t::try_get_or_find_rigidbody_ancestor(rScene, ent);
        auto& rCompRb = rScene.reg_get<ACompRigidBody_t>(pRbAncestor->m_ancestor);
        auto const& rCompTf = rScene.reg_get<ACompTransform>(pRbAncestor->m_ancestor);

        if (WireData *ignition = machine.m_wiIgnition.connected_value())
        {

        }

        Matrix4 relTransform = pRbAncestor->m_relTransform;

        /* Compute thrust force
         * Thrust force is defined to be along -Z by convention. This choice was
         * made so that the "arrow" empty in Blender can be used to represent
         * MachineRockets, and the arrow points along the direction of exhaust
         * momentum.
         * Obtains thrust vector in rigidbody space
         */
        Vector3 thrustDir = relTransform.transformVector(Vector3{0.0f, 0.0f, -1.0f});
        float thrustMag = machine.m_params.m_maxThrust * pThrotPercent->m_value;
        // Take thrust in rigidbody space and apply to RB in world space
        Vector3 thrust = thrustMag * thrustDir;
        Vector3 worldThrust = rCompTf.m_transform.transformVector(thrust);
        SysPhysics_t::body_apply_force(rCompRb, worldThrust);

        // Obtain point where thrust is applied relative to RB CoM
        Vector3 location = relTransform.translation();
        // Compute worldspace torque from engine location, thrust vector
        Vector3 torque = Magnum::Math::cross(location, thrust);
        Vector3 worldTorque = rCompTf.m_transform.transformVector(torque);
        SysPhysics_t::body_apply_torque(rCompRb, worldTorque);
        
        rCompRb.m_inertiaDirty = true;

        // Perform resource consumption calculation
        for (MachineRocket::ResourceInput const& resource : machine.m_resourceLines)
        {
            // Pipe must be non-null since we checked earlier
            const auto& pipe = *resource.m_source.get_if<wiretype::Pipe>();
            auto& src = rScene.reg_get<MachineContainer>(pipe.m_source);

            uint64_t required = resource_units_required(rScene, machine,
                pThrotPercent->m_value, resource);
            uint64_t consumed = src.request_contents(required);
            std::cout << "consumed " << consumed << " units of fuel, "
                << src.check_contents().m_quantity << " remaining\n";
        }

        // Set output power level (for plume effect)
        // TODO: later, take into account low fuel pressure, bad mixture, etc.
        machine.m_powerOutput = pThrotPercent->m_value;
    }
}

Machine& SysMachineRocket::instantiate(ActiveEnt ent, PrototypeMachine config,
    BlueprintMachine settings)
{
    // Read engine config
    MachineRocket::Parameters params;
    params.m_maxThrust = std::get<double>(config.m_config["thrust"]);
    params.m_specImpulse = std::get<double>(config.m_config["Isp"]);

    auto& fuelIdent = std::get<std::vector<std::string>>(config.m_config["input_names"]);
    auto& massRates = std::get<std::vector<double>>(config.m_config["input_mass_rate_fractions"]);
    assert(fuelIdent.size() == massRates.size());

    // Compute normalization coefficient for fuel ratios
    double normalization = 0.0;
    for (double rate : massRates)
    {
        normalization += (rate*rate);
    }
    normalization = 1.0 / sqrt(normalization);

    std::vector<MachineRocket::input_t> inputs;
    for (size_t i = 0; i < fuelIdent.size(); i++)
    {
        std::string_view identifier = fuelIdent[i];
        double massRate = normalization * massRates[i];

        Path resPath = decompose_path(identifier);
        Package& pkg = m_scene.get_application().debug_find_package(resPath.prefix);
        DependRes<ShipResourceType> input = pkg.get<ShipResourceType>(resPath.identifier);
        if (!input.empty())
        {
            inputs.push_back({std::move(input), massRate});
        }
    }

    return m_scene.reg_emplace<MachineRocket>(ent, std::move(params), std::move(inputs));
}

Machine& SysMachineRocket::get(ActiveEnt ent)
{
    return m_scene.reg_get<MachineRocket>(ent);//emplace(ent);
}

uint64_t SysMachineRocket::resource_units_required(
    osp::active::ActiveScene const& scene,
    MachineRocket const& machine, float throttle,
    MachineRocket::ResourceInput const& resource)
{
    float massFlowRate = resource_mass_flow_rate(machine,
        throttle, resource);
    float massFlow = massFlowRate * scene.get_time_delta_fixed();
    return resource.m_type->resource_quantity(massFlow);
}

constexpr float SysMachineRocket::resource_mass_flow_rate(MachineRocket const& machine,
    float throttle, MachineRocket::ResourceInput const& resource)
{
    float thrustMag = machine.m_params.m_maxThrust * throttle;
    float massFlowRateTot = thrustMag /
        (phys::constants::g_0 * machine.m_params.m_specImpulse);

    return massFlowRateTot * resource.m_massRateFraction;
}

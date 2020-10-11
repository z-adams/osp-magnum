#pragma once
#include <utility>
#include <osp/Active/SysMachine.h>
#include <osp/Resource/blueprints.h>
#include "adera/ShipResources.h"

namespace adera::active::machines
{

class MachineRocket;


class SysMachineRocket :
        public osp::active::SysMachine<SysMachineRocket, MachineRocket>
{
public:

    SysMachineRocket(osp::active::ActiveScene &scene);

    //void update_sensor();
    void update_physics();

    /**
     * Attach a visual exhaust plume effect to MachineRocket
     *
     * Searches the hierarchy under the specified MachineRocket entity and
     * attaches a visual exhaust effect to the appropriate node
     * @param ent The MachineRocket entity
     */
    void attach_plume_effect(osp::active::ActiveEnt ent);

    /**
     * Attach a MachineRocket to an entity
     * 
     * Also attempts to attach a plume component to the appropriate child node
     * @param ent The entity that owns the MachineRocket
     * @return The new MachineRocket instance
     */
    osp::active::Machine& instantiate(osp::active::ActiveEnt ent,
        osp::PrototypeMachine config, osp::BlueprintMachine settings) override;

    osp::active::Machine& get(osp::active::ActiveEnt ent) override;

private:

    osp::active::SysPhysics &m_physics;
    osp::active::UpdateOrderHandle m_updatePhysics;
};

/**
 *
 */
class MachineRocket : public osp::active::Machine
{
    friend SysMachineRocket;
    struct ResourceInput
    {
        osp::DependRes<ShipResourceType> m_type;
        float m_massRateFraction;
        osp::active::ActiveEnt m_sourceEnt;
    };
    using fuel_list_t = std::vector<ResourceInput>;

    struct Parameters
    {
        float m_maxThrust;
        float m_specImpulse;
    };

public:
    MachineRocket(Parameters params, fuel_list_t& resources);
    MachineRocket(MachineRocket &&move);

    MachineRocket& operator=(MachineRocket&& move);


    ~MachineRocket() = default;

    void propagate_output(osp::active::WireOutput *output) override;

    osp::active::WireInput* request_input(osp::WireInPort port) override;
    osp::active::WireOutput* request_output(osp::WireOutPort port) override;

    std::vector<osp::active::WireInput*> existing_inputs() override;
    std::vector<osp::active::WireOutput*> existing_outputs() override;

    /**
     * Return normalized power output level of the rocket this frame
     *
     * Returns a value [0,1] corresponding to the current output power of the
     * engine. This value is equal to the throttle input level, unless the
     * engine has run out of fuel, has a nonlinear throttle response, or some
     * similar reason. Used primarily by SysExhaustPlume to determine what the
     * exhaust plume effect should look like.
     * @return normalized float [0,1] representing engine power output
     */
    float current_output_power() const
    { return m_lastPowerOutput; }

private:
    osp::active::WireInput m_wiGimbal;
    osp::active::WireInput m_wiIgnition;
    osp::active::WireInput m_wiThrottle;

    osp::active::ActiveEnt m_rigidBody;
    fuel_list_t m_resourceLines;
    Parameters m_params;
    float m_lastPowerOutput;
};


}

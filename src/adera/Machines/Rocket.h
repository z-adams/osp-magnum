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

using fuel_list_t = std::vector<std::pair<osp::DependRes<ShipResourceType>, float>>;

/**
 *
 */
class MachineRocket : public osp::active::Machine
{
    friend SysMachineRocket;

public:
    MachineRocket(float thrust, fuel_list_t consumptionRatios);
    MachineRocket(MachineRocket &&move);

    MachineRocket& operator=(MachineRocket&& move);


    ~MachineRocket() = default;

    void propagate_output(osp::active::WireOutput *output) override;

    osp::active::WireInput* request_input(osp::WireInPort port) override;
    osp::active::WireOutput* request_output(osp::WireOutPort port) override;

    std::vector<osp::active::WireInput*> existing_inputs() override;
    std::vector<osp::active::WireOutput*> existing_outputs() override;

private:
    osp::active::WireInput m_wiGimbal;
    osp::active::WireInput m_wiIgnition;
    osp::active::WireInput m_wiThrottle;

    struct ResourceInput
    {
        osp::DependRes<ShipResourceType> m_type;
        float m_massRateFraction;
        osp::active::WireInput m_lineIn;
    };

    osp::active::ActiveEnt m_rigidBody;
    float m_thrust;
    std::vector<ResourceInput> m_resourceLines;
};


}

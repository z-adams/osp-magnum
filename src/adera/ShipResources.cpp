#include "ShipResources.h"
#include "osp/Active/ActiveScene.h"

using namespace osp;
using namespace osp::active;
using namespace adera::active::machines;

/* MachineContainer */

MachineContainer::MachineContainer(float capacity, ShipResource resource) :
    Machine(true),
    m_capacity(capacity),
    m_contents(resource)
{
}

MachineContainer::MachineContainer(MachineContainer&& move) :
    Machine(std::move(move)),
    m_capacity(std::move(move.m_capacity)),
    m_contents(std::move(move.m_contents))
{
}

MachineContainer& MachineContainer::operator=(MachineContainer&& move)
{
    m_enable = move.m_enable;
    m_capacity = move.m_capacity;
    m_contents = std::move(move.m_contents);
}

void MachineContainer::propagate_output(WireOutput* output)
{

}

WireInput* MachineContainer::request_input(WireInPort port)
{
    return nullptr;
}

WireOutput* MachineContainer::request_output(WireOutPort port)
{
    return nullptr;
}

std::vector<WireInput*> MachineContainer::existing_inputs()
{
    return m_inputs;
}

std::vector<WireOutput*> MachineContainer::existing_outputs()
{
    return m_outputs;
}

/* SysMachineContainer */

SysMachineContainer::SysMachineContainer(ActiveScene& scene) :
    SysMachine<SysMachineContainer, MachineContainer>(scene),
    m_updateContainers(scene.get_update_order(), "mach_container", "", "mach_rocket",
        std::bind(&SysMachineContainer::update_containers, this))
{
}

void SysMachineContainer::update_containers()
{
    auto view = m_scene.get_registry().view<MachineContainer>();

    for (ActiveEnt ent : view)
    {
        auto& container = view.get<MachineContainer>(ent);

        // Do something useful here... or not
    }
}


float SysMachineContainer::resource_volume(ShipResourceType type, uint64_t quantity)
{
    uint64_t units = quantity / (1ull << type.m_quanta);
    return static_cast<float>(units) * type.m_volume;
}

float SysMachineContainer::resource_mass(ShipResourceType type, uint64_t quantity)
{
    uint64_t units = quantity / (1ull << type.m_quanta);
    return static_cast<float>(units) * type.m_mass;
}

uint64_t SysMachineContainer::resource_capacity(ShipResourceType type, float volume)
{
    double units = volume / type.m_volume;
    double quantaPerUnit = static_cast<double>(1ull << type.m_quanta);
    return static_cast<uint64_t>(units * quantaPerUnit);
}

Machine & SysMachineContainer::instantiate(ActiveEnt ent,
    PrototypeMachine config, BlueprintMachine settings)
{
    float capacity = std::get<float>(config.m_config["capacity"]);
    
    std::string resName = std::get<std::string>(settings.m_config["resourcename"]);
    Package& pkg = m_scene.get_application().debug_get_packges()[0];

    ShipResource resource;
    resource.m_type = pkg.get<ShipResourceType>(resName);
    resource.m_quantity = resource_capacity(*resource.m_type, capacity);

    return m_scene.reg_emplace<MachineContainer>(ent, capacity, resource);
}

Machine& SysMachineContainer::get(ActiveEnt ent)
{
    return m_scene.reg_get<MachineContainer>(ent);
}

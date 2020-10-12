#include "ShipResources.h"
#include "osp/Active/ActiveScene.h"
#include "osp/Resource/PrototypePart.h"

using namespace osp;
using namespace osp::active;
using namespace adera::active::machines;

/* ShipResourceType */

float ShipResourceType::resource_volume(uint64_t quantity) const
{
    uint64_t units = quantity / (1ull << m_quanta);
    return static_cast<float>(units) * m_volume;
}

float ShipResourceType::resource_mass(uint64_t quantity) const
{
    uint64_t units = quantity / (1ull << m_quanta);
    return static_cast<float>(units) * m_mass;
}

uint64_t ShipResourceType::resource_capacity(float volume) const
{
    double units = volume / m_volume;
    double quantaPerUnit = static_cast<double>(1ull << m_quanta);
    return static_cast<uint64_t>(units * quantaPerUnit);
}

uint64_t ShipResourceType::resource_quantity(float mass) const
{
    double units = mass / m_mass;
    double quantaPerUnit = static_cast<double>(1ull << m_quanta);
    return static_cast<uint64_t>(units * quantaPerUnit);
}

/* MachineContainer */

MachineContainer::MachineContainer(ActiveEnt ownID, float capacity, ShipResource resource) :
    Machine(true),
    m_capacity(capacity),
    m_contents(resource),
    m_outputs(this, "output")
{
    m_outputs.value() = wiretype::Pipe{ownID};
}

MachineContainer::MachineContainer(MachineContainer&& move) :
    Machine(std::move(move)),
    m_capacity(std::move(move.m_capacity)),
    m_contents(std::move(move.m_contents)),
    m_outputs(this, std::move(move.m_outputs))
{
}

MachineContainer& MachineContainer::operator=(MachineContainer&& move)
{
    m_enable = move.m_enable;
    m_capacity = move.m_capacity;
    m_contents = std::move(move.m_contents);
    return *this;
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
    return &m_outputs;
}

std::vector<WireInput*> MachineContainer::existing_inputs()
{
    return {};
}

std::vector<WireOutput*> MachineContainer::existing_outputs()
{
    return {&m_outputs};
}

uint64_t MachineContainer::request_contents(uint64_t quantity)
{
    if (quantity > m_contents.m_quantity)
    {
        uint64_t remainder = m_contents.m_quantity;
        m_contents.m_quantity = 0;
        return remainder;
    }

    m_contents.m_quantity -= quantity;
    return quantity;
}

float MachineContainer::current_mass() const
{
    if (m_contents.m_type.empty()) { return 0.0f; }
    return m_contents.m_type->resource_mass(m_contents.m_quantity);
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
    auto view = m_scene.get_registry().view<MachineContainer, ACompMass>();

    for (ActiveEnt ent : view)
    {
        auto& container = view.get<MachineContainer>(ent);
        auto& mass = view.get<ACompMass>(ent);

        mass.m_mass = container.current_mass();
    }
}

Machine& SysMachineContainer::instantiate(ActiveEnt ent,
    PrototypeMachine config, BlueprintMachine settings)
{
    float capacity = std::get<double>(config.m_config["capacity"]);
    
    ShipResource resource{};
    if (settings.m_config.find("resourcename") != settings.m_config.end())
    {
        std::string resName = std::get<std::string>(settings.m_config["resourcename"]);
        Package& pkg = m_scene.get_application().debug_get_packges()[0];

        resource.m_type = pkg.get<ShipResourceType>(resName);
        double fuelLevel = std::get<double>(settings.m_config["fuellevel"]);
        resource.m_quantity = resource.m_type->resource_capacity(fuelLevel*capacity);
    }

    m_scene.reg_emplace<ACompMass>(ent, 0.0f);
    m_scene.reg_emplace<ACompContainerShape>(ent, ECollisionShape::CYLINDER);

    return m_scene.reg_emplace<MachineContainer>(ent, ent, capacity, resource);
}

Machine& SysMachineContainer::get(ActiveEnt ent)
{
    return m_scene.reg_get<MachineContainer>(ent);
}

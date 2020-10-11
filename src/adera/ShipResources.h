#pragma once
#include <string>

#include "osp/Resource/Resource.h"
#include "osp/Active/SysMachine.h"

/* Sample resource:

identifier: lox
name: Liquid Oxygen
quanta: 16
mass: 1141.0 (kg)
volume: 1.0 (m^3)

*/
namespace adera::active::machines
{

/**
 * Represents a type of consumable ship resource
 *
 * Resources might be quantized differently depending on the rate at which
 * they're consumed. This system allows this quantization to be configured
 * per-resource. Resources will be represented by 64-bit integers to maximize
 * the available precision to avoid infinite-fuel exploits.
 *
 * The 64-bit range is divided into two sections by an arbitrary choice of unit;
 * the "quanta" value specifies the number of quanta that represents one unit
 * of the resource. One unit of the resource is divided into (2^quanta) pieces,
 * making 1/(2^quanta) the smallest representable unit of the resource. The
 * remaining 64-quanta bits of precision are left over for tank capacity.
 */
struct ShipResourceType
{
    // A short, unique, identifying name readable by both human and machine
    std::string m_identifier;

    // The full, screen-display name of the resource
    std::string m_displayName;

    // 1/(2^quanta) is the smallest representable quantity of this resource
    uint8_t m_quanta;

    // The mass (in kg, for now) of 2^quanta units of this resource
    float m_mass;

    // The volume (in m^3, for now) of 2^quanta units of this resource
    float m_volume;

    // The density (kg/m^3, for now) of 2^quanta units of this resource
    float m_density;

    float resource_volume(uint64_t quantity) const;
    float resource_mass(uint64_t quantity) const;
    uint64_t resource_capacity(float volume) const;
    uint64_t resource_quantity(float mass) const;
};

struct ShipResource
{
    osp::DependRes<ShipResourceType> m_type;
    uint64_t m_quantity;
};

class MachineContainer;

class SysMachineContainer
    : public osp::active::SysMachine<SysMachineContainer, MachineContainer>
{
public:
    SysMachineContainer(osp::active::ActiveScene& scene);

    void update_containers();

    osp::active::Machine& instantiate(
        osp::active::ActiveEnt ent,
        osp::PrototypeMachine config,
        osp::BlueprintMachine settings) override;

    osp::active::Machine& get(osp::active::ActiveEnt ent) override;

private:
    
    osp::active::UpdateOrderHandle m_updateContainers;
};

class MachineContainer : public osp::active::Machine
{
    friend SysMachineContainer;

public:
    MachineContainer(float capacity, ShipResource resource);
    MachineContainer(MachineContainer&& move);
    MachineContainer& operator=(MachineContainer&& move);

    ~MachineContainer() = default;

    void propagate_output(osp::active::WireOutput *output) override;

    osp::active::WireInput* request_input(osp::WireInPort port) override;
    osp::active::WireOutput* request_output(osp::WireOutPort port) override;

    std::vector<osp::active::WireInput*> existing_inputs() override;
    std::vector<osp::active::WireOutput*> existing_outputs() override;

    constexpr const ShipResource& check_contents() const
    { return m_contents; }
    
    /**
     * Request a quantity of the contained resource
     *
     * Since the resources are stored as unsigned integers, avoiding wraparound
     * is crucial. This function wraps the resource withdrawal process by
     * internally checking the requested quantity of resource, bounds checking
     * it, and only returning as much resources as are available.
     * @param quantity [in] The requested quantity of resource
     * @return The amount of resource that was received
     */
    uint64_t request_contents(uint64_t quantity);
private:
    std::vector<osp::active::WireInput*> m_inputs;
    std::vector<osp::active::WireOutput*> m_outputs;

    float m_capacity;
    ShipResource m_contents;
};

}
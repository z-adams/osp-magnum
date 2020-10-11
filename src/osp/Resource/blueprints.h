#pragma once

#include "../types.h"

#include "Resource.h"
#include "PrototypePart.h"

namespace osp
{

using WireInPort = uint16_t;
using WireOutPort = uint16_t;

struct BlueprintMachine
{
    // TODO specific settings for a machine
    std::map<std::string, config_node_t> m_config;
};

struct BlueprintObject
{
    std::vector<BlueprintMachine> m_machines;
};

/**
 * Specific information on a part in a vehicle:
 * * Which kind of part
 * * Enabled/disabled properties
 * * Transformation
 */
struct BlueprintPart
{ 

    unsigned m_partIndex; // index to BlueprintVehicle's m_partsUsed

    Vector3 m_translation;
    Quaternion m_rotation;
    Vector3 m_scale;

    // put some sort of config here
    std::vector<BlueprintObject> m_objects;
};

/**
 * Describes a "from output -> to input" wire connection
 *
 * [  machine  out]--->[in  other machine  ]
 */
struct BlueprintWire
{
    BlueprintWire(unsigned fromPart, unsigned fromMachine, WireOutPort fromPort,
                  unsigned toPart, unsigned toMachine, WireOutPort toPort) :
        m_fromPart(fromPart),
        m_fromMachine(fromMachine),
        m_fromPort(fromPort),
        m_toPart(toPart),
        m_toMachine(toMachine),
        m_toPort(toPort)
    {}

    unsigned m_fromPart;
    unsigned m_fromMachine;
    WireOutPort m_fromPort;

    unsigned m_toPart;
    unsigned m_toMachine;
    WireInPort m_toPort;
};

/**
 * Specific information on a vehicle
 * * List of part blueprints
 * * Attachments
 * * Wiring
 */
class BlueprintVehicle
{

public:

    BlueprintVehicle() = default;
    BlueprintVehicle(BlueprintVehicle&& move) = default;

    /**
     * Emplaces a BlueprintPart. This function searches the prototype vector
     * to see if the part has been added before.
     * @param part
     * @param translation
     * @param rotation
     * @param scale
     * @return Resulting blueprint part
     */
    BlueprintPart& add_part(DependRes<PrototypePart>& part,
                  const Vector3& translation,
                  const Quaternion& rotation,
                  const Vector3& scale);

    /**
     * Emplace a BlueprintWire
     * @param fromPart
     * @param fromMachine
     * @param fromPort
     * @param toPart
     * @param toMachine
     * @param toPort
     */
    void add_wire(unsigned fromPart, unsigned fromMachine, WireOutPort fromPort,
                  unsigned toPart, unsigned toMachine, WireInPort toPort);

    constexpr std::vector<DependRes<PrototypePart>>& get_prototypes()
    { return m_prototypes; }

    constexpr std::vector<BlueprintPart>& get_blueprints()
    { return m_blueprints; }

    constexpr std::vector<BlueprintWire>& get_wires()
    { return m_wires; }

private:
    // Unique part Resources used
    std::vector<DependRes<PrototypePart>> m_prototypes;

    // Arrangement of Individual Parts
    std::vector<BlueprintPart> m_blueprints;

    // Wires to connect
    std::vector<BlueprintWire> m_wires;

};

}

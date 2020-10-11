#include "blueprints.h"


using namespace osp;

BlueprintPart& BlueprintVehicle::add_part(
        DependRes<PrototypePart>& prototype,
        const Vector3& translation,
        const Quaternion& rotation,
        const Vector3& scale)
{
    unsigned partIndex = m_prototypes.size();

    // check if the part is added already.
    for (unsigned i = 0; i < partIndex; i ++)
    {
        const DependRes<PrototypePart>& dep = m_prototypes[i];

        if (dep == prototype)
        {
            // prototype was already added to the list
            partIndex = i;
            break;
        }
    }

    if (partIndex == m_prototypes.size())
    {
        // Add the unlisted prototype to the end
        m_prototypes.emplace_back(prototype);
    }

    // now we have a part index.

    // Create and default initialize object blueprint machines
    size_t numMachines = prototype->get_machines().size();
    std::vector<BlueprintMachine> machineBPs(numMachines);

    BlueprintPart blueprint
    {
        partIndex,
        translation, rotation, scale,
        std::move(machineBPs)
    };

    m_blueprints.push_back(std::move(blueprint));

    return m_blueprints.back();
}

void BlueprintVehicle::add_wire(
        unsigned fromPart, unsigned fromMachine, WireOutPort fromPort,
        unsigned toPart, unsigned toMachine, WireInPort toPort)
{
    m_wires.emplace_back(fromPart, fromMachine, fromPort,
                         toPart, toMachine, toPort);
}

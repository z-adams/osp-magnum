#pragma once

#include "activetypes.h"

#include "SysAreaAssociate.h"

#include "../Universe.h"
#include "../Resource/Package.h"
#include "../Resource/blueprints.h"

#include <Magnum/Shaders/Phong.h>

namespace osp
{
class PrototypePart;
}

namespace osp::universe
{
class SatActiveArea;
}

namespace osp::active
{

struct WireMachineConnection
{
    ActiveEnt m_fromPartEnt;
    unsigned m_fromMachine;
    WireOutPort m_fromPort;

    ActiveEnt m_toPartEnt;
    unsigned m_toMachine;
    WireInPort m_toPort;
};

struct ACompVehicle
{
    std::vector<ActiveEnt> m_parts;
    //entt::sparse_set<ActiveEnt> m_parts;

    // index to 'main part' in m_parts. if the vehicle separates into multiple
    // vehicles, then the resulting vehicle containing the main part is the
    // original vehicle.
    unsigned m_mainPart{0};

    // set this if vehicle is modified:
    // * 0: nothing happened
    // * 1: something exploded (m_destroy set on some parts), but vehicle
    //      isn't split into peices
    // * 2+: number of separation islands
    unsigned m_separationCount{0};
};

struct ACompPart
{
    ActiveEnt m_vehicle{entt::null};

    // set this to true if this part is to be destroyed on the vehicle
    // modification update. also set m_separationCount on ACompVehicle
    bool m_destroy{false};

    // if vehicle separates into more vehicles, then set this to
    // something else on parts that are separated together.
    // actual separation happens in update_vehicle_modification
    unsigned m_separationIsland{0};
};

class SysVehicle : public IDynamicSystem, public IActivator
{
public:

    SysVehicle(ActiveScene &scene);
    SysVehicle(SysNewton const& copy) = delete;
    SysVehicle(SysNewton&& move) = delete;
    ~SysVehicle() = default;

    //static int area_activate_vehicle(ActiveScene& scene,
    //                                 SysAreaAssociate &area,
    //                                 universe::Satellite areaSat,
    //                                 universe::Satellite loadMe);
    StatusActivated activate_sat(ActiveScene &scene, SysAreaAssociate &area,
            universe::Satellite areaSat, universe::Satellite tgtSat);
    int deactivate_sat(ActiveScene &scene, SysAreaAssociate &area,
            universe::Satellite areaSat, universe::Satellite tgtSat,
            ActiveEnt tgtEnt);

    /**
     * Add machines to the specified entity
     *
     * Accepts a list of prototype machines and their associated blueprint
     * machines, and adds them to the specified entity
     * @param entity [in] The target entity
     * @param protoMachines [in] The prototype machines to add to the entity
     * @param blueprintMachines [in] The associated configs for the machines
     */
    void create_machines(ActiveScene& scene, ActiveEnt entity,
        std::vector<PrototypeMachine> const& protoMachines,
        std::vector<BlueprintMachine> const& blueprintMachines);

    // Stores all the information necessary to instantiate a machine
    using machine_def_t = std::tuple<
        ActiveEnt,
        std::vector<PrototypeMachine> const&,
        std::vector<BlueprintMachine> const&>;

    /**
     * Adds all machines to a part at once
     *
     * Machine instantiation requires the part's hierarchy to already exist
     * in case a sub-object needs information about its peers. Since this means
     * machines can't be instantiated alongside their associated object, the
     * information about each machine is captured in a machine_def_t, then used
     * to call this function after all part children exist to instance all the
     * machines at once.
     * @param definitions [in] List of machine configs and their associated ents
     */
    void add_machines_deferred(std::vector<machine_def_t> definitions);

    /**
     * Create a Physical Part from a PrototypePart and put it in the world
     * @param part [in] The part prototype to instantiate
     * @param blueprint [in] Unique part configuration data
     * @param rootParent [in] Entity to put part into
     * @param machineDefinitions [out] List of part machines for later creation
     * @return Pointer to object created
     */
    ActiveEnt part_instantiate(PrototypePart& part, BlueprintPart& blueprint,
        ActiveEnt rootParent, std::vector<machine_def_t>& machineDefinitions);

    // Handle deleted parts and separations
    void update_vehicle_modification();

private:
    ActiveScene& m_scene;
    //AppPackages& m_packages;

    // temporary
    std::unique_ptr<Magnum::Shaders::Phong> m_shader;

    UpdateOrderHandle m_updateVehicleModification;
};


}

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

#include "vehicles.h"

#include <osp/Satellites/SatVehicle.h>
#include <osp/string_concat.h>

using namespace testapp;

using osp::Vector2;
using osp::Vector3;
using osp::Matrix4;
using osp::Quaternion;

using osp::Package;
using osp::DependRes;

using osp::universe::Universe;
using osp::universe::Satellite;
using osp::universe::SatVehicle;
using osp::universe::UCompTransformTraj;
using osp::universe::UCompVehicle;

using osp::BlueprintVehicle;
using osp::PrototypePart;

/**
 * Utility: computes the displacement between two parts, relative to the
 * specified sub-object (e.g. an attachment node).
 *
 * @param attachTo [in] The parent part
 * @param attachToName [in] The name of the parent's object/attach point
 * @param toAttach [in] The child part
 * @param toAttachName [in] The name of the child's object/attach point
 */
Vector3 part_offset(
    PrototypePart const& attachTo, std::string_view attachToName,
    PrototypePart const& toAttach, std::string_view toAttachName);

size_t find_machine_index(BlueprintVehicle const& vehicle,
    size_t partIndex, std::string_view objectName, std::string_view machineType);

std::map<std::string, osp::config_node_t>& get_config(
    osp::BlueprintVehicle const& vehicle, osp::BlueprintPart& part,
    std::string_view object, std::string_view machineType);

/**
 * Utility: creates wire coonnections based on named part elements
 * 
 * @param fromPart [in] The source part
 * @param fromObj [in] The name of the object which owns the source machine
 * @param fromMachine [in] The type of machine to look for
 * @param fromPort [in] The port from the selected machine to connect
 * @param toPart [in] The sink part
 * @param toObj [in] The name of the object which owns the sink machine
 * @param toMachine [in] The type of machine to look for
 * @param toPort [in] The port from the selected machine to connect
 */
void add_wire(osp::BlueprintVehicle const& vehicle,
    osp::BlueprintPart const& fromPart,
    std::string_view fromObj, std::string_view fromMachine, size_t fromPort,
    osp::BlueprintPart const& toPart,
    std::string_view toObj, std::string_view toMachine, size_t toPort);

osp::universe::Satellite testapp::debug_add_deterministic_vehicle(
        Universe& uni, Package& pkg, std::string_view name)
{
    // Begin blueprint
    BlueprintVehicle blueprint;

    // Part to add
    DependRes<PrototypePart> rocket = pkg.get<PrototypePart>("part_stomper");
    blueprint.add_part(rocket, Vector3(0.0f), Quaternion(), Vector3(1.0f));

    // Wire throttle control
    // from (output): a MachineUserControl m_woThrottle
    // to    (input): a MachineRocket m_wiThrottle
    blueprint.add_wire(0, 0, 1,
        0, 1, 2);

    // Wire attitude control to gimbal
    // from (output): a MachineUserControl m_woAttitude
    // to    (input): a MachineRocket m_wiGimbal
    blueprint.add_wire(0, 0, 0,
        0, 1, 0);

    // Save blueprint
    DependRes<BlueprintVehicle> depend =
        pkg.add<BlueprintVehicle>(std::string{name}, std::move(blueprint));

    // Create new satellite
    Satellite sat = uni.sat_create();

    // Set name
    auto& posTraj = uni.get_reg().get<UCompTransformTraj>(sat);
    posTraj.m_name = name;

    // Make the satellite into a vehicle
    SatVehicle::add_vehicle(uni, sat, std::move(depend));

    return sat;
}

osp::universe::Satellite testapp::debug_add_random_vehicle(
        osp::universe::Universe& uni, osp::Package& pkg,
        std::string_view name)
{

    // Start making the blueprint
    BlueprintVehicle blueprint;

    // Part to add, very likely a spamcan
    DependRes<PrototypePart> victim = pkg.get<PrototypePart>("part_spamcan");

    // Add 12 parts
    for (int i = 0; i < 12; i ++)
    {
        // Generate random vector
        Vector3 randomvec(std::rand() % 64 - 32,
                          std::rand() % 64 - 32,
                          (i - 6) * 12);

        randomvec /= 64.0f;

        // Add a new [victim] part
        blueprint.add_part(victim, randomvec,
                           Quaternion(), Vector3(1, 1, 1));
        //std::cout << "random: " <<  << "\n";
    }

    // Wire throttle control
    // from (output): a MachineUserControl m_woThrottle
    // to    (input): a MachineRocket m_wiThrottle
    blueprint.add_wire(0, 0, 1,
                       0, 1, 2);

    // Wire attitude control to gimbal
    // from (output): a MachineUserControl m_woAttitude
    // to    (input): a MachineRocket m_wiGimbal
    blueprint.add_wire(0, 0, 0,
                       0, 1, 0);

    // put blueprint in package
    DependRes<BlueprintVehicle> depend =
        pkg.add<BlueprintVehicle>(std::string{name}, std::move(blueprint));

    // Create the Satellite containing a SatVehicle

    // Create blank satellite
    Satellite sat = uni.sat_create();

    // Set the name
    auto &posTraj = uni.get_reg().get<UCompTransformTraj>(sat);
    posTraj.m_name = name;

    // Make the satellite into a vehicle
    SatVehicle::add_vehicle(uni, sat, std::move(depend));

    return sat;

}

Vector3 part_offset(PrototypePart const& attachTo,
    std::string_view attachToName, PrototypePart const& toAttach,
    std::string_view toAttachName)
{
    Vector3 oset1{0.0f};
    Vector3 oset2{0.0f};

    for (osp::PrototypeObject const& obj : attachTo.get_objects())
    {
        if (obj.m_name == attachToName)
        {
            oset1 = obj.m_translation;
            break;
        }
    }

    for (osp::PrototypeObject const& obj : toAttach.get_objects())
    {
        if (obj.m_name == toAttachName)
        {
            oset2 = obj.m_translation;
            break;
        }
    }

    return oset1 - oset2;
}

size_t find_machine_index(BlueprintVehicle const& vehicle,
    size_t partIndex, std::string_view objectName, std::string_view machineType)
{
    using osp::PrototypeMachine;
    using osp::PrototypeObject;
    using osp::PrototypePart;

    PrototypePart const& part = *vehicle.get_prototypes()[partIndex];
    std::vector<PrototypeObject> const& objects = part.get_objects();
    auto foundObject = std::find_if(objects.begin(), objects.end(),
        [objectName](std::vector<PrototypeObject>::iterator::value_type const& obj) -> bool
        {
            if (obj.m_name.compare(objectName) == 0) { return true; }
            return false;
        });

    if (foundObject == objects.end())
    {
        throw std::runtime_error("Couldn't find object");
    }
    
    std::vector<PrototypeMachine> const& objMachines = part.get_machines();
    auto foundMachine = std::find_if(
        foundObject->m_machineIndices.begin(),
        foundObject->m_machineIndices.end(),
        [machineType, &part](auto const index) -> bool
        {
            if (part.get_machines()[index].m_type.compare(machineType) == 0)
            {
                return true;
            }
            return false;
        });

    if (foundMachine == foundObject->m_machineIndices.end())
    {
        throw std::runtime_error("Couldn't find machine");
    }
    return *foundMachine;
}

std::map<std::string, osp::config_node_t>& get_config(
    osp::BlueprintVehicle const& vehicle, osp::BlueprintPart& part,
    std::string_view object, std::string_view machineType)
{
    size_t machineIndex = find_machine_index(vehicle, part.m_partIndex, object, machineType);
    return part.m_machines[machineIndex].m_config;
}

void add_wire(osp::BlueprintVehicle& vehicle,
    osp::BlueprintPart const& fromPartBlueprint,
    std::string_view fromObjName, std::string_view fromMachineName, size_t fromPort,
    osp::BlueprintPart const& toPartBlueprint,
    std::string_view toObjName, std::string_view toMachineName, size_t toPort)
{
    using osp::BlueprintPart;
    using osp::PrototypeObject;    
    using osp::PrototypeMachine;

    try
    {
        size_t fromPartBlueprintIndex = fromPartBlueprint.m_blueprintPartIndex;
        size_t fromPartIndex = fromPartBlueprint.m_partIndex;
        size_t fromMachineIndex =
            find_machine_index(vehicle, fromPartIndex, fromObjName, fromMachineName);

        size_t toPartBlueprintIndex = toPartBlueprint.m_blueprintPartIndex;
        size_t toPartIndex = toPartBlueprint.m_partIndex;
        size_t toMachineIndex =
            find_machine_index(vehicle, toPartIndex, toObjName, toMachineName);

        vehicle.add_wire(
            fromPartBlueprintIndex, fromMachineIndex, fromPort,
            toPartBlueprintIndex, toMachineIndex, toPort);

    } catch (std::exception const& e)
    {
        std::cout << "ERROR connecting " << fromPartBlueprint.m_blueprintPartIndex
            << ":" << fromObjName << ":" << fromMachineName << ":" << fromPort
            << " to " << toPartBlueprint.m_blueprintPartIndex
            << ":" << toObjName << ":" << toMachineName << ":" << toPort << "\n";
        e.what();
    }
}

osp::universe::Satellite testapp::debug_add_part_vehicle(
    osp::universe::Universe& uni, osp::Package& pkg,
    std::string_view name)
{
    using namespace Magnum::Math::Literals;

    // Start making the blueprint
    BlueprintVehicle blueprint;

    // Parts
    DependRes<PrototypePart> capsule = pkg.get<PrototypePart>("part_phCapsule");
    DependRes<PrototypePart> fuselage = pkg.get<PrototypePart>("part_phFuselage");
    DependRes<PrototypePart> engine = pkg.get<PrototypePart>("part_phEngine");
    DependRes<PrototypePart> rcs = pkg.get<PrototypePart>("part_phLinRCS");

    Vector3 cfOset = part_offset(*capsule, "attach_bottom_capsule",
        *fuselage, "attach_top_fuselage");
    Vector3 feOset = part_offset(*fuselage, "attach_bottom_fuselage",
        *engine, "attach_top_eng");
    Vector3 rcsOsetTop = cfOset + Vector3{1.0f, 0.0f, 2.0f};
    Vector3 rcsOsetBtm = cfOset + Vector3{1.0f, 0.0f, -2.0f};

    Quaternion idRot;
    Vector3 scl{1};
    Magnum::Rad qtrTurn(-90.0_degf);
    Quaternion rotY_090 = Quaternion::rotation(qtrTurn, Vector3{0, 1, 0});
    Quaternion rotZ_090 = Quaternion::rotation(qtrTurn, Vector3{0, 0, 1});
    Quaternion rotZ_180 = Quaternion::rotation(2 * qtrTurn, Vector3{0, 0, 1});
    Quaternion rotZ_270 = Quaternion::rotation(3 * qtrTurn, Vector3{0, 0, 1});

    blueprint.add_part(capsule, Vector3{0}, idRot, scl);

    auto& fuselageBP = blueprint.add_part(fuselage, cfOset, idRot, scl);
    fuselageBP.m_machines[1].m_config.emplace("resourcename", "lzdb:fuel");
    fuselageBP.m_machines[1].m_config.emplace("fuellevel", 0.5);

    auto& engBP = blueprint.add_part(engine, cfOset + feOset, idRot, scl);

    Quaternion yz090 = rotZ_090 * rotY_090;
    Quaternion yz180 = rotZ_180 * rotY_090;
    Quaternion yz270 = rotZ_270 * rotY_090;

    // Top RCS ring
    blueprint.add_part(rcs, rcsOsetTop, rotY_090, scl);
    blueprint.add_part(rcs, rotZ_090.transformVector(rcsOsetTop), yz090, scl);
    blueprint.add_part(rcs, rotZ_180.transformVector(rcsOsetTop), yz180, scl);
    blueprint.add_part(rcs, rotZ_270.transformVector(rcsOsetTop), yz270, scl);

    // Top RCS ring
    blueprint.add_part(rcs, rcsOsetBtm, rotY_090, scl);
    blueprint.add_part(rcs, rotZ_090.transformVector(rcsOsetBtm), yz090, scl);
    blueprint.add_part(rcs, rotZ_180.transformVector(rcsOsetBtm), yz180, scl);
    blueprint.add_part(rcs, rotZ_270.transformVector(rcsOsetBtm), yz270, scl);

    enum Parts
    {
        CAPSULE = 0,
        FUSELAGE = 1,
        ENGINE = 2,
        RCS1 = 3,
        RCS2 = 4,
        RCS3 = 5,
        RCS4 = 6,
        RCS5 = 7,
        RCS6 = 8,
        RCS7 = 9,
        RCS8 = 10
    };

    std::vector<int> rcsPorts = {RCS1, RCS2, RCS3, RCS4, RCS5, RCS6, RCS7, RCS8};

    // Wire throttle control
    // from (output): a MachineUserControl m_woThrottle
    // to    (input): a MachineRocket m_wiThrottle
    blueprint.add_wire(
        Parts::CAPSULE, 0, 1,
        Parts::ENGINE, 0, 2);

    // Wire attitude contrl to gimbal
    // from (output): a MachineUserControl m_woAttitude
    // to    (input): a MachineRocket m_wiGimbal
    blueprint.add_wire(
        Parts::CAPSULE, 0, 0,
        Parts::ENGINE, 0, 0);

    // Pipe fuel tank to rocket engine
    // from (output): fuselage MachineContainer m_outputs;
    // to    (input): entine MachineRocket m_resourcesLines[0]
    blueprint.add_wire(Parts::FUSELAGE, 0, 0,
        Parts::ENGINE, 0, 3);

    for (auto port : rcsPorts)
    {
        // Attitude control -> RCS Control
        blueprint.add_wire(Parts::CAPSULE, 0, 0,
            port, 0, 0);
        // RCS Control -> RCS Rocket
        blueprint.add_wire(port, 0, 0,
            port, 1, 2);
        // Fuselage tank -> RCS Rocket
        blueprint.add_wire(Parts::FUSELAGE, 0, 0,
            port, 1, 3);
    }

    // Put blueprint in package
    auto depend = pkg.add<BlueprintVehicle>(std::string{name}, std::move(blueprint));

    Satellite sat = uni.sat_create();

    // Set the name
    auto& posTraj = uni.get_reg().get<osp::universe::UCompTransformTraj>(sat);
    posTraj.m_name = name;

    // Make the satellite into a vehicle
    SatVehicle::add_vehicle(uni, sat, std::move(depend));

    return sat;
}

Satellite testapp::debug_add_lander(Universe& uni, Package& pkg, std::string_view name)
{
    using namespace Magnum::Math::Literals;

    // Start making the blueprint
    BlueprintVehicle blueprint(11);

    // Parts
    DependRes<PrototypePart> capsule = pkg.get<PrototypePart>("part_ph_landerCapsule");
    DependRes<PrototypePart> fuselage = pkg.get<PrototypePart>("part_ph_landerFuselage");
    DependRes<PrototypePart> engine = pkg.get<PrototypePart>("part_ph_landerME");
    DependRes<PrototypePart> leg = pkg.get<PrototypePart>("part_ph_landerLeg");
    DependRes<PrototypePart> rcs = pkg.get<PrototypePart>("part_ph_rcs45");

    Vector3 cfOset = part_offset(*capsule, "attach_bottom",
        *fuselage, "attach_top");
    Vector3 feOset = part_offset(*fuselage, "attach_eng",
        *engine, "attach_engPoint");

    Vector3 leg0 = part_offset(*fuselage, "attach_fuselageLeg0",
        *leg, "attach_leg");
    Vector3 leg90 = part_offset(*fuselage, "attach_fuselageLeg90",
        *leg, "attach_leg");
    Vector3 leg180 = part_offset(*fuselage, "attach_fuselageLeg180",
        *leg, "attach_leg");
    Vector3 leg270 = part_offset(*fuselage, "attach_fuselageLeg270",
        *leg, "attach_leg");

    Vector3 rcs045d = part_offset(*capsule, "attach_rcs45d",
        *rcs, "attach_root");
    Vector3 rcs135d = part_offset(*capsule, "attach_rcs135d",
        *rcs, "attach_root");
    Vector3 rcs225d = part_offset(*capsule, "attach_rcs225d",
        *rcs, "attach_root");
    Vector3 rcs315d = part_offset(*capsule, "attach_rcs315d",
        *rcs, "attach_root");

    Quaternion idRot;
    Vector3 scl{1};
    Magnum::Rad qtrTurn(-90.0_degf);
    Magnum::Rad turn45(-135.0_degf);
    Quaternion rotY_090 = Quaternion::rotation(qtrTurn, Vector3{0, 1, 0});
    Quaternion rotZ_045 = Quaternion::rotation(turn45, Vector3{0, 0, 1});
    Quaternion rotZ_090 = Quaternion::rotation(qtrTurn, Vector3{0, 0, 1});
    Quaternion rotZ_180 = Quaternion::rotation(2 * qtrTurn, Vector3{0, 0, 1});
    Quaternion rotZ_270 = Quaternion::rotation(3 * qtrTurn, Vector3{0, 0, 1});

    auto& capsuleBP = blueprint.add_part(capsule, Vector3{0}, idRot, scl);
    auto& capMmhCfg = get_config(blueprint, capsuleBP, "MMH_tank", "Container");
    capMmhCfg.emplace("ResourceName", "lzdb:mmh");
    capMmhCfg.emplace("FuelLevel", 1.0);
    auto& capNtoCfg = get_config(blueprint, capsuleBP, "NTO_tank", "Container");
    capNtoCfg.emplace("ResourceName", "lzdb:nto");
    capNtoCfg.emplace("FuelLevel", 1.0);

    auto& fuselageBP = blueprint.add_part(fuselage, cfOset, idRot, scl);
    auto& fusNtoCfg = get_config(blueprint, fuselageBP, "fueltankOx", "Container");
    fusNtoCfg.emplace("ResourceName", "lzdb:nto");
    fusNtoCfg.emplace("FuelLevel", 1.0);
    auto& fusA50Cfg = get_config(blueprint, fuselageBP, "fueltankFuel", "Container");
    fusA50Cfg.emplace("ResourceName", "lzdb:aero50");
    fusA50Cfg.emplace("FuelLevel", 1.0);

    auto& engBP = blueprint.add_part(engine, cfOset + feOset, idRot, scl);

    Quaternion yz045 = rotZ_045 * rotY_090;
    Quaternion yz135 = rotZ_045 * rotZ_270 * rotY_090;
    Quaternion yz225 = rotZ_045 * rotZ_180 * rotY_090;
    Quaternion yz315 = rotZ_045 * rotZ_090 * rotY_090;

    auto& rcs1 = blueprint.add_part(rcs, rcs045d, yz045, scl);
    auto& rcs2 = blueprint.add_part(rcs, rcs135d, yz135, scl);
    auto& rcs3 = blueprint.add_part(rcs, rcs225d, yz225, scl);
    auto& rcs4 = blueprint.add_part(rcs, rcs315d, yz315, scl);

    std::vector<std::reference_wrapper<osp::BlueprintPart>> rcsThrusters
    {
        rcs1, rcs2, rcs3, rcs4
    };

    blueprint.add_part(leg, cfOset + leg0, idRot, scl);
    blueprint.add_part(leg, cfOset + leg90, rotZ_270, scl);
    blueprint.add_part(leg, cfOset + leg180, rotZ_180, scl);
    blueprint.add_part(leg, cfOset + leg270, rotZ_090, scl);

    // Wire throttle control
    // from (output): a MachineUserControl m_woThrottle
    // to    (input): a MachineRocket m_wiThrottle
    add_wire(blueprint,
        capsuleBP, "part_ph_landerCapsule", "UserControl", 1,
        engBP, "landerMErkt", "Rocket", 2);

    // Wire attitude contrl to gimbal
    // from (output): a MachineUserControl m_woAttitude
    // to    (input): a MachineRocket m_wiGimbal
    add_wire(blueprint,
        capsuleBP, "part_ph_landerCapsule", "UserControl", 0,
        engBP, "landerMErkt", "Rocket", 0);

    // Pipe fuel tank to rocket engine
    // from (output): fuselage MachineContainer m_outputs;
    // to    (input): engine MachineRocket m_resourcesLines[0]
    add_wire(blueprint,
        fuselageBP, "fueltankFuel", "Container", 0,
        engBP, "landerMErkt", "Rocket", 3);
    add_wire(blueprint,
        fuselageBP, "fueltankOx", "Container", 0,
        engBP, "landerMErkt", "Rocket", 4);

    for (auto const& port : rcsThrusters)
    {
        constexpr std::string_view nozzleBase = "rcs45thruster_";
        std::vector<std::string_view> nozzleNames
        {
            "+x",
            "+y",
            "-x",
            "-y"
        };
        for (std::string_view direction : nozzleNames)
        {
            std::string objName = osp::string_concat(nozzleBase, direction);

            // Attitude control -> RCS Control
            add_wire(blueprint,
                capsuleBP, "part_ph_landerCapsule", "UserControl", 0,
                port, objName, "RCSController", 0);
            // RCS Control -> RCS Rocket
            add_wire(blueprint,
                port, objName, "RCSController", 0,
                port, objName, "Rocket", 2);
            // Capsule tanks -> RCS Rocket
            add_wire(blueprint,
                capsuleBP, "MMH_tank", "Container", 0,
                port, objName, "Rocket", 3);
            add_wire(blueprint,
                capsuleBP, "NTO_tank", "Container", 0,
                port, objName, "Rocket", 4);
        }
    }

    // Put blueprint in package
    auto depend = pkg.add<BlueprintVehicle>(std::string{name}, std::move(blueprint));

    Satellite sat = uni.sat_create();

    // Set the name
    auto& posTraj = uni.get_reg().get<osp::universe::UCompTransformTraj>(sat);
    posTraj.m_name = name;

    // Make the satellite into a vehicle
    SatVehicle::add_vehicle(uni, sat, std::move(depend));

    return sat;
}

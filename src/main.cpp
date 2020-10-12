// This file is a bit spaghetti-style, but should be easy to understand.
// All parts of OSP can be configured through just C++, and understanding what
// this file is doing is a good start to getting familar with the codebase.
// Replace this entire file eventually.

#include <iostream>
//#include <format>
#include <memory>
#include <thread>
#include <stack>
#include <random>

#include "OSPMagnum.h"
#include "DebugObject.h"

#include "osp/types.h"
#include "osp/Universe.h"
#include "osp/Trajectories/Stationary.h"
#include "osp/Satellites/SatActiveArea.h"
#include "osp/Satellites/SatVehicle.h"
#include "osp/Resource/AssetImporter.h"
#include "osp/Resource/Package.h"

#include "osp/Active/ActiveScene.h"
#include "osp/Active/SysVehicle.h"
#include "osp/Active/SysForceFields.h"
#include "osp/Active/SysAreaAssociate.h"

#include "adera/Machines/UserControl.h"
#include "adera/Machines/Rocket.h"
#include "adera/Machines/RCSController.h"
#include "adera/ShipResources.h"

#include "planet-a/Satellites/SatPlanet.h"
#include "planet-a/Active/SysPlanetA.h"

namespace universe
{
    using namespace osp::universe;
    using namespace planeta::universe;
}

namespace active
{
    using namespace osp::active;
    using namespace adera::active;
    using namespace planeta::active;
}

namespace machines
{
    using namespace adera::active::machines;
}

using osp::Vector2;
using osp::Vector3;
using osp::Matrix4;
using osp::Quaternion;

// for the 0xrrggbb_rgbf and angle literals
using namespace Magnum::Math::Literals;


/**
 * Starts a magnum application, an active area, and links them together
 */
void magnum_application();

void config_controls();

/**
 * As the name implies. This should only be called once for the entire lifetime
 * of the program.
 *
 * prefer not to use names like this anywhere else but main.cpp
 */
void load_a_bunch_of_stuff();

/**
 * Adds stuff to the universe
 */
void create_solar_system();

/**
 * Creates a BlueprintVehicle and adds a random mess of 10 part_spamcans to it
 *
 * Also creates a
 *
 * Call load_a_bunch_of_stuff before this function to make sure part_spamcan
 * is loaded
 *
 * @param name
 * @return
 */
osp::universe::Satellite debug_add_random_vehicle(std::string const& name);

osp::universe::Satellite debug_add_deterministic_vehicle(std::string const& name);

Vector3 part_offset(osp::PrototypePart const& attachTo, std::string const& attachToName,
    osp::PrototypePart const& toAttach, std::string const& toAttachName);
osp::universe::Satellite debug_add_part_vehicle(std::string const& name);

/**
 * The spaghetti command line interface that gets inputs from stdin. This
 * function will only return once the user exits.
 * @return An error code maybe
 */
int debug_cli_loop();

// called only from commands to display information
void debug_print_help();
void debug_print_sats();
void debug_print_hier();
void debug_print_update_order();
void debug_print_machines();

// Deals with the underlying OSP universe, with the satellites and stuff. A
// Magnum application or OpenGL context is not required for the universe to
// exist. This also stores loaded resources in packages.
osp::OSPApplication g_osp;

// Deals with the window, OpenGL context, and other game engine stuff that
// often have "Active" written all over them
std::unique_ptr<osp::OSPMagnum> g_ospMagnum;
std::thread g_magnumThread;

// lazily save the arguments to pass to Magnum
int g_argc;
char** g_argv;

int main(int argc, char** argv)
{   
    // eventually do more important things here.
    // just lazily save the arguments
    g_argc = argc;
    g_argv = argv;

    // start doing debug cli loop
    return debug_cli_loop();
}

int debug_cli_loop()
{

    debug_print_help();

    std::string command;

    while(true)
    {
        std::cout << "> ";
        std::cin >> command;

        if (command == "help")
        {
            debug_print_help();
        }
        if (command == "list_uni")
        {
            debug_print_sats();
        }
        if (command == "list_ent")
        {
            debug_print_hier();
        }
        if (command == "list_mach")
        {
            debug_print_machines();
        }
        if (command == "list_upd")
        {
            debug_print_update_order();
        }
        else if (command == "start")
        {
            if (g_magnumThread.joinable())
            {
                g_magnumThread.join();
            }
            std::thread t(magnum_application);
            g_magnumThread.swap(t);
        }
        else if (command == "exit")
        {
            if (g_ospMagnum)
            {
                // request exit if application exists
                g_ospMagnum->exit();
            }
            break;
        }
        else
        {
            std::cout << "that doesn't do anything ._.\n";
        }
    }

    // wait for magnum thread to exit if it exists
    if (g_magnumThread.joinable())
    {
        g_magnumThread.join();
    }

    // destory the universe
    //g_osp.get_universe().get_sats().clear();
    return 0;
}

void magnum_application()
{
    // Create the application
    g_ospMagnum = std::make_unique<osp::OSPMagnum>(
                        osp::OSPMagnum::Arguments{g_argc, g_argv}, g_osp);

    config_controls(); // as the name implies

    // Load if not loaded yet. This only calls once during entire runtime
    if (!g_osp.debug_get_packges().size())
    {
        load_a_bunch_of_stuff();
        create_solar_system();
    }

    // Create an ActiveArea, an ActiveScene, then connect them together

    // Get needed variables
    universe::Universe &uni = g_osp.get_universe();
    universe::SatActiveArea *satAA = static_cast<universe::SatActiveArea*>(
            uni.sat_type_find("Vehicle")->second.get());
    universe::SatPlanet *satPlanet = static_cast<universe::SatPlanet*>(
            uni.sat_type_find("Planet")->second.get());

    // Create an ActiveScene
    active::ActiveScene& scene = g_ospMagnum->scene_add("Area 1");
 
    // Register dynamic systems for that scene
    auto &sysArea = scene.dynamic_system_add<active::SysAreaAssociate>(
                "AreaAssociate", uni);
    auto &sysVehicle = scene.dynamic_system_add<active::SysVehicle>(
                "Vehicle");
    auto &sysPlanet = scene.dynamic_system_add<active::SysPlanetA>(
                "Planet");
    auto &sysGravity = scene.dynamic_system_add<active::SysFFGravity>(
                "FFGravity");

    // Register machines for that scene
    scene.system_machine_add<machines::SysMachineUserControl>("UserControl",
            g_ospMagnum->get_input_handler());
    scene.system_machine_add<machines::SysMachineRocket>("Rocket");
    scene.system_machine_add<machines::SysMachineRCSController>("RCSController");
    scene.system_machine_add<machines::SysMachineContainer>("Container");

    // Make active areas load vehicles and planets
    sysArea.activator_add(satAA, sysVehicle);
    sysArea.activator_add(satPlanet, sysPlanet);

    // create a Satellite with an ActiveArea
    universe::Satellite sat = g_osp.get_universe().sat_create();

    // assign sat as an ActiveArea
    universe::UCompActiveArea &area = satAA->add_get_ucomp(sat);

    // Link ActiveArea to scene using the AreaAssociate
    sysArea.connect(sat);

    // Add a camera to the scene

    // Create the camera entity
    active::ActiveEnt camera = scene.hier_create_child(scene.hier_get_root(),
                                                       "Camera");
    auto &cameraTransform = scene.reg_emplace<active::ACompTransform>(camera);
    auto &cameraComp = scene.get_registry().emplace<active::ACompCamera>(camera);

    cameraTransform.m_transform = Matrix4::translation(Vector3(0, 0, 25));
    cameraTransform.m_enableFloatingOrigin = true;

    cameraComp.m_viewport
            = Vector2(Magnum::GL::defaultFramebuffer.viewport().size());
    cameraComp.m_far = 4096.0f;
    cameraComp.m_near = 0.125f;
    cameraComp.m_fov = 45.0_degf;

    cameraComp.calculate_projection();

    // Add the debug camera controller to the scene. This adds controls
    auto camObj = std::make_unique<osp::DebugCameraController>(scene, camera);

    // Add a CompDebugObject to camera to manage camObj's lifetime
    scene.reg_emplace<osp::CompDebugObject>(camera, std::move(camObj));

    // Starts the game loop. This function is blocking, and will only return
    // when the window is  closed. See OSPMagnum::drawEvent
    g_ospMagnum->exec();

    // Close button has been pressed

    std::cout << "Magnum Application closed\n";

    // Disconnect ActiveArea
    sysArea.disconnect();

    // workaround: wipe mesh resources because they're specific to the
    // opengl context
    osp::Package& pkg = g_osp.debug_get_packges()[0];
    pkg.clear<Magnum::GL::Mesh>();
    pkg.clear<Magnum::GL::Texture2D>();
    pkg.clear<PlumeShader>();

    // destruct the application, this closes the window
    g_ospMagnum.reset();
}

void config_controls()
{
    // Configure Controls

    // It should be pretty easy to write a config file parser that calls these
    // functions.

    using namespace osp;

    using Key = OSPMagnum::KeyEvent::Key;
    using Mouse = OSPMagnum::MouseEvent::Button;
    using VarOp = ButtonVarConfig::VarOperator;
    using VarTrig = ButtonVarConfig::VarTrigger;
    UserInputHandler& userInput = g_ospMagnum->get_input_handler();

    // vehicle control, used by MachineUserControl

    // would help to get an axis for yaw, pitch, and roll, but use individual
    // axis buttons for now
    userInput.config_register_control("vehicle_pitch_up", true,
            {{0, (int) Key::S, VarTrig::PRESSED, false, VarOp::AND}});
    userInput.config_register_control("vehicle_pitch_dn", true,
            {{0, (int) Key::W, VarTrig::PRESSED, false, VarOp::AND}});
    userInput.config_register_control("vehicle_yaw_lf", true,
            {{0, (int) Key::A, VarTrig::PRESSED, false, VarOp::AND}});
    userInput.config_register_control("vehicle_yaw_rt", true,
            {{0, (int) Key::D, VarTrig::PRESSED, false, VarOp::AND}});
    userInput.config_register_control("vehicle_roll_lf", true,
            {{0, (int) Key::Q, VarTrig::PRESSED, false, VarOp::AND}});
    userInput.config_register_control("vehicle_roll_rt", true,
            {{0, (int) Key::E, VarTrig::PRESSED, false, VarOp::AND}});

    // Set throttle max to Z
    userInput.config_register_control("vehicle_thr_max", false,
            {{0, (int) Key::Z, VarTrig::PRESSED, false, VarOp::OR}});
    // Set throttle min to X
    userInput.config_register_control("vehicle_thr_min", false,
            {{0, (int) Key::X, VarTrig::PRESSED, false, VarOp::OR}});
    // Set throttle increase to LShift
    userInput.config_register_control("vehicle_thr_more", true,
        {{osp::sc_keyboard, (int)Key::LeftShift, VarTrig::PRESSED, false, VarOp::OR}});
    // Set throttle decrease to LCtrl
    userInput.config_register_control("vehicle_thr_less", true,
        {{osp::sc_keyboard, (int)Key::LeftCtrl, VarTrig::PRESSED, false, VarOp::OR}});
    // Set self destruct to LeftCtrl+C or LeftShift+A
    userInput.config_register_control("vehicle_self_destruct", false,
            {{0, (int) Key::LeftCtrl, VarTrig::HOLD, false, VarOp::AND},
             {0, (int) Key::C, VarTrig::PRESSED, false, VarOp::OR},
             {0, (int) Key::LeftShift, VarTrig::HOLD, false, VarOp::AND},
             {0, (int) Key::A, VarTrig::PRESSED, false, VarOp::OR}});

    // Camera and Game controls, handled in DebugCameraController

    // Switch to next vehicle
    userInput.config_register_control("game_switch", false,
            {{0, (int) Key::V, VarTrig::PRESSED, false, VarOp::OR}});

    // Set UI Up/down/left/right to arrow keys. this is used to rotate the view
    // for now
    userInput.config_register_control("ui_up", true,
            {{osp::sc_keyboard, (int) Key::Up, VarTrig::PRESSED, false, VarOp::AND}});
    userInput.config_register_control("ui_dn", true,
            {{osp::sc_keyboard, (int) Key::Down, VarTrig::PRESSED, false, VarOp::AND}});
    userInput.config_register_control("ui_lf", true,
            {{osp::sc_keyboard, (int) Key::Left, VarTrig::PRESSED, false, VarOp::AND}});
    userInput.config_register_control("ui_rt", true,
            {{osp::sc_keyboard, (int) Key::Right, VarTrig::PRESSED, false, VarOp::AND}});

    userInput.config_register_control("ui_rmb", true,
            {{osp::sc_mouse, (int) Mouse::Right, VarTrig::PRESSED, false, VarOp::AND}});
}

void load_a_bunch_of_stuff()
{
    // Create a new package
    osp::Package lazyDebugPack("lzdb", "lazy-debug");

    std::string datapath = "OSPData/adera/";
    // Load sturdy glTF files
    // TODO: meshes conflict because they have the same underlying names
    //osp::AssetImporter::load_sturdy_file(datapath + "spamcan.sturdy.gltf", lazyDebugPack);
    //osp::AssetImporter::load_sturdy_file(datapath + "stomper.sturdy.gltf", lazyDebugPack);
    osp::AssetImporter::load_sturdy_file(datapath + "ph_capsule.sturdy.gltf", lazyDebugPack);
    osp::AssetImporter::load_sturdy_file(datapath + "ph_fuselage.sturdy.gltf", lazyDebugPack);
    osp::AssetImporter::load_sturdy_file(datapath + "ph_engine.sturdy.gltf", lazyDebugPack);
    osp::AssetImporter::load_sturdy_file(datapath + "ph_plume.sturdy.gltf", lazyDebugPack);
    osp::AssetImporter::load_sturdy_file(datapath + "ph_rcs.sturdy.gltf", lazyDebugPack);
    osp::AssetImporter::load_sturdy_file(datapath + "ph_rcs_plume.sturdy.gltf", lazyDebugPack);

    // Load placeholder fuel type
    using adera::active::machines::ShipResourceType;
    ShipResourceType fuel;
    fuel.m_identifier = "fuel";
    fuel.m_displayName = "Rocket fuel";
    fuel.m_quanta = 16;
    fuel.m_mass = 1.0f;
    fuel.m_volume = 0.001f;
    fuel.m_density = fuel.m_mass / fuel.m_volume;

    lazyDebugPack.add<ShipResourceType>("fuel", std::move(fuel));

    // Immediately load noise textures
    std::string const noise256 = "noise256";
    std::string const noise1024 = "noise1024";
    std::string const n256path = datapath + noise256 + ".png";
    std::string const n1024path = datapath + noise1024 + ".png";

    osp::AssetImporter::load_image(n256path, lazyDebugPack);
    osp::AssetImporter::load_image(n1024path, lazyDebugPack);
    osp::AssetImporter::compile_tex(n256path, lazyDebugPack);
    osp::AssetImporter::compile_tex(n1024path, lazyDebugPack);

    lazyDebugPack.add<PlumeShader>("plume_shader");

    // Add package to the univere
    g_osp.debug_get_packges().push_back(std::move(lazyDebugPack));

    // Add 50 vehicles so there's something to load
    //g_osp.get_universe().get_sats().reserve(64);

    //s_partsLoaded = true;
}

void create_solar_system()
{
    using osp::universe::Universe;

    Universe& uni = g_osp.get_universe();

    // Register satellite types used
    uni.type_register<universe::SatActiveArea>(uni);
    uni.type_register<universe::SatVehicle>(uni);
    auto &typePlanet = uni.type_register<universe::SatPlanet>(uni);

    // Create trajectory that will make things added to the universe stationary
    auto &stationary = uni.trajectory_create<universe::TrajStationary>(
                                        uni, uni.sat_root());

    //for (int i = 0; i < 20; i ++)
    //{
    //    // Creates a random mess of spamcans
    //    universe::Satellite sat = debug_add_random_vehicle("TestyMcTestFace Mk"
    //                                             + std::to_string(i));

    //    auto &posTraj = uni.get_reg().get<universe::UCompTransformTraj>(sat);

    //    posTraj.m_position = osp::Vector3s(i * 1024l * 5l, 0l, 0l);
    //    posTraj.m_dirty = true;

    //    stationary.add(sat);
    //}

    //universe::Satellite sat = debug_add_deterministic_vehicle("Stomper Mk. I");
    universe::Satellite sat = debug_add_part_vehicle("Placeholder Mk. I");
    auto& posTraj = uni.get_reg().get<universe::UCompTransformTraj>(sat);
    posTraj.m_position = osp::Vector3s(22 * 1024l * 5l, 0l, 0l);
    posTraj.m_dirty = true;
    stationary.add(sat);

    // Add Grid of planets too
    // for now, planets are hard-coded to 128 meters in radius

    for (int x = -0; x < 1; x ++)
    {
        for (int z = -0; z < 1; z ++)
        {
            universe::Satellite sat = g_osp.get_universe().sat_create();

            // assign sat as a planet
            universe::UCompPlanet &planet = typePlanet.add_get_ucomp(sat);

            // set radius
            planet.m_radius = 128;

            auto &posTraj = uni.get_reg().get<universe::UCompTransformTraj>(sat);

            // space planets 400m apart from each other
            // 1024 units = 1 meter
            posTraj.m_position = {x * 1024l * 400l,
                                  1024l * -140l,
                                  z * 1024l * 400l};
        }
    }

}

osp::universe::Satellite debug_add_random_vehicle(std::string const& name)
{
    using osp::BlueprintVehicle;
    using osp::PrototypePart;
    using osp::DependRes;

    // Start making the blueprint

    BlueprintVehicle blueprint;

    // Part to add, very likely a spamcan
    DependRes<PrototypePart> victim =
            g_osp.debug_get_packges()[0]
            .get<PrototypePart>("part_spamcan");

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
    DependRes<BlueprintVehicle> depend = g_osp.debug_get_packges()[0]
            .add<BlueprintVehicle>(name, std::move(blueprint));

    // Create the Satellite containing a SatVehicle

    // TODO: make this more safe

    universe::Universe &uni = g_osp.get_universe();

    // Create blank satellite
    universe::Satellite sat = uni.sat_create();

    // Set the name
    auto &posTraj = uni.get_reg().get<universe::UCompTransformTraj>(sat);
    posTraj.m_name = name;

    // Make it into a vehicle
    auto &typeVehicle = *static_cast<universe::SatVehicle*>(
            uni.sat_type_find("Vehicle")->second.get());
    universe::UCompVehicle &UCompVehicle = typeVehicle.add_get_ucomp(sat);

    // set the SatVehicle's blueprint to the one just made
    UCompVehicle.m_blueprint = std::move(depend);

    return sat;

}

osp::universe::Satellite debug_add_deterministic_vehicle(std::string const & name)
{
    using osp::BlueprintVehicle;
    using osp::PrototypePart;
    using osp::DependRes;

    // Begin blueprint
    BlueprintVehicle blueprint;

    // Part to add
    DependRes<PrototypePart> rocket =
        g_osp.debug_get_packges()[0].get<PrototypePart>("part_stomper");
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
    DependRes<BlueprintVehicle> depend = g_osp.debug_get_packges()[0]
        .add<BlueprintVehicle>(name, std::move(blueprint));


    universe::Universe &uni = g_osp.get_universe();

    // Create new satellite
    universe::Satellite sat = uni.sat_create();

    // Set name
    auto& posTraj = uni.get_reg().get<universe::UCompTransformTraj>(sat);
    posTraj.m_name = name;

    // Make it into a vehicle
    auto& typeVehicle = *static_cast<universe::SatVehicle*>(
        uni.sat_type_find("Vehicle")->second.get());
    universe::UCompVehicle &UCompVehicle = typeVehicle.add_get_ucomp(sat);
    UCompVehicle.m_blueprint = std::move(depend);

    return sat;
}

Vector3 part_offset(osp::PrototypePart const& attachTo, std::string const& attachToName,
    osp::PrototypePart const& toAttach, std::string const& toAttachName)
{
    using namespace osp;

    Vector3 oset1{0.0f};
    Vector3 oset2{0.0f};

    for (PrototypeObject const& obj : attachTo.get_objects())
    {
        if (obj.m_name == attachToName)
        {
            oset1 = obj.m_translation;
            break;
        }
    }

    for (PrototypeObject const& obj : toAttach.get_objects())
    {
        if (obj.m_name == toAttachName)
        {
            oset2 = obj.m_translation;
            break;
        }
    }

    return oset1 - oset2;
}

osp::universe::Satellite debug_add_part_vehicle(std::string const& name)
{
    using osp::BlueprintVehicle;
    using osp::PrototypePart;
    using osp::DependRes;

    // Start making the blueprint
    BlueprintVehicle blueprint;

    osp::Package& pkg = g_osp.debug_get_packges()[0];

    // Parts
    DependRes<PrototypePart> capsule = pkg.get<PrototypePart>("part_phCapsule");
    DependRes<PrototypePart> fuselage = pkg.get<PrototypePart>("part_phFuselage");
    DependRes<PrototypePart> engine = pkg.get<PrototypePart>("part_phEngine");
    DependRes<PrototypePart> rcs = pkg.get<PrototypePart>("part_phLinRCS");

    Vector3 cfOset = part_offset(*capsule, "attach_bottom_capsule",
                                *fuselage, "attach_top_fuselage");
    Vector3 feOset = part_offset(*fuselage, "attach_bottom_fuselage",
        *engine, "attach_top_eng");
    Vector3 rcsOsetTop = Vector3{1.0f, 0.0f, 2.0f} + cfOset;
    Vector3 rcsOsetBtm = Vector3{1.0f, 0.0f, -2.0f} + cfOset;

    Quaternion idRot;
    Vector3 scl{1};
    Magnum::Rad qtrTurn(-90.0_degf);
    Quaternion rotY_090 = Quaternion::rotation(qtrTurn, Vector3{0, 1, 0});
    Quaternion rotZ_090 = Quaternion::rotation(qtrTurn, Vector3{0, 0, 1});
    Quaternion rotZ_180 = Quaternion::rotation(2 * qtrTurn, Vector3{0, 0, 1});
    Quaternion rotZ_270 = Quaternion::rotation(3 * qtrTurn, Vector3{0, 0, 1});

    blueprint.add_part(capsule, Vector3{0}, idRot, scl);
    auto& fuselageBP = blueprint.add_part(fuselage, cfOset, idRot, scl);
    fuselageBP.m_machines[1].m_config.emplace("resourcename", "fuel");
    fuselageBP.m_machines[1].m_config.emplace("fuellevel", 0.5);

    auto& engBP = blueprint.add_part(engine, cfOset + feOset, idRot, scl);

    // Top RCS ring
    blueprint.add_part(rcs, rcsOsetTop, rotY_090, Vector3{1});
    blueprint.add_part(rcs, rotZ_090.transformVector(rcsOsetTop), rotZ_090 * rotY_090, scl);
    blueprint.add_part(rcs, rotZ_180.transformVector(rcsOsetTop), rotZ_180 * rotY_090, scl);
    blueprint.add_part(rcs, rotZ_270.transformVector(rcsOsetTop), rotZ_270 * rotY_090, scl);

    // Bottom RCS ring
    blueprint.add_part(rcs, rcsOsetBtm, rotY_090, Vector3{1});
    blueprint.add_part(rcs, rotZ_090.transformVector(rcsOsetBtm), rotZ_090 * rotY_090, scl);
    blueprint.add_part(rcs, rotZ_180.transformVector(rcsOsetBtm), rotZ_180 * rotY_090, scl);
    blueprint.add_part(rcs, rotZ_270.transformVector(rcsOsetBtm), rotZ_270 * rotY_090, scl);

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

    std::vector<int> rcsPorts = {RCS1, RCS2, RCS3, RCS4,
        RCS5, RCS6, RCS7, RCS8};

    // Wire throttle control
    // from (output): a MachineUserControl m_woThrottle
    // to    (input): a MachineRocket m_wiThrottle
    blueprint.add_wire(Parts::CAPSULE, 0, 1,
        Parts::ENGINE, 0, 2);

    // Wire attitude control to gimbal
    // from (output): a MachineUserControl m_woAttitude
    // to    (input): a MachineRocket m_wiGimbal
    blueprint.add_wire(Parts::CAPSULE, 0, 0,
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

    // put blueprint in package
    DependRes<BlueprintVehicle> depend = pkg.add<BlueprintVehicle>(name, std::move(blueprint));

    // Create the Satellite containing a SatVehicle

    // TODO: make this more safe

    universe::Universe &uni = g_osp.get_universe();

    // Create blank satellite
    universe::Satellite sat = uni.sat_create();

    // Set the name
    auto &posTraj = uni.get_reg().get<universe::UCompTransformTraj>(sat);
    posTraj.m_name = name;

    // Make it into a vehicle
    auto &typeVehicle = *static_cast<universe::SatVehicle*>(
        uni.sat_type_find("Vehicle")->second.get());
    universe::UCompVehicle &UCompVehicle = typeVehicle.add_get_ucomp(sat);

    // set the SatVehicle's blueprint to the one just made
    UCompVehicle.m_blueprint = std::move(depend);

    return sat;

}

void debug_print_help()
{
    std::cout
        << "OSP-Magnum Temporary Debug CLI\n"
        << "Things to type:\n"
        << "* start     - Create an ActiveArea and start Magnum\n"
        << "* list_uni  - List Satellites in the universe\n"
        << "* list_ent  - List Entities in active scene\n"
        << "* list_mach - List Machines in active scene\n"
        << "* list_upd  - List Update order from active scene\n"
        << "* help      - Show this again\n"
        << "* exit      - Deallocate everything and return memory to OS\n";
}

void debug_print_update_order()
{
    if (!g_ospMagnum)
    {
        std::cout << "Can't do that yet, start the magnum application first!\n";
        return;
    }

    osp::active::UpdateOrder &order = g_ospMagnum->get_scenes().begin()
                                            ->second.get_update_order();

    std::cout << "Update order:\n";
    for (auto call : order.get_call_list())
    {
        std::cout << "* " << call.m_name << "\n";
    }
}

void debug_print_machines()
{
    if (!g_ospMagnum)
    {
        std::cout << "Can't do that yet, start the magnum application first!\n";
        return;
    }

    active::ActiveScene &scene = g_ospMagnum->get_scenes().begin()->second;
    auto view = scene.get_registry().view<osp::active::ACompMachines>();

    for (active::ActiveEnt ent : view)
    {
        active::ACompHierarchy& hier = scene.reg_get<active::ACompHierarchy>(ent);
        std::cout << "[" << int(ent) << "]: " << hier.m_name << "\n";

        auto& machines = scene.reg_get<active::ACompMachines>(ent);
        for (auto mach : machines.m_machines)
        {
            active::ActiveEnt machEnt = mach.m_partEnt;
            std::string sysName = mach.m_system->first;
            std::cout << "  ->[" << int(machEnt) << "]: " << sysName << "\n";
        }
    }
}

void debug_print_hier()
{

    if (!g_ospMagnum)
    {
        std::cout << "Can't do that yet, start the magnum application first!\n";
        return;
    }

    std::cout << "ActiveScene Entity Hierarchy:\n";

    std::vector<active::ActiveEnt> parentNextSibling;
    active::ActiveScene &scene = g_ospMagnum->get_scenes().begin()->second;
    active::ActiveEnt currentEnt = scene.hier_get_root();

    parentNextSibling.reserve(16);

    while (true)
    {
        // print some info about the entity
        active::ACompHierarchy &hier = scene.reg_get<active::ACompHierarchy>(currentEnt);
        for (unsigned i = 0; i < hier.m_level; i ++)
        {
            // print arrows to indicate level
            std::cout << "  ->";
        }
        std::cout << "[" << int(currentEnt) << "]: " << hier.m_name << "\n";

        if (hier.m_childCount)
        {
            // entity has some children
            currentEnt = hier.m_childFirst;


            // save next sibling for later if it exists
            if (hier.m_siblingNext != entt::null)
            {
                parentNextSibling.push_back(hier.m_siblingNext);
            }
        }
        else if (hier.m_siblingNext != entt::null)
        {
            // no children, move to next sibling
            currentEnt = hier.m_siblingNext;
        }
        else if (parentNextSibling.size())
        {
            // last sibling, and not done yet
            // is last sibling, move to parent's (or ancestor's) next sibling
            currentEnt = parentNextSibling.back();
            parentNextSibling.pop_back();
        }
        else
        {
            break;
        }
    }
}

void debug_print_sats()
{

    universe::Universe &universe = g_osp.get_universe();

    auto view = universe.get_reg().view<universe::UCompTransformTraj,
                                        universe::UCompType>();

    for (universe::Satellite sat : view)
    {
        auto &posTraj = view.get<universe::UCompTransformTraj>(sat);
        auto &type = view.get<universe::UCompType>(sat);

        auto &pos = posTraj.m_position;

        std::cout << "SATELLITE: \"" << posTraj.m_name << "\" \n";
        if (type.m_type)
        {
            std::cout << " * Type: " << type.m_type->get_name() << "\n";
        }

        if (posTraj.m_trajectory)
        {
            std::cout << " * Trajectory: "
                      << posTraj.m_trajectory->get_type_name() << "\n";
        }

        std::cout << " * Position: ["
                  << pos.x() << ", " << pos.y() << ", " << pos.z() << "]\n";
    }


}

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
#include "DebugObject2.h"

#include "osp/types.h"
#include "osp/Universe.h"
#include "osp/Trajectories/Stationary.h"
#include "osp/Satellites/SatActiveArea.h"
#include "osp/Satellites/SatVehicle.h"
#include "osp/Resource/AssetImporter.h"
#include "osp/Resource/Package.h"

#include "osp/Active/ActiveScene.h"
#include "osp/Active/SysDebugRender2.h"
#include "adera/SysMap.h"

#include "adera/Machines/UserControl.h"
#include "adera/Machines/Rocket.h"

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
 * Starts a magnum application and creates a map scene
 */
void magnum_application_map();
bool g_firstTimeStart = true;

void config_controls_map();

/**
 * Adds stuff to the map universe
 */
void create_solar_system_map();

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
            std::thread t(magnum_application_map);
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

void magnum_application_map()
{
    using namespace osp::active;
    // Create application
    g_ospMagnum = std::make_unique<osp::OSPMagnum>(
        osp::OSPMagnum::Arguments{g_argc, g_argv}, g_osp);

    config_controls_map();

    // Initialize on the first time only
    if (g_firstTimeStart)
    {
        create_solar_system_map();
        g_firstTimeStart = false;
    }

    universe::Universe& uni = g_osp.get_universe();

    // Create a scene
    ActiveScene& scene = g_ospMagnum->scene_add("Map");

    // Dynamics systems
    auto &sysDebugRender = scene.dynamic_system_add<active::SysDebugRender>("DebugRender");
    auto& sysPlanet = scene.dynamic_system_add<active::SysPlanetA>("Planet");
    auto& sysMap = scene.dynamic_system_add<osp::active::SysMap>("Map", uni);

    // Camera
    ActiveEnt camera = scene.hier_create_child(scene.hier_get_root(), "Camera");
    auto& cameraTransform = scene.reg_emplace<ACompTransform>(camera);
    auto& cameraComp = scene.get_registry().emplace<ACompCamera>(camera);

    cameraTransform.m_transform = Matrix4::translation(Vector3(0, 0, 2000.0f));
    cameraComp.m_viewport = Vector2(Magnum::GL::defaultFramebuffer.viewport().size());
    cameraComp.m_far = 1e8f;
    cameraComp.m_near = 1000.0f;
    cameraComp.m_fov = 45.0_degf;
    cameraComp.calculate_projection();

    // Add debug camera controller
    auto camObj = std::make_unique<osp::DebugCameraController>(scene, camera);

    // Add CompDebugObject
    scene.reg_emplace<osp::CompDebugObject>(camera, std::move(camObj));

    // Run game loop
    g_ospMagnum->exec();

    std::cout << "Application closed\n";

    g_ospMagnum.reset();
}

void config_controls_map()
{
    using namespace osp;

    using Key = OSPMagnum::KeyEvent::Key;
    using Mouse = OSPMagnum::MouseEvent::Button;
    using VarOp = ButtonVarConfig::VarOperator;
    using VarTrig = ButtonVarConfig::VarTrigger;
    UserInputHandler& userInput = g_ospMagnum->get_input_handler();

    userInput.config_register_control("ui_rmb", true,
        {{osp::sc_mouse, (int)Mouse::Right, VarTrig::PRESSED, false, VarOp::AND}});
}

osp::SpaceInt meter_to_spaceint(double meters)
{
    return static_cast<osp::SpaceInt>(meters) * 1024ll;
}

osp::Vector3s polar_km_to_v3s(double radiusKM, double angle)
{
    return {
        meter_to_spaceint(1000.0 * radiusKM * cos(angle)),
        meter_to_spaceint(1000.0 * radiusKM * sin(angle)),
        0ll};
}

void create_solar_system_map()
{
    using osp::universe::Universe;
    using osp::universe::Satellite;
    using osp::universe::TrajStationary;
    using osp::universe::UCompTransformTraj;
    using planeta::universe::SatPlanet;
    using planeta::universe::UCompPlanet;
    using osp::universe::UCompType;
    using namespace Magnum::Math::Literals;

    Universe& uni = g_osp.get_universe();

    UCompType* type = uni.get_reg().try_get<UCompType>(static_cast<Satellite>(0));

    SatPlanet &typePlanet = uni.type_register<SatPlanet>(uni);
    uni.type_register<universe::SatActiveArea>(uni);

    TrajStationary &stat = uni.trajectory_create<TrajStationary>(uni, uni.sat_root());

    // Create the sun
    Satellite sun = uni.sat_create();
    UCompPlanet& sunPlanet = typePlanet.add_get_ucomp(sun);
    UCompTransformTraj& sunTT = uni.get_reg().get<UCompTransformTraj>(sun);

    sunPlanet.m_radius = 6.9634e8;
    sunTT.m_position = {0ll, 0ll, 0ll};
    sunTT.m_name = "The sun";

    stat.add(sun);

    // Create mercury
    Satellite mercury = uni.sat_create();
    UCompPlanet& mercuryPlanet = typePlanet.add_get_ucomp(mercury);
    UCompTransformTraj& mercuryTT = uni.get_reg().get<UCompTransformTraj>(mercury);

    mercuryPlanet.m_radius = 1e3;
    mercuryTT.m_position = polar_km_to_v3s(58e6, 0.0);
    mercuryTT.m_name = "Mercury";
    mercuryTT.m_color = 0xCCA91f_rgbf;

    stat.add(mercury);

    // Create venus
    Satellite venus = uni.sat_create();
    UCompPlanet& venusPlanet = typePlanet.add_get_ucomp(venus);
    UCompTransformTraj& venusTT = uni.get_reg().get<UCompTransformTraj>(venus);

    venusPlanet.m_radius = 1e3;
    venusTT.m_position = polar_km_to_v3s(108e6, 0.0);
    venusTT.m_name = "Venus";
    venusTT.m_color = 0xFFDF80_rgbf;

    stat.add(venus);

    // Create the earth
    Satellite earth = uni.sat_create();
    UCompPlanet& earthPlanet = typePlanet.add_get_ucomp(earth);
    UCompTransformTraj& earthTT = uni.get_reg().get<UCompTransformTraj>(earth);

    earthPlanet.m_radius = 6.371e6;
    earthTT.m_position = polar_km_to_v3s(149.6e6, 0.0);
    earthTT.m_name = "Earth";
    earthTT.m_color = 0x24A36E_rgbf;

    stat.add(earth);

    // Create mars
    Satellite mars = uni.sat_create();
    UCompPlanet& marsPlanet = typePlanet.add_get_ucomp(mars);
    UCompTransformTraj& marsTT = uni.get_reg().get<UCompTransformTraj>(mars);

    marsPlanet.m_radius = 1e3;
    marsTT.m_position = polar_km_to_v3s(228e6, 0.0);
    marsTT.m_name = "Mars";
    marsTT.m_color = 0xBF6728_rgbf;

    stat.add(mars);

    // Jupiter
    Satellite jupiter = uni.sat_create();
    UCompPlanet& jupiterPlanet = typePlanet.add_get_ucomp(jupiter);
    UCompTransformTraj& jupiterTT = uni.get_reg().get<UCompTransformTraj>(jupiter);

    jupiterPlanet.m_radius = 1e3;
    jupiterTT.m_position = polar_km_to_v3s(778e6, 0.0);
    jupiterTT.m_name = "Jupiter";
    jupiterTT.m_color = 0xA68444_rgbf;

    stat.add(jupiter);

    // Saturn
    Satellite saturn = uni.sat_create();
    UCompPlanet& saturnPlanet = typePlanet.add_get_ucomp(saturn);
    UCompTransformTraj& saturnTT = uni.get_reg().get<UCompTransformTraj>(saturn);

    saturnPlanet.m_radius = 1e3;
    saturnTT.m_position = polar_km_to_v3s(1400e6, 0.0);
    saturnTT.m_name = "Saturn";
    saturnTT.m_color = 0xCFB78A_rgbf;

    stat.add(saturn);

    // Uranus
    Satellite uranus = uni.sat_create();
    UCompPlanet& uranusPlanet = typePlanet.add_get_ucomp(uranus);
    UCompTransformTraj& uranusTT = uni.get_reg().get<UCompTransformTraj>(uranus);

    uranusPlanet.m_radius = 1e3;
    uranusTT.m_position = polar_km_to_v3s(3000e6, 0.0);
    uranusTT.m_name = "Uranus";
    uranusTT.m_color = 0x91C7EB_rgbf;

    stat.add(uranus);

    // Neptune
    Satellite neptune = uni.sat_create();
    UCompPlanet& neptunePlanet = typePlanet.add_get_ucomp(neptune);
    UCompTransformTraj& neptuneTT = uni.get_reg().get<UCompTransformTraj>(neptune);

    neptunePlanet.m_radius = 1e3;
    neptuneTT.m_position = polar_km_to_v3s(4488e6, 0.0);;
    neptuneTT.m_name = "Neptune";
    neptuneTT.m_color = 0x0785D9_rgbf;

    stat.add(neptune);
}

void debug_print_help()
{
    std::cout
        << "OSP-Magnum Temporary Debug CLI\n"
        << "Things to type:\n"
        << "* start     - Create an ActiveArea and start Magnum\n"
        << "* list_uni  - List Satellites in the universe\n"
        << "* list_ent  - List Entities in active scene\n"
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

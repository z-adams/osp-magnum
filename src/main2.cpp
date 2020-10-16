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
#include "osp/Trajectories/NBody.h"
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
using Magnum::Vector3d;
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

    // Switch to next vehicle
    userInput.config_register_control("game_switch_back", false,
        {{0, (int)Key::V, VarTrig::PRESSED, false, VarOp::OR}});
    // Switch to prev vehicle
    userInput.config_register_control("game_switch", false,
        {{0, (int)Key::LeftShift, VarTrig::HOLD, false, VarOp::AND},
         {0, (int)Key::V, VarTrig::PRESSED, false, VarOp::OR}});


    // RMB drag
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

Vector3d orbit_vel(double radiusKM, double sunMass, double ownMass)
{
    constexpr double G = 6.67e-11;
    return {0.0, 1024.0 * sqrt(G * (sunMass + ownMass) / (1000.0 * radiusKM)), 0.0};
}

struct PlanetBody
{
    double m_radius;
    double m_mass;
    double m_orbitDist;
    std::string m_name;
    Magnum::Color3 m_color;
};

void create_solar_system_map()
{
    using osp::universe::Universe;
    using osp::universe::Satellite;
    using osp::universe::TrajStationary;
    using osp::universe::TrajNBody;
    using osp::universe::TCompMassG;
    using osp::universe::TCompVel;
    using osp::universe::UCompTransformTraj;
    using planeta::universe::SatPlanet;
    using planeta::universe::UCompPlanet;
    using osp::universe::UCompType;
    using namespace Magnum::Math::Literals;

    Universe& uni = g_osp.get_universe();

    UCompType* type = uni.get_reg().try_get<UCompType>(static_cast<Satellite>(0));

    SatPlanet &typePlanet = uni.type_register<SatPlanet>(uni);
    uni.type_register<universe::SatActiveArea>(uni);

    //TrajStationary &stat = uni.trajectory_create<TrajStationary>(uni, uni.sat_root());
    TrajNBody& stat = uni.trajectory_create<TrajNBody>(uni, uni.sat_root());

    auto& reg = uni.get_reg();
    // Create the sun
    Satellite sun = uni.sat_create();
    UCompPlanet& sunPlanet = typePlanet.add_get_ucomp(sun);
    UCompTransformTraj& sunTT = reg.get<UCompTransformTraj>(sun);
    constexpr double sunMass = 1.988e30;
    TCompMassG& sunM = reg.emplace<TCompMassG>(sun, sunMass);

    sunPlanet.m_radius = 6.9634e8;
    sunTT.m_position = {0ll, 0ll, 0ll};
    sunTT.m_name = "The sun";

    stat.add(sun);

    std::vector<PlanetBody> planets;

    // Create mercury
    PlanetBody mercury;
    mercury.m_mass = 3.30e23;
    mercury.m_radius = 1e3;
    mercury.m_orbitDist = 58e6;
    mercury.m_name = "Mercury";
    mercury.m_color = 0xCCA91f_rgbf;
    planets.push_back(std::move(mercury));

    // Create venus
    PlanetBody venus;
    venus.m_mass = 4.867e24;
    venus.m_radius = 1e3;
    venus.m_orbitDist = 108e6;
    venus.m_name = "Venus";
    venus.m_color = 0xFFDF80_rgbf;
    planets.push_back(std::move(venus));

    // Create the earth
    PlanetBody earth;
    earth.m_mass = 5.97e24;
    earth.m_radius = 6.371e6;
    earth.m_orbitDist = 149.6e6;
    earth.m_name = "Earth";
    earth.m_color = 0x24A36E_rgbf;
    planets.push_back(std::move(earth));

    // Create the moon
    PlanetBody moon;
    moon.m_mass = 7.34e22;
    moon.m_radius = 1.737e6;
    moon.m_orbitDist = 149.6e6 - 400e3;
    moon.m_name = "Moon";
    moon.m_color = 0xDDDDDD_rgbf;
    planets.push_back(std::move(moon));

    // Create mars
    PlanetBody mars;
    mars.m_mass = 6.42e23;
    mars.m_radius = 1e3;
    mars.m_orbitDist = 228e6;
    mars.m_name = "Mars";
    mars.m_color = 0xBF6728_rgbf;
    planets.push_back(std::move(mars));

    // Jupiter
    PlanetBody jupiter;
    jupiter.m_mass = 1.898e27;
    jupiter.m_radius = 1e3;
    jupiter.m_orbitDist = 778e6;
    jupiter.m_name = "Jupiter";
    jupiter.m_color = 0xA68444_rgbf;
    planets.push_back(std::move(jupiter));

    // Saturn
    PlanetBody saturn;
    saturn.m_mass = 5.68e26;
    saturn.m_radius = 1e3;
    saturn.m_orbitDist = 1400e6;
    saturn.m_name = "Saturn";
    saturn.m_color = 0xCFB78A_rgbf;
    planets.push_back(std::move(saturn));

    // Uranus
    PlanetBody uranus;
    uranus.m_mass = 8.68e4;
    uranus.m_radius = 1e3;
    uranus.m_orbitDist = 3000e6;
    uranus.m_name = "Uranus";
    uranus.m_color = 0x91C7EB_rgbf;
    planets.push_back(std::move(uranus));

    // Neptune
    PlanetBody neptune;
    neptune.m_mass = 1.02e26;
    neptune.m_radius = 1e3;
    neptune.m_orbitDist = 4488e6;
    neptune.m_name = "Neptune";
    neptune.m_color = 0x0785D9_rgbf;
    planets.push_back(std::move(neptune));

    for (PlanetBody const& body : planets)
    {
        Satellite sat = uni.sat_create();
        UCompPlanet& satPlanet = typePlanet.add_get_ucomp(sat);
        UCompTransformTraj& satTT = reg.get<UCompTransformTraj>(sat);
        TCompMassG& satM = reg.emplace<TCompMassG>(sat, body.m_mass);
        TCompVel& satV = reg.emplace<TCompVel>(sat, orbit_vel(body.m_orbitDist, sunMass, body.m_mass));

        satPlanet.m_radius = body.m_radius;
        satTT.m_name = body.m_name;
        satTT.m_color = body.m_color;
        satTT.m_position = polar_km_to_v3s(body.m_orbitDist, 0.0);

        stat.add(sat);
    }

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

    std::vector<active::ActiveEnt> parentNextSibling;
    active::ActiveScene &scene = g_ospMagnum->get_scenes().begin()->second;
    auto planetView = scene.get_registry().view<osp::active::CompDrawableDebug>(entt::exclude<osp::active::CompPass1Debug>);
    auto ringView = scene.get_registry().view<osp::active::CompDrawableDebug, osp::active::CompPass1Debug>();

    std::cout << "Planets:\n";
    for (auto ent : planetView)
    {
        std::cout << static_cast<int>(ent) << " ";
    }
    std::cout << "\nRings:\n";
    for (auto ent : ringView)
    {
        std::cout << static_cast<int>(ent) << " ";
    }
    std::cout << "\n";
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

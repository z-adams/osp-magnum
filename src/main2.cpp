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
#include "adera/MassSpringLattice.h"
#include "adera/GraphViewer.h"

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
void magnum_application();
bool g_firstTimeStart = true;

void config_controls();

/**
 * Adds stuff to the map universe
 */
void create_scene(osp::active::ActiveScene& scene);

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

    while (true)
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
    using namespace osp::active;
    // Create application
    g_ospMagnum = std::make_unique<osp::OSPMagnum>(
        osp::OSPMagnum::Arguments{g_argc, g_argv}, g_osp);

    config_controls();

    universe::Universe& uni = g_osp.get_universe();

    // Create a scene
    ActiveScene& scene = g_ospMagnum->scene_add("Map");

    // Dynamics systems
    auto& sysDebugRender = scene.dynamic_system_add<active::SysDebugRender>("DebugRender");
    auto& sysSoftBody = scene.dynamic_system_add<SysSoftBody>("soft_body");
    auto& sysView = scene.dynamic_system_add<SysGraphViewer>("graph_view");
    
    create_scene(scene);
    sysView.check_and_initialize_objects();

    // Camera
    ActiveEnt camera = scene.hier_create_child(scene.hier_get_root(), "Camera");
    auto& cameraTransform = scene.reg_emplace<ACompTransform>(camera);
    auto& cameraComp = scene.get_registry().emplace<ACompCamera>(camera);

    cameraTransform.m_transform = Matrix4::translation(Vector3(0, 0, 10.0f));
    cameraComp.m_viewport = Vector2(Magnum::GL::defaultFramebuffer.viewport().size());
    cameraComp.m_far = 100.0f;
    cameraComp.m_near = 0.1f;
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

void config_controls()
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
        {{0, (int)Key::V, VarTrig::PRESSED, false, VarOp::OR}});

    // RMB drag
    userInput.config_register_control("ui_rmb", true,
        {{osp::sc_mouse, (int)Mouse::Right, VarTrig::PRESSED, false, VarOp::AND}});
}

void create_scene(osp::active::ActiveScene& scene)
{
    using namespace active;
    ActiveEnt body = scene.get_registry().create();
    auto& bodyTransform = scene.reg_emplace<ACompTransform>(body);

    auto& softbody = scene.reg_emplace<LCompSoftBody>(body);

    auto spring = [](unsigned a, unsigned b, float len)
    {
        float strength = 50.0f;
        return LatticeSpring{a, b, len, strength, 0.0f};
    };

    std::vector<LatticeMass> masses = {
        {Vector3{1.0f, -1.0f, -1.0f}, 1.0f},
        {Vector3{1.0f, 2.0f, -1.0f}, 1.0f},
        {Vector3{-1.0f, 1.0f, -1.0f}, 1.0f},
        {Vector3{-1.0f, -1.0f, -1.0f}, 1.0f},

        {Vector3{1.0f, -1.0f, 1.0f}, 1.0f},
        {Vector3{1.0f, 1.0f, 1.0f}, 1.0f},
        {Vector3{-1.0f, 1.0f, 1.0f}, 1.0f},
        {Vector3{-1.0f, -1.0f, 1.0f}, 1.0f}
    };

    /* Cube
            6          5
     2          1


            7          4
     3          0
     */
    float len = 2.0f;
    float diag = len * sqrtf(2.0f);
    std::vector<LatticeSpring> springs = {
        {0, 1, 2.0f, 50.0f, 0.0f},
        {1, 2, 2.0f, 50.0f, 0.0f},
        {2, 3, 2.0f, 50.0f, 0.0f},
        {3, 0, 2.0f, 50.0f, 0.0f},
        {0, 2, 2.0f * sqrtf(2.0f), 50.0f, 0.0f},
        {1, 3, 2.0f * sqrtf(2.0f), 50.0f, 0.0f},

        {4, 5, 2.0f, 50.0f, 0.0f},
        {5, 6, 2.0f, 50.0f, 0.0f},
        {6, 7, 2.0f, 50.0f, 0.0f},
        {7, 4, 2.0f, 50.0f, 0.0f},
        {4, 6, 2.0f * sqrtf(2.0f), 50.0f, 0.0f},
        {5, 7, 2.0f * sqrtf(2.0f), 50.0f, 0.0f},

        spring(0, 4, 2.0f),
        spring(1, 5, 2.0f),
        spring(3, 7, 2.0f),
        spring(2, 6, 2.0f),
        spring(0, 5, diag),
        spring(1, 4, diag),
        spring(3, 6, diag),
        spring(2, 7, diag),
        spring(3, 4, diag),
        spring(0, 7, diag),
        spring(2, 5, diag),
        spring(1, 6, diag)
    };

    softbody.m_masses = masses;
    softbody.m_velocities = std::vector<Vector3>(masses.size(), Vector3{0.0f});
    softbody.m_springs = springs;
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

        std::cout << "SATELLITE: \"" << posTraj.m_name << "\" (" << static_cast<int>(sat) << ")\n";
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
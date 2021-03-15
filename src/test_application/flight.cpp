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

#include "flight.h"

#include "DebugObject.h"

#include <osp/Active/ActiveScene.h>
#include <osp/Active/SysDebugRender.h>
#include <osp/Active/SysVehicle.h>
#include <osp/Active/SysForceFields.h>
#include <osp/Active/SysAreaAssociate.h>
#include <osp/Active/SysSkybox.h>

#include <osp/Satellites/SatVehicle.h>

#include <adera/Machines/UserControl.h>
#include <adera/Machines/Rocket.h>
#include <adera/Machines/RCSController.h>
#include <adera/ShipResources.h>

#include <planet-a/Active/SysPlanetA.h>
#include <planet-a/Satellites/SatPlanet.h>

//tmp
#include <osp/CubemapUtils.h>

using namespace testapp;

using osp::Vector2;

using osp::Vector3;
using osp::Quaternion;
using osp::Matrix3;
using osp::Matrix4;

// for the 0xrrggbb_rgbf and angle literals
using namespace Magnum::Math::Literals;

using osp::universe::Universe;
using osp::universe::Satellite;
using osp::universe::SatActiveArea;
using osp::universe::SatVehicle;

using osp::universe::UCompActiveArea;

using osp::active::ActiveEnt;
using osp::active::ACompTransform;
using osp::active::ACompCamera;
using osp::active::ACompFloatingOrigin;

using adera::active::machines::SysMachineUserControl;
using adera::active::machines::SysMachineRocket;
using adera::active::machines::SysMachineRCSController;
using adera::active::machines::SysMachineContainer;

using planeta::universe::SatPlanet;


void testapp::test_flight(std::unique_ptr<OSPMagnum>& pMagnumApp,
                 osp::OSPApplication& rOspApp, OSPMagnum::Arguments args)
{

    // Get needed variables
    Universe &rUni = rOspApp.get_universe();

    // Create the application
    pMagnumApp = std::make_unique<OSPMagnum>(args, rOspApp);

    // Run temporary stuff
    /*using osp::math::cubemap::NormalMapGenerator;
    auto compute = std::make_unique<NormalMapGenerator>();
    compute->process("ldem_16.tga", 10916.4f, 0.001f, "normalmap.tga");*/
    /*using osp::math::cubemap::CubemapComputeShader;
    auto compute = std::make_unique<CubemapComputeShader<Magnum::GL::ImageFormat::RGBA8>>();
    compute->process("lroc_color_poles_4k.png", "OSPData/adera/Moon/Diffuse/", 2048);*/

    /*compute.reset();
    pMagnumApp.reset();
    return;*/

    // Configure the controls
    config_controls(*pMagnumApp);

    // Create an ActiveScene
    osp::active::ActiveScene& scene = pMagnumApp->scene_create("Area 1");

    // Register dynamic systems needed for flight scene

    scene.dynamic_system_create<osp::active::SysPhysics_t>();
    scene.dynamic_system_create<osp::active::SysWire>();
    scene.dynamic_system_create<osp::active::SysDebugRender>();
    scene.dynamic_system_create<osp::active::SysAreaAssociate>();
    scene.dynamic_system_create<osp::active::SysVehicle>();
    scene.dynamic_system_create<osp::active::SysExhaustPlume>();
    scene.dynamic_system_create<planeta::active::SysPlanetA>(pMagnumApp->get_input_handler());
    scene.dynamic_system_create<osp::active::SysFFGravity>();
    scene.dynamic_system_create<osp::active::SysSkybox>();

    osp::active::SysSkybox::set_skybox(scene, "WWWTyroSkybox");

    // Register machines for that scene
    scene.system_machine_create<SysMachineUserControl>(pMagnumApp->get_input_handler());
    scene.system_machine_create<SysMachineRocket>();
    scene.system_machine_create<SysMachineRCSController>();
    scene.system_machine_create<SysMachineContainer>();

    // Make active areas load vehicles and planets
    //sysArea.activator_add(rUni.sat_type_find_index<SatVehicle>(), sysVehicle);
    //sysArea.activator_add(rUni.sat_type_find_index<SatPlanet>(), sysPlanet);

    // create a Satellite with an ActiveArea
    Satellite areaSat = rUni.sat_create();

    // assign sat as an ActiveArea
    UCompActiveArea &area = SatActiveArea::add_active_area(rUni, areaSat);

    // Link ActiveArea to scene using the AreaAssociate
    osp::active::SysAreaAssociate::connect(scene, rUni, areaSat);

    // Add default-constructed physics world to scene
    scene.get_registry().emplace<osp::active::ACompPhysicsWorld_t>(scene.hier_get_root());

    // Add a camera to the scene

    // Create the camera entity
    ActiveEnt camera = scene.hier_create_child(scene.hier_get_root(),
                                                       "Camera");
    auto &cameraTransform = scene.reg_emplace<ACompTransform>(camera);
    auto &cameraComp = scene.get_registry().emplace<ACompCamera>(camera);

    cameraTransform.m_transform = Matrix4::translation(Vector3(0, 0, 25));
    scene.reg_emplace<ACompFloatingOrigin>(camera);

    cameraComp.m_viewport
            = Vector2(Magnum::GL::defaultFramebuffer.viewport().size());
    cameraComp.m_far = 1u << 24;
    cameraComp.m_near = 1.0f;
    cameraComp.m_fov = 45.0_degf;

    cameraComp.calculate_projection();

    // Add the debug camera controller to the scene. This adds controls
    auto camObj = std::make_unique<DebugCameraController>(scene, camera);

    // Add a ACompDebugObject to camera to manage camObj's lifetime
    scene.reg_emplace<ACompDebugObject>(camera, std::move(camObj));

    // Starts the game loop. This function is blocking, and will only return
    // when the window is  closed. See OSPMagnum::drawEvent
    pMagnumApp->exec();

    // Close button has been pressed

    std::cout << "Magnum Application closed\n";
    
    // Disconnect ActiveArea
    osp::active::SysAreaAssociate::disconnect(scene);

    // destruct the application, this closes the window
    pMagnumApp.reset();
}

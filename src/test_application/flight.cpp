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
#include <adera/SysSunflare.h>
#include <osp/Active/SysPrepass.h>
#include <osp/Shaders/RT.h>
#include <planet-a/Active/SysPlanetA.h>
#include <planet-a/Satellites/SatPlanet.h>

//tmp
#include <osp/CubemapUtils.h>
#include <Magnum/GL/Renderer.h>
#include <Magnum/GL/Framebuffer.h>
#include <Magnum/Math/Range.h>
#include <Magnum/GL/Renderbuffer.h>
#include <Magnum/GL/RenderbufferFormat.h>
#include <Magnum/GL/Texture.h>
#include <Magnum/GL/TextureFormat.h>

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
    osp::active::ActiveScene& rScene = pMagnumApp->scene_create("Area 1");

    // Add system functions needed for flight scene
    osp::active::SysPhysics_t::add_functions(rScene);
    //osp::active::SysWire::add_functions(rScene);
    osp::active::SysDebugRender::add_functions(rScene);
    osp::active::PrepassExecutor::add_functions(rScene);
    osp::active::SysAreaAssociate::add_functions(rScene);
    osp::active::SysVehicle::add_functions(rScene);
    osp::active::SysExhaustPlume::add_functions(rScene);
    planeta::active::SysPlanetA::add_functions(rScene);
    osp::active::SysFFGravity::add_functions(rScene);
    osp::active::SysSkybox::add_functions(rScene);
    adera::active::SysSunflare::add_functions(rScene);

    osp::active::SysSkybox::set_skybox(rScene, "WWWTyroSkybox");

    // Register machines for that scene
    rScene.system_machine_create<SysMachineUserControl>(pMagnumApp->get_input_handler());
    rScene.system_machine_create<SysMachineRocket>();
    rScene.system_machine_create<SysMachineRCSController>();
    rScene.system_machine_create<SysMachineContainer>();

    // Add sun flare
    {
        ActiveEnt sunFlare = rScene.hier_create_child(rScene.hier_get_root(), "sunflare");

        adera::active::SysSunflare::add_flare(rScene, sunFlare, "sunflare.png");
    }

    // Define render passes

    // Prepass
    //rScene.get_render_queue().push_back(&osp::active::PrepassExecutor::execute_prepass);

    // Opaque pass
    rScene.get_render_queue().push_back(
        [](osp::active::ActiveScene& rScene, ACompCamera& camera)
        {
            using namespace Magnum;
            using Magnum::GL::Renderer;
            using namespace osp::active;

            auto& reg = rScene.get_registry();

            camera.m_renderTarget->bind();

            camera.m_renderTarget->clear(
                GL::FramebufferClear::Color
                | GL::FramebufferClear::Depth
                | GL::FramebufferClear::Stencil);

            Renderer::enable(Renderer::Feature::DepthTest);
            Renderer::enable(Renderer::Feature::FaceCulling);
            Renderer::disable(Renderer::Feature::Blending);

            using Skybox_t = SkyboxShader::ACompSkyboxShaderInstance;

            // Fetch opaque objects
            auto opaqueView = reg.view<CompDrawableDebug, ACompTransform>(
                entt::exclude<CompTransparentDebug, Skybox_t, CompBackgroundDebug>);

            SysDebugRender::draw_group(rScene, opaqueView, camera);
        }
    );

    // Draw Skybox
    rScene.get_render_queue().push_back(
        [](osp::active::ActiveScene& rScene, ACompCamera& camera)
        {
            using Magnum::GL::Renderer;
            using namespace osp::active;

            auto& reg = rScene.get_registry();

            using Skybox_t = SkyboxShader::ACompSkyboxShaderInstance;
            auto boxview = reg.view<CompDrawableDebug, ACompTransform,
                Skybox_t, CompBackgroundDebug>();
            Renderer::enable(Renderer::Feature::SeamlessCubeMapTexture);
            Renderer::disable(Renderer::Feature::Blending);
            Renderer::enable(Renderer::Feature::DepthTest);
            Renderer::enable(Renderer::Feature::DepthClamp);
            Renderer::setDepthFunction(Renderer::DepthFunction::LessOrEqual);
            Renderer::enable(Renderer::Feature::FaceCulling);
            Renderer::setFaceCullingMode(Renderer::PolygonFacing::Front);

            SysDebugRender::draw_group(rScene, boxview, camera);

            Renderer::setDepthFunction(Renderer::DepthFunction::Less);
            Renderer::setFaceCullingMode(Renderer::PolygonFacing::Back);
            Renderer::disable(Renderer::Feature::DepthClamp);
        }
    );

    // Draw sun test point
    rScene.get_render_queue().push_back(
        [](osp::active::ActiveScene& rScene, ACompCamera& camera)
        {
            using Magnum::GL::Renderer;
            using namespace osp::active;

            auto& reg = rScene.get_registry();

            auto backgroundView = reg.view<CompDrawableDebug, ACompTransform,
                CompBackgroundDebug, CompQueryObj>();
            for (ActiveEnt e : backgroundView)
            {
                auto& q = backgroundView.get<CompQueryObj>(e);
                glBeginQuery(GL_ANY_SAMPLES_PASSED, q.m_id);
            }
            SysDebugRender::draw_group(rScene, backgroundView, camera);
            glEndQuery(GL_ANY_SAMPLES_PASSED);
        }
    );

    // Transparent pass
    rScene.get_render_queue().push_back(
        [](osp::active::ActiveScene& rScene, ACompCamera& camera)
        {
            using Magnum::GL::Renderer;
            using namespace osp::active;

            auto& reg = rScene.get_registry();

            Renderer::enable(Renderer::Feature::DepthTest);
            Renderer::enable(Renderer::Feature::FaceCulling);
            Renderer::enable(Renderer::Feature::Blending);
            Renderer::setBlendFunction(
                Renderer::BlendFunction::SourceAlpha,
                Renderer::BlendFunction::OneMinusSourceAlpha);

            auto transparentView = reg.view<CompDrawableDebug, CompVisibleDebug,
                CompTransparentDebug, ACompTransform>(entt::exclude<CompOverlayDebug>);

            // Draw backfaces
            Renderer::setFaceCullingMode(Renderer::PolygonFacing::Front);
            SysDebugRender::draw_group(rScene, transparentView, camera);

            // Draw frontfaces
            Renderer::setFaceCullingMode(Renderer::PolygonFacing::Back);
            SysDebugRender::draw_group(rScene, transparentView, camera);
        }
    );

    // Draw overlays
    rScene.get_render_queue().push_back(
        [](osp::active::ActiveScene& rScene, ACompCamera& camera)
        {
            using namespace Magnum;
            using Magnum::GL::Renderer;
            using namespace osp::active;

            auto& reg = rScene.get_registry();

            auto overlayObjects = reg.view<CompDrawableDebug, CompVisibleDebug,
                CompTransparentDebug, ACompTransform, CompOverlayDebug>();

            if (camera.m_renderTarget.empty())
            {
                GL::defaultFramebuffer.clear(GL::FramebufferClear::Depth);
            }
            else
            {
                camera.m_renderTarget->clear(GL::FramebufferClear::Depth);
            }

            Renderer::enable(Renderer::Feature::Blending);
            Renderer::setBlendFunction(
                Renderer::BlendFunction::SourceAlpha,
                Renderer::BlendFunction::OneMinusSourceAlpha);
            Renderer::setFaceCullingMode(Renderer::PolygonFacing::Back);

            SysDebugRender::draw_group(rScene, overlayObjects, camera);
        }
    );

    // Raytracing pass
    //rScene.get_render_queue().push_back(&osp::active::SysRaytracer::raytrace);

    // Render offscreen buffer
    rScene.get_render_queue().push_back(
        [](osp::active::ActiveScene& rScene, ACompCamera& camera)
        {
            using namespace Magnum;
            using Magnum::GL::Renderer;
            using namespace osp::active;
            using namespace osp;

            GL::defaultFramebuffer.bind();
            GL::defaultFramebuffer.clear(
                GL::FramebufferClear::Color
                | GL::FramebufferClear::Depth
                | GL::FramebufferClear::Stencil);

            Renderer::disable(Renderer::Feature::DepthTest);
            Renderer::disable(Renderer::Feature::FaceCulling);
            Renderer::disable(Renderer::Feature::Blending);

            DependRes<GL::Texture2D> colorTex =
                rScene.get_context_resources().get<GL::Texture2D>("offscreen_fbo_color");
            SysDebugRender::render_framebuffer(rScene, *colorTex);
        }
    );

    // Make active areas load vehicles and planets
    //sysArea.activator_add(rUni.sat_type_find_index<SatVehicle>(), sysVehicle);
    //sysArea.activator_add(rUni.sat_type_find_index<SatPlanet>(), sysPlanet);

    // create a Satellite with an ActiveArea
    Satellite areaSat = rUni.sat_create();

    // assign sat as an ActiveArea
    UCompActiveArea &area = SatActiveArea::add_active_area(rUni, areaSat);

    // Link ActiveArea to scene using the AreaAssociate
    osp::active::SysAreaAssociate::connect(rScene, rUni, areaSat);

    // Add default-constructed physics world to scene
    rScene.get_registry().emplace<osp::active::ACompPhysicsWorld_t>(rScene.hier_get_root());

    // Add a camera to the scene

    // Create the camera entity
    ActiveEnt camera = rScene.hier_create_child(rScene.hier_get_root(),
        "Camera");
    auto &cameraTransform = rScene.reg_emplace<ACompTransform>(camera);
    auto &cameraComp = rScene.get_registry().emplace<ACompCamera>(camera);

    cameraTransform.m_transform = Matrix4::translation(Vector3(0, 0, 25));
    rScene.reg_emplace<ACompFloatingOrigin>(camera);

    {
        // Add an offscreen framebuffer
        using namespace Magnum;

        auto& glResources = rScene.get_context_resources();

        Vector2i viewSize = GL::defaultFramebuffer.viewport().size();

        GL::Texture2D color;
        color.setStorage(1, GL::TextureFormat::RGB8, viewSize);
        osp::DependRes<GL::Texture2D> colorRes = glResources.add<GL::Texture2D>("offscreen_fbo_color", std::move(color));

        GL::Renderbuffer depthStencil;
        depthStencil.setStorage(GL::RenderbufferFormat::Depth24Stencil8, viewSize);
        osp::DependRes<GL::Renderbuffer> depthStencilRes =
            glResources.add<GL::Renderbuffer>("offscreen_fbo_depthStencil", std::move(depthStencil));

        GL::Framebuffer fbo(Range2Di{{0, 0}, viewSize});
        fbo.attachTexture(GL::Framebuffer::ColorAttachment{0}, *colorRes, 0);
        fbo.attachRenderbuffer(GL::Framebuffer::BufferAttachment::DepthStencil, *depthStencilRes);

        osp::DependRes<GL::Framebuffer> fboRes =
            rScene.get_context_resources().add<GL::Framebuffer>("offscreen_fbo", std::move(fbo));

        // Set up camera
        cameraComp.m_viewport = Vector2(viewSize);
        cameraComp.m_far = 1u << 24;
        cameraComp.m_near = 1.0f;
        cameraComp.m_fov = 45.0_degf;
        cameraComp.m_renderTarget = std::move(fboRes);

        cameraComp.calculate_projection();
    }

    // Add the debug camera controller to the scene. This adds controls
    auto camObj = std::make_unique<DebugCameraController>(rScene, camera);

    // Add a ACompDebugObject to camera to manage camObj's lifetime
    rScene.reg_emplace<ACompDebugObject>(camera, std::move(camObj));

    // Starts the game loop. This function is blocking, and will only return
    // when the window is  closed. See OSPMagnum::drawEvent
    pMagnumApp->exec();

    // Close button has been pressed

    std::cout << "Magnum Application closed\n";
    
    // Disconnect ActiveArea
    osp::active::SysAreaAssociate::disconnect(rScene);

    // destruct the application, this closes the window
    pMagnumApp.reset();
}

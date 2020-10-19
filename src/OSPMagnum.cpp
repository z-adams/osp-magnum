#include "OSPMagnum.h"
#include "osp/types.h"

#include <Magnum/Math/Color.h>
#include <Magnum/PixelFormat.h>
#include <Magnum/GL/Renderer.h>

#include "osp/Trajectories/NBody.h"

#include <iostream>

static int counter = 100;
static int psIdx = 0;
static float psX[10001], psY[10001];

namespace osp
{

OSPMagnum::OSPMagnum(const Magnum::Platform::Application::Arguments& arguments,
                     OSPApplication &ospApp) :
        Magnum::Platform::Application{
            arguments,
            Configuration{}.setTitle("OSP-Magnum").setSize({1280, 720})},
        m_userInput(12),
        m_ospApp(ospApp)
{
    //.setWindowFlags(Configuration::WindowFlag::Hidden)

    m_imgui = Magnum::ImGuiIntegration::Context(Vector2{windowSize()} / dpiScaling(),
        windowSize(), framebufferSize());
    m_implot = ImPlot::CreateContext();

    m_timeline.start();

}

OSPMagnum::~OSPMagnum()
{
    ImPlot::DestroyContext(m_implot);
}

void OSPMagnum::drawEvent()
{
    using namespace Magnum;

    GL::defaultFramebuffer.clear(GL::FramebufferClear::Color | GL::FramebufferClear::Depth);

//    if (m_area)
//    {
//        //Scene3D& scene = m_area->get_scene();
//        //if (!m_area->is_loaded_active())
//        //{
//        //    // Enable active area if not already done so
//        //    m_area->activate();
//        //}

//       // std::cout << "deltaTime: " << m_timeline.previousFrameDuration() << "\n";

//        // TODO: physics update
//        m_userInput.update_controls();
//        m_area->update_physics(1.0f / 60.0f);
//        m_userInput.clear_events();
//        // end of physics update

//        m_area->draw_gl();
//    }

    m_userInput.update_controls();

    m_ospApp.get_universe().update();

    for (auto &[name, scene] : m_scenes)
    {
        scene.update();
    }

    m_userInput.clear_events();

    for (auto &[name, scene] : m_scenes)
    {
        scene.update_hierarchy_transforms();


        // temporary: draw using first camera component found
        scene.draw(scene.get_registry().view<active::ACompCamera>().front());
    }


    // TODO: GUI and stuff
    m_imgui.newFrame();
    
    if (counter > 100)
    {
        counter = 0;
        universe::Satellite earth{4};
        auto& tt = m_ospApp.get_universe().get_reg().get<universe::UCompTransformTraj>(earth);
        auto& vel = m_ospApp.get_universe().get_reg().get<universe::TCompVel>(earth);
        psX[psIdx] = static_cast<double>(tt.m_position.x()) / (1024.0 * 1e6); 
        psY[psIdx] = vel.m_vel.x() / (1024.0);
        psIdx++;
        if (psIdx > 10000) { psIdx = 0; }
    }
    counter++;
    if (ImPlot::BeginPlot("Phase Space (x)", "x", "dx/dt", {-1, -1}))
    {
        //ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle);
        ImPlot::PlotLine("Earth", psX, psY, psIdx);
        ImPlot::EndPlot();
    }

    m_imgui.updateApplicationCursor(*this);

    GL::Renderer::enable(GL::Renderer::Feature::Blending);
    GL::Renderer::enable(GL::Renderer::Feature::ScissorTest);
    GL::Renderer::disable(GL::Renderer::Feature::FaceCulling);
    GL::Renderer::disable(GL::Renderer::Feature::DepthTest);
    m_imgui.drawFrame();
    GL::Renderer::enable(GL::Renderer::Feature::DepthTest);
    GL::Renderer::enable(GL::Renderer::Feature::FaceCulling);
    GL::Renderer::disable(GL::Renderer::Feature::ScissorTest);
    GL::Renderer::disable(GL::Renderer::Feature::Blending);

    swapBuffers();
    m_timeline.nextFrame();
    redraw();
}


void OSPMagnum::keyPressEvent(KeyEvent& event)
{
    if (event.isRepeated()) { return; }
    if (m_imgui.handleKeyPressEvent(event)) { return; }
    m_userInput.event_raw(sc_keyboard, (int) event.key(),
                          UserInputHandler::ButtonRawEvent::PRESSED);
}

void OSPMagnum::keyReleaseEvent(KeyEvent& event)
{
    if (event.isRepeated()) { return; }
    if (m_imgui.handleKeyReleaseEvent(event)) { return; }
    m_userInput.event_raw(sc_keyboard, (int) event.key(),
                          UserInputHandler::ButtonRawEvent::RELEASED);
}

void OSPMagnum::mousePressEvent(MouseEvent& event)
{
    if (m_imgui.handleMousePressEvent(event)) { return; }
    m_userInput.event_raw(sc_mouse, (int) event.button(),
                          UserInputHandler::ButtonRawEvent::PRESSED);
}

void OSPMagnum::mouseReleaseEvent(MouseEvent& event)
{
    if (m_imgui.handleMouseReleaseEvent(event)) { return; }
    m_userInput.event_raw(sc_mouse, (int) event.button(),
                          UserInputHandler::ButtonRawEvent::RELEASED);
}

void OSPMagnum::mouseMoveEvent(MouseMoveEvent& event)
{
    if (m_imgui.handleMouseMoveEvent(event)) { return; }
    m_userInput.mouse_delta(event.relativePosition());
}

void OSPMagnum::mouseScrollEvent(MouseScrollEvent & event)
{
    if (m_imgui.handleMouseScrollEvent(event)) { return; }
    m_userInput.scroll_delta(static_cast<Vector2i>(event.offset()));
}

active::ActiveScene& OSPMagnum::scene_add(const std::string &name)
{
    auto pair = m_scenes.try_emplace(name, m_userInput, m_ospApp);
    return pair.first->second;
}

}

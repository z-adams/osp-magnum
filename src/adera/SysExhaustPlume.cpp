#include "SysExhaustPlume.h"
#include "osp/Active/ActiveScene.h"
#include "adera/Machines/Rocket.h"
#include "osp/Resource/AssetImporter.h"

#include <Magnum/Trade/MeshData.h>
#include <Magnum/MeshTools/Compile.h>
#include <Magnum/GL/Mesh.h>
#include <Magnum/GL/Texture.h>
#include <Magnum/Trade/ImageData.h>
#include <Magnum/ImageView.h>
#include <Magnum/GL/Sampler.h>
#include <Magnum/GL/TextureFormat.h>

osp::active::SysExhaustPlume::SysExhaustPlume(ActiveScene& scene)
    : m_scene(scene), m_time(0.0f),
    m_updatePlume(scene.get_update_order(), "exhaust_plume", "mach_rocket", "",
        std::bind(&SysExhaustPlume::update_plumes, this))
{}

void osp::active::SysExhaustPlume::update_plumes()
{
    m_time += m_scene.get_time_delta_fixed();

    using adera::active::machines::MachineRocket;
    auto plumeView = m_scene.get_registry().view<ACompExhaustPlume, CompDrawableDebug,
        CompVisibleDebug>();

    for (ActiveEnt plumeEnt : plumeView)
    {
        ACompExhaustPlume& plume = plumeView.get<ACompExhaustPlume>(plumeEnt);
        PlumeShaderInstance& shader = std::get<PlumeShaderInstance>(
            plumeView.get<CompDrawableDebug>(plumeEnt).m_shader);
        CompVisibleDebug& visibility = plumeView.get<CompVisibleDebug>(plumeEnt);

        auto& machine = m_scene.reg_get<MachineRocket>(plume.m_parentMachineRocket);
        const auto& throttle = *machine.request_input(2)->connected_value();
        const auto throttlePos = std::get<wiretype::Percent>(throttle).m_value;

        shader.update_time(m_time);

        if (throttlePos > 0.0f)
        {
            shader.set_power(throttlePos);
            visibility.state = true;
        }
        else
        {
            visibility.state = false;
        }
    }
}

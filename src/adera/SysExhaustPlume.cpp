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
#include "SysExhaustPlume.h"
#include "osp/Active/ActiveScene.h"
#include "adera/Machines/Rocket.h"
#include "osp/Resource/AssetImporter.h"
#include "osp/Active/SysDebugRender.h"

#include <Magnum/Trade/MeshData.h>
#include <Magnum/MeshTools/Compile.h>
#include <Magnum/GL/Mesh.h>
#include <Magnum/GL/Texture.h>
#include <Magnum/Trade/ImageData.h>
#include <Magnum/ImageView.h>
#include <Magnum/GL/Sampler.h>
#include <Magnum/GL/TextureFormat.h>

using namespace osp::active;

SysExhaustPlume::SysExhaustPlume(ActiveScene& rScene)
    : m_scene(rScene)
    , m_time(0.0f),
    m_updatePlume(rScene.get_update_order(), "exhaust_plume", "mach_rocket", "",
        [this](ActiveScene& rScene){this->update_plumes(rScene);})
{}

/*void attach_plume_effect(ActiveEnt partRoot, ActiveEnt ent)
{
    ActiveEnt plumeNode = entt::null;

    auto findPlumeHandle = [this, &plumeNode](ActiveEnt ent)
    {
        auto const& node = m_scene.reg_get<ACompHierarchy>(ent);
        static constexpr std::string_view nodePrefix = "fx_plume_";
        if (0 == node.m_name.compare(0, nodePrefix.size(), nodePrefix))
        {
            plumeNode = ent;
            return EHierarchyTraverseStatus::Stop;  // terminate search
        }
        return EHierarchyTraverseStatus::Continue;
    };

    m_scene.hierarchy_traverse(ent, findPlumeHandle);

    if (plumeNode == entt::null)
    {
        std::cout << "ERROR: could not find plume anchor for MachineRocket "
            << ent << "\n";
        return;
    }
    std::cout << "MachineRocket " << ent << "'s associated plume: "
        << plumeNode << "\n";

    // Get plume effect
    Package& pkg = m_scene.get_application().debug_find_package("lzdb");
    std::string_view plumeAnchorName = m_scene.reg_get<ACompHierarchy>(plumeNode).m_name;
    std::string_view effectName = plumeAnchorName.substr(3, plumeAnchorName.length() - 3);
    DependRes<PlumeEffectData> plumeEffect = pkg.get<PlumeEffectData>(effectName);
    if (plumeEffect.empty())
    {
        std::cout << "ERROR: couldn't find plume effect " << effectName << "!\n";
        return;
    }

    m_scene.reg_emplace<ACompExhaustPlume>(plumeNode, ent, plumeEffect);
}*/

void SysExhaustPlume::create_plumes(ActiveScene& rScene)
{
    for (auto [ent, precursor] : rScene.get_registry().view<ACompPlumePrecursor>().each())
    {
        ActiveEnt rocket = entt::null;
        std::string_view rocketName = precursor.m_rocketName;
        auto findPlumeRocket = [&rScene, &rocket, rocketName](ActiveEnt ent)
        {
            auto const& node = rScene.reg_get<ACompHierarchy>(ent);
            if (0 == node.m_name.compare(0, rocketName.size(), rocketName))
            {
                rocket = ent;
                return EHierarchyTraverseStatus::Stop;  // terminate search
            }
            return EHierarchyTraverseStatus::Continue;
        };

        // Parts are only 1 level deep for now
        ActiveEnt partRoot = rScene.reg_get<ACompHierarchy>(ent).m_parent;
        rScene.hierarchy_traverse(partRoot, findPlumeRocket);

        if (rocket == entt::null)
        {
            std::cout << "ERROR: could not find rocket named " << rocketName
                << " for plume " << ent << "\n";
            continue;
        }
        std::cout << "Plume " << ent << " created and associated with rocket "
            << rocket << "\n";

        // Get plume effect
        Package& pkg = m_scene.get_application().debug_find_package("lzdb");
        DependRes<PlumeEffectData> plumeEffect =
            pkg.get<PlumeEffectData>(precursor.m_plumeEffectName);
        if (plumeEffect.empty())
        {
            std::cout << "ERROR: couldn't find plume effect "
                << precursor.m_plumeEffectName << "!\n";
            return;
        }

        // Create ACompExhaustPlume
        rScene.reg_emplace<ACompExhaustPlume>(ent, rocket, plumeEffect);
        rScene.get_registry().remove<ACompPlumePrecursor>(ent);
    }
}

void SysExhaustPlume::initialize_plume(ActiveScene& rScene, ActiveEnt node)
{
    using Magnum::GL::Mesh;
    using Magnum::GL::Texture2D;
    using Magnum::Trade::ImageData2D;
    using namespace Magnum::Math::Literals;
    using adera::shader::PlumeShader;
    using ShaderInstance_t = PlumeShader::ACompPlumeShaderInstance;

    auto const& plume = m_scene.reg_get<ACompExhaustPlume>(node);

    if (m_scene.get_registry().try_get<ShaderInstance_t>(node))
    {
        return;  // plume already initialized
    }

    DependRes<PlumeEffectData> plumeEffect = plume.m_effect;

    Package& pkg = m_scene.get_application().debug_find_package("lzdb");

    // Get plume mesh
    Package& glResources = m_scene.get_context_resources();
    DependRes<Mesh> plumeMesh = glResources.get<Mesh>(plumeEffect->m_meshName);
    if (plumeMesh.empty())
    {
        plumeMesh = AssetImporter::compile_mesh(plumeEffect->m_meshName, pkg, glResources);
    }

    // Get plume tex (TEMPORARY: just grab noise1024.png)
    constexpr std::string_view texName = "OSPData/adera/noise1024.png";
    DependRes<Texture2D> n1024 = glResources.get<Texture2D>(texName);
    if (n1024.empty())
    {
        n1024 = AssetImporter::compile_tex(texName, pkg, glResources);
    }

    // Emplace plume shader instance component from plume effect parameters
    m_scene.reg_emplace<ShaderInstance_t>(node,
        glResources.get<PlumeShader>("plume_shader"),
        n1024,
        n1024,
        *plumeEffect);

    m_scene.reg_emplace<CompDrawableDebug>(node, plumeMesh,
        &adera::shader::PlumeShader::draw_plume);
    m_scene.reg_emplace<CompVisibleDebug>(node, false);
    m_scene.reg_emplace<CompTransparentDebug>(node, true);
}

void SysExhaustPlume::update_plumes(ActiveScene& rScene)
{
    m_time += m_scene.get_time_delta_fixed();

    using adera::active::machines::MachineRocket;
    using ShaderInstance_t = adera::shader::PlumeShader::ACompPlumeShaderInstance;

    auto& reg = m_scene.get_registry();

    // Create plumes that were initialized by SysVehicle
    create_plumes(rScene);

    // Initialize any uninitialized plumes
    auto uninitializedComps =
        reg.view<ACompExhaustPlume>(entt::exclude<ShaderInstance_t>);
    for (ActiveEnt plumeEnt : uninitializedComps)
    {
        initialize_plume(rScene, plumeEnt);
    }

    // Process plumes
    auto plumeView =
        reg.view<ACompExhaustPlume, ShaderInstance_t, CompVisibleDebug>();
    for (ActiveEnt plumeEnt : plumeView)
    {
        auto& plume = plumeView.get<ACompExhaustPlume>(plumeEnt);
        auto& plumeShader = plumeView.get<ShaderInstance_t>(plumeEnt);
        CompVisibleDebug& visibility = plumeView.get<CompVisibleDebug>(plumeEnt);

        auto& machine = m_scene.reg_get<MachineRocket>(plume.m_parentMachineRocket);
        float powerLevel = machine.current_output_power();

        plumeShader.m_currentTime = m_time;

        if (powerLevel > 0.0f)
        {
            plumeShader.m_powerLevel = powerLevel;
            visibility.m_state = true;
        }
        else
        {
            visibility.m_state = false;
        }
    }
}

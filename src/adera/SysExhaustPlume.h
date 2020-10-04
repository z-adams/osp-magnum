#pragma once
#include "osp/Active/physics.h"
#include <Magnum/Shaders/Phong.h>
#include "adera/Plume.h"
#include "osp/Resource/Resource.h"

namespace osp::active
{

struct ACompExhaustPlume
{
    ActiveEnt m_parentMachineRocket{entt::null};
    Magnum::Shaders::Phong m_shader;
    //DependRes<PlumeEffect> m_effect;
};

class SysExhaustPlume : public IDynamicSystem
{
public:
    SysExhaustPlume(ActiveScene& scene);
    ~SysExhaustPlume() = default;

    void update_plumes();

private:
    ActiveScene& m_scene;
    float m_time;

    UpdateOrderHandle m_updatePlume;
};

} // namespace osp::active
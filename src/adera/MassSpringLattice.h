#pragma once
#include <vector>

#include "osp/types.h"
#include "osp/Active/activetypes.h"
#include "osp/Active/ActiveScene.h"

struct LatticeMass
{
    Magnum::Vector3 m_pos;
    float m_mass;
};

struct LatticeSpring
{
    unsigned m_massA;
    unsigned m_massB;
    float m_lengthRelaxed;
    float m_springConstant;
    float m_damping;
};

struct LCompSoftBody
{
    std::vector<LatticeMass> m_masses;
    std::vector<Magnum::Vector3> m_velocities;
    std::vector<LatticeSpring> m_springs;
};

class SysSoftBody : public osp::active::IDynamicSystem
{
public:
    SysSoftBody(osp::active::ActiveScene& scene);
    ~SysSoftBody() = default;
    void update();
private:
    osp::active::ActiveScene& m_scene;
    osp::active::UpdateOrderHandle m_softBodyUpdate;
};

/*struct SoftBoneJoint
{
    Magnum::Vector3 m_pos;
    Magnum::Quaternion m_orient;
    float m_mass;
};

struct SoftSkeletonBone
{
    unsigned m_massA;
    Magnum::Vector3 m_dirFromA;
    unsigned m_massB;
    Magnum::Vector3 m_dirFromB;
    float m_lengthRelaxed;
    float m_springConstant;
    float m_damping;
    float m_bendingSpringConstant;
    float m_bendingDamping;
};

struct LCompSkeleton
{
    std::vector<LatticeMass> m_masses;
};

class SysSoftSkeleton : public osp::active::IDynamicSystem
{
public:
    SysSoftSkeleton(osp::active::ActiveScene& scene);
    ~SysSoftSkeleton() = default;
    void update();
private:
    osp::active::ActiveScene& m_scene;
    osp::active::UpdateOrderHandle m_skeletonUpdate;
};*/

#include "MassSpringLattice.h"

using namespace osp::active;

SysSoftBody::SysSoftBody(ActiveScene& scene)
    :m_scene(scene),
    m_softBodyUpdate(scene.get_update_order(),
        "soft_body", "physics", "debug",
        std::bind(&SysSoftBody::update, this))
{}

void SysSoftBody::update()
{
    constexpr float dt = 1.0f / 60.0f;
    std::vector<Magnum::Vector3> accelerations(100, Magnum::Vector3{0.0f});
    auto view = m_scene.get_registry().view<LCompSoftBody>();
    for (ActiveEnt e : view)
    {
        auto& softbody = view.get<LCompSoftBody>(e);
        for (LatticeSpring const& s : softbody.m_springs)
        {
            LatticeMass const& m1 = softbody.m_masses[s.m_massA];
            LatticeMass const& m2 = softbody.m_masses[s.m_massB];
            Magnum::Vector3 r = m2.m_pos - m1.m_pos;
            float dist = r.length();
            Magnum::Vector3 rHat = r.normalized();

            float displacement = dist - s.m_lengthRelaxed;
            Magnum::Vector3 force = -displacement * s.m_springConstant * r;

            Magnum::Vector3& a1 = accelerations[s.m_massA];
            Magnum::Vector3& a2 = accelerations[s.m_massB];
            a1 += -force / m1.m_mass;
            a2 += force / m2.m_mass;
        }

        for (size_t i = 0; i < softbody.m_masses.size(); i++)
        {
            softbody.m_velocities[i] += accelerations[i] * dt;
            softbody.m_masses[i].m_pos += softbody.m_velocities[i] * dt;
        }
    }
}

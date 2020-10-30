#include "NBody.h"
#include <iostream>

using namespace osp::universe;

using Magnum::Vector3d;

TrajNBody::TrajNBody(Universe& universe, Satellite center) :
    CommonTrajectory<TrajNBody>(universe, center)
{
    m_stepsAhead = 5'000;
    m_currentStep = 9'999;
}

template <typename VIEW_T, typename INPUTVIEW_T>
void TrajNBody::update_accelerations(VIEW_T& view, INPUTVIEW_T& inputView)
{
    // ### Precompute Gravity Sources ###
    struct Source
    {
        Vector3d pos;  // 24 bytes
        double mass;   // 8 bytes
        Satellite sat;  // 4 bytes :'(
    };
    std::vector<Source> sources;
    sources.reserve(inputView.size());
    for (Satellite src : inputView)
    {
        Source source
        {
            static_cast<Vector3d>(inputView.get<UCompTransformTraj>(src).m_position) / 1024.0,
            inputView.get<TCompMassG>(src).m_mass,
            src
        };
        sources.push_back(std::move(source));
    }

    // ### Update Accelerations ###
    for (Satellite sat : view)
    {
        auto& pos = view.get<UCompTransformTraj>(sat).m_position;
        Vector3d posD = static_cast<Vector3d>(pos) / 1024.0;

        Vector3d A{0.0};
        for (Source const& src : sources)
        {
            if (src.sat == sat) { continue; }
            Vector3d r = src.pos - posD;
            Vector3d rHat = r.normalized();
            double denom = r.x() * r.x() + r.y() * r.y() + r.z() * r.z();
            A += (src.mass / denom) * rHat;
        }
        A *= 1024.0 * G;

        view.get<TCompAccel>(sat).m_acceleration = A;
    }
}

template<typename VIEW_T>
void TrajNBody::update_world(VIEW_T& view)
{
    for (size_t body = 0; body < m_evol.m_bodies.size(); body++)
    {
        Satellite sat = m_evol.m_bodies[body];
        IntegrationStep const& lastStep = m_evol.m_steps.back();

        view.get<TCompVel>(sat).m_vel = lastStep.m_velocities[body] * 1024.0;
        view.get<UCompTransformTraj>(m_evol.m_bodies[body]).m_position = static_cast<Vector3s>(m_evol.m_steps[m_currentStep].m_positions[body] * 1024.0);
    }
}

template <typename VIEW_T>
void TrajNBody::evolve_system(VIEW_T& view)
{
    size_t nBodies = 0;
    for (Satellite const sat : view) { nBodies++; }
    // ### Compute initial conditions ###
    m_evol.reserve_bodies(nBodies);
    m_evol.reserve_steps(m_stepsAhead);
    {
        size_t i = 0;
        for (Satellite src : view)
        {
            std::string name = view.get<UCompTransformTraj>(src).m_name;
            m_evol.m_bodies[i] = src;
            m_evol.m_masses[i] = view.get<TCompMassG>(src).m_mass;
            IntegrationStep& step1 = m_evol.m_steps[0];
            step1.m_positions[i] =
                static_cast<Vector3d>(view.get<UCompTransformTraj>(src).m_position) / 1024.0;
            step1.m_velocities[i] = view.get<TCompVel>(src).m_vel / 1024.0;
            i++;
        }
    }

    std::vector<double>& mass = m_evol.m_masses;
    std::vector<Satellite>& bodies = m_evol.m_bodies;
    for (size_t step = 1; step < m_stepsAhead; step++)
    {
        IntegrationStep& prevStep = m_evol.m_steps[step - 1];

        // ### Update Accelerations ###
        for (size_t body = 0; body < nBodies; body++)
        {
            std::string name = view.get<UCompTransformTraj>(bodies[body]).m_name;
            Vector3d A{0.0};
            for (size_t src = 0; src < nBodies; src++)
            {
                std::string name2 = view.get<UCompTransformTraj>(bodies[src]).m_name;

                // Crucial bandwidth: reading positions and masses
                if (src == body) { continue; }
                Vector3d r = prevStep.m_positions[src] - prevStep.m_positions[body];
                Vector3d rHat = r.normalized();
                double denom = r.x()*r.x() + r.y()*r.y() + r.z()*r.z();
                A += (mass[src] / denom) * rHat;
            }
            A *= G;

            prevStep.m_accelerations[body] = A;
        }

        double dt = m_timestep;
        IntegrationStep& nextStep = m_evol.m_steps[step];
        // ### Update velocities & positions ###
        for (size_t body = 0; body < nBodies; body++)
        {
            std::string name = view.get<UCompTransformTraj>(bodies[body]).m_name;
            // Crucial bandwidth: reading acceleration, velocity, position all at once
            auto const& acceleration = prevStep.m_accelerations[body];
            auto const& vel = prevStep.m_velocities[body];
            auto const& pos = prevStep.m_positions[body];

            auto& newVel = nextStep.m_velocities[body];
            auto& newPos = nextStep.m_positions[body];

            newVel = vel + acceleration*dt;
            newPos = pos + newVel*dt;
        }
    }
}

template<typename VIEW_T, typename INPUTVIEW_T>
void TrajNBody::fast_update(VIEW_T& view, INPUTVIEW_T& posMassView)
{
    double dt = m_timestep;
    struct Source
    {
        Vector3d pos;
        double mass;
    };
    std::vector<Source> sources;
    sources.reserve(posMassView.size());
    for (Satellite src : posMassView)
    {
        Source source
        {
            static_cast<Vector3d>(posMassView.get<UCompTransformTraj>(src).m_position) / 1024.0,
            posMassView.get<TCompMassG>(src).m_mass
        };
        sources.push_back(std::move(source));
    }
    for (Satellite asteroid : view)
    {
        auto& pos = view.get<UCompTransformTraj>(asteroid).m_position;
        auto& vel = view.get<TCompVel>(asteroid).m_vel;
        Vector3d posD = static_cast<Vector3d>(pos) / 1024.0;

        Vector3d A{0.0};
        for (Source const& src : sources)
        {
            Vector3d r = src.pos - posD;
            Vector3d rHat = r.normalized();
            double denom = r.x() * r.x() + r.y() * r.y() + r.z() * r.z();
            A += (src.mass / denom) * rHat;
        }
        A *= 1024.0 * G;
        vel += A * dt;
        pos += static_cast<Vector3s>(vel * dt);
    }
}

void TrajNBody::update()
{
    double dt = m_timestep;
    auto& reg = m_universe.get_reg();
    auto view = reg.view<UCompTransformTraj, TCompMassG, TCompVel, TCompAccel>(entt::exclude<TCompAsteroid>);
    auto sourceView = reg.view<UCompTransformTraj, TCompMassG>(entt::exclude<TCompAsteroid>);

    // Update accelerations (full dynamics)
    update_accelerations(view, sourceView);

    // Update asteroids
    auto asteroidView = reg.view<UCompTransformTraj, TCompVel, TCompAsteroid>();
    fast_update(asteroidView, sourceView);

    // Update velocities & positions (full dynamics)
    for (Satellite sat : view)
    {
        auto& acceleration = view.get<TCompAccel>(sat).m_acceleration;
        auto& vel = view.get<TCompVel>(sat).m_vel;
        auto& pos = view.get<UCompTransformTraj>(sat).m_position;

        vel += acceleration * dt;
        pos += static_cast<Vector3s>(vel * dt);
    }
}

/*void TrajNBody::update()
{
    static bool firstTime = true;
    double dt = m_timestep;
    auto& reg = m_universe.get_reg();
    auto view = reg.view<UCompTransformTraj, TCompMassG, TCompVel, TCompAccel>(entt::exclude<TCompAsteroid>);

    auto asteroidView = reg.view<UCompTransformTraj, TCompVel, TCompAsteroid>();
    auto sourceView = reg.view<UCompTransformTraj, TCompMassG>(entt::exclude<TCompAsteroid>);
    fast_update(asteroidView, sourceView);

    m_currentStep++;
    if (m_currentStep >= m_stepsAhead - 1)
    {
        std::cout << "Calculating...\n";
        evolve_system(view);
        m_currentStep = 1;
        m_justUpdated = true;
    }
    else
    {
        m_justUpdated = false;
    }
    update_world(view);
}*/

std::vector<Magnum::Vector3> TrajNBody::get_sat_traj(Satellite sat) const
{
    // Find index of sat
    size_t idx = 0;
    while (idx < m_evol.m_bodies.size())
    {
        if (m_evol.m_bodies[idx] == sat) { break; }
        idx++;
    }

    std::vector<Vector3> data;
    data.reserve(m_evol.m_nSteps);

    for (size_t i = 0; i < m_evol.m_nSteps; i++)
    {
        Vector3 v = static_cast<Vector3>(m_evol.m_steps[i].m_positions[idx]) * 1e-6f;
        data.push_back(std::move(v));
    }

    return data;
}

// with ints
//void TrajNBody::update()
//{
//    auto& reg = m_universe.get_reg();
//    auto& view = reg.view<UCompTransformTraj, TCompMassG, TCompVel>();
//    auto& posMassView = reg.view<UCompTransformTraj, TCompMassG>();
//
//    for (Satellite sat : view)
//    {
//        auto& pos = view.get<UCompTransformTraj>(sat).m_position;
//        TCompMassG& ownMass = view.get<TCompMassG>(sat);
//
//        Vector3s F{0ll, 0ll, 0ll};
//        for (Satellite otherSat : posMassView)
//        {
//            if (otherSat == sat) { continue; }
//            auto const& otherPos = posMassView.get<UCompTransformTraj>(sat).m_position;
//            TCompMassG const& otherMass = posMassView.get<TCompMassG>(sat);
//
//            Vector3s r = otherPos - pos;
//            SpaceInt rx = r.x();
//            SpaceInt ry = r.y();
//            SpaceInt rz = r.z();
//            SpaceInt rMag2 = rx*rx + ry*ry + rz*rz;
//
//            /* Unit conversion
//
//               Units here are a bit strange. "space mass" is defined as 10^10 / G kilograms,
//               and each distance unit is defined as 1 / 1024 meters. Since the force due to
//               gravity only has one factor of 'G', the force equation for our purposes is
//               F = SM1 * SM2 / (G * r^2). However, this number is quite small due to the
//               magnutude of the distance, so to correct things we use the "space newton"
//               which in the given units is equal to 10^10 / (1024G) newtons, or about 10^17.
//               Factoring this in with the "space mass/distance" force equation, the terms
//               cancel leaving only 10^10 * 1024^3, which is the constant here.
//               If we divide the force of the earth and sun on each other by this factor,
//               we get 242'026 space newtons, which is of fairly reasonable magnutude and
//               which I will test here. I'm tired as hell so we'll see if this makes any
//               sense tomorrow.
//            
//            */
//            constexpr SpaceInt a = 10'000'000'000ll * 1'073'741'824ll;
//            constexpr SpaceInt a1 = 10'000'000'000 * 1024;
//
//            SpaceInt Fx = a1 * ownMass.m_massG * otherMass.m_massG / rx;
//            SpaceInt Fy = a1 * ownMass.m_massG * otherMass.m_massG / ry;
//            SpaceInt Fz = a1 * ownMass.m_massG * otherMass.m_massG / rz;
//
//            Fx /= rx;
//            Fy /= ry;
//            Fz /= rz;
//        }
//    }
//}

void SysEvolution::reserve_bodies(size_t bodies)
{
    m_bodies.clear();
    m_masses.clear();
    m_nBodies = bodies;
    m_bodies.resize(bodies);
    m_masses.resize(bodies);
}

void SysEvolution::reserve_steps(size_t steps)
{
    if (m_nBodies == 0) { return; } // Silently fail without telling anyone

    m_nSteps = steps;
    m_steps.resize(steps);
    for (size_t i = 0; i < steps; i++)
    {
        IntegrationStep& is = m_steps[i];
        is.m_accelerations.clear();
        is.m_velocities.clear();
        is.m_positions.clear();

        is.m_accelerations.resize(m_nBodies);
        is.m_velocities.resize(m_nBodies);
        is.m_positions.resize(m_nBodies);
    }
}

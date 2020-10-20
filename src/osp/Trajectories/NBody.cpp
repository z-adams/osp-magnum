#include "NBody.h"

using namespace osp::universe;

using Magnum::Vector3d;

TrajNBody::TrajNBody(Universe& universe, Satellite center) :
    CommonTrajectory<TrajNBody>(universe, center)
{

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
            // 1:
            /*Vector3d r = (src.pos - posD) / 1024.0;
            Vector3d rHat = r.normalized();
            double denom = r.x()*r.x() + r.y()*r.y() + r.z()*r.z();
            A += src.mass * rHat / denom;*/

            // 2:
            /*Vector3d r = src.pos - posD;
            Vector3d rHat = r.normalized();
            double denom = (r.x()*r.x() + r.y()*r.y() + r.z()*r.z()) / (1024.0 * 1024.0);
            A += src.mass * rHat / denom;*/

            // 3 (with /1024 division in source pre-fetch loop and on posD init)
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

/*void TrajNBody::update()
{
    double dt = m_timestep;
    auto& reg = m_universe.get_reg();
    auto view = reg.view<UCompTransformTraj, TCompMassG, TCompVel, TCompAccel>();
    auto sourceView = reg.view<UCompTransformTraj, TCompMassG>(entt::exclude<TCompAsteroid>);

    // Update accelerations
    for (Satellite sat : view)
    {
        Vector3d A = update_accelerations(sat, view, sourceView);
        A *= 1024.0 * G;
        view.get<TCompAccel>(sat).m_acceleration = A;
    }

    // Update velocities & positions
    for (Satellite sat : view)
    {
        auto& acceleration = view.get<TCompAccel>(sat).m_acceleration;
        auto& vel = view.get<TCompVel>(sat).m_vel;
        auto& pos = view.get<UCompTransformTraj>(sat).m_position;

        vel += acceleration * dt;
        pos += static_cast<Vector3s>(vel * dt);
    }
}*/

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
    // 1:
    /*for (Satellite asteroid : asteroidView)
    {
        Vector3d A = update_accelerations(asteroid, asteroidView, sourceView);
        auto& vel = view.get<TCompVel>(asteroid).m_vel;
        auto& pos = view.get<UCompTransformTraj>(asteroid).m_position;

        vel += A * dt;
        pos += static_cast<Vector3s>(vel * dt);
    }*/
    // 2:
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

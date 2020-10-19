#include "NBody.h"

using namespace osp::universe;

using Magnum::Vector3d;

TrajNBody::TrajNBody(Universe& universe, Satellite center) :
    CommonTrajectory<TrajNBody>(universe, center)
{

}

void TrajNBody::update()
{
    double dt = m_timestep;
    auto& reg = m_universe.get_reg();
    auto& view = reg.view<UCompTransformTraj, TCompMassG, TCompVel, TCompAccel>();
    auto& posMassView = reg.view<UCompTransformTraj, TCompMassG>();

    // Update accelerations
    for (Satellite sat : view)
    {
        auto& tt = view.get<UCompTransformTraj>(sat);
        auto& pos = tt.m_position;
        Vector3d posD = static_cast<Vector3d>(pos);
        TCompMassG& mass = view.get<TCompMassG>(sat);
        std::string name = tt.m_name;

        Vector3d A{0.0};
        for (Satellite otherSat : posMassView)
        {
            if (otherSat == sat) { continue; }
            auto const& othertt = posMassView.get<UCompTransformTraj>(otherSat);
            auto const& otherPos = othertt.m_position;
            std::string otherName = othertt.m_name;
            TCompMassG const& otherMass = posMassView.get<TCompMassG>(otherSat);

            Vector3d r = (static_cast<Vector3d>(otherPos) - posD) / 1024.0;
            Vector3d rHat = r.normalized();
            double denom = r.x()*r.x() + r.y()*r.y() + r.z()*r.z();

            A += otherMass.m_mass * rHat / denom;
        }
        A *= 1024.0 * G;
        view.get<TCompAccel>(sat).m_acceleration = A;
    }

    // Update velocities & positions
    for (Satellite sat : view)
    {
        auto& acceleration = view.get<TCompAccel>(sat).m_acceleration;
        auto& vel = view.get<TCompVel>(sat);
        auto& tt = view.get<UCompTransformTraj>(sat);
        auto& pos = tt.m_position;

        vel.m_vel += acceleration * dt;
        pos += static_cast<Vector3s>(vel.m_vel * dt);
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

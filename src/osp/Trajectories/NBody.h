#pragma once

#include "../Universe.h"
#include <Magnum/Math/Vector3.h>

namespace osp::universe
{

/* Spacemass: To convert mass to space mass, multiply by G/10^10.
 * This makes e.g.
    - Ceres = 3
    - Pluto = 87
    - Earth = 39,846 units
    - Jupiter = 12,667,821
    - Sun = 13,275,182,700
*/
//typedef uint32_t SpaceMass;

constexpr double G = 6.674e-11;

struct TCompMassG
{
    //SpaceMass m_massG;
    double m_mass;
};

// Units: space vel (1/1024 m/s)
struct TCompVel
{
    Magnum::Vector3d m_vel;
    //Vector3s m_vel;
};

struct TCompAccel
{
    Magnum::Vector3d m_acceleration;
};

struct TCompAsteroid
{

};

//struct GravitySource
//{
//    Vector3d pos;
//    double mass;
//};

struct IntegrationStep
{
    std::vector<Magnum::Vector3d> m_accelerations;
    std::vector<Magnum::Vector3d> m_velocities;
    std::vector<Magnum::Vector3d> m_positions;
    //std::vector<GravitySource> m_posMass;
};

class SysEvolution
{
public:
    size_t m_nBodies{0};
    size_t m_nSteps{0};
    std::vector<Satellite> m_bodies;
    std::vector<double> m_masses;
    std::vector<IntegrationStep> m_steps;
    
    void reserve_bodies(size_t bodies);
    void reserve_steps(size_t steps);
};

/**
 * A not very static universe where everything moves constantly
 */
class TrajNBody : public CommonTrajectory<TrajNBody>
{
public:
    static constexpr double m_timestep = 1'000.0f;
    TrajNBody(Universe& universe, Satellite center);
    ~TrajNBody() = default;
    void update();

    std::vector<Magnum::Vector3> get_sat_traj(Satellite sat) const;
    bool just_recomputed() const { return m_justUpdated; }
private:
    template <typename VIEW_T, typename INPUTVIEW_T>
    void update_accelerations(VIEW_T& view, INPUTVIEW_T& posMassView);

    template <typename VIEW_T, typename INPUTVIEW_T>
    void fast_update(VIEW_T& view, INPUTVIEW_T& posMassView);

    size_t m_stepsAhead;
    size_t m_currentStep;
    SysEvolution m_evol;
    bool m_justUpdated;

    template <typename VIEW_T>
    void update_world(VIEW_T& view);

    template <typename VIEW_T>
    void evolve_system(VIEW_T& view);
};

}

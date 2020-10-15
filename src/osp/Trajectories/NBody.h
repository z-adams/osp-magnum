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

struct TCompVel
{
    Magnum::Vector3d m_vel;
    //Vector3s m_vel;
};

/**
 * A static universe where everything stays still
 */
class TrajNBody : public CommonTrajectory<TrajNBody>
{
public:

    TrajNBody(Universe& universe, Satellite center);
    ~TrajNBody() = default;
    void update();
};

}

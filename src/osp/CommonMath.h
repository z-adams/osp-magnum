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
#pragma once

#include "types.h"
#include <type_traits>

namespace osp::math
{

template <typename UINT_T>
constexpr UINT_T uint_2pow(UINT_T exponent)
{
    static_assert(std::is_integral<UINT_T>::value, "Unsigned int required");
    return UINT_T(1) << exponent;
}

template <typename UINT_T>
constexpr bool is_power_of_2(UINT_T value)
{
    static_assert(std::is_integral<UINT_T>::value, "Unsigned int required");
    // Test to see if the value contains more than 1 set bit
    return !(value == 0) && !(value & (value - 1));
}

/**
 * Convert cartesian coordinates to spherical coordinates
 *
 * Takes a cartesian vector (X, Y, Z) and transforms it into spherical
 * coordinates (radius, theta, phi), where theta is the inclination angle, and
 * phi is the azimuthal angle.
 * NOTE: despite residing in "CommonMath.h", these functions are based on the
 * physics convention for spherical coordinate naming conventions, not the
 * math convention (where theta is the azimuthal angle).
 *
 * @param xyz [in] A cartesian vector
 * @return The vector in spherical coordinates {radius, theta, phi}
 */
template <typename T>
inline Vector3 cartesian_to_spherical(Magnum::Math::Vector3<T> xyz)
{
    T r = xyz.length();
    T theta = acos(xyz.z() / r);
    T phi = atan2(xyz.y(), xyz.x());

    return {r, theta, phi};
}

/**
 * Convert spherical coordinates to cartesian coordinates
 *
 * Takes a vector in spherical coordinates (radius, theta, phi), where theta
 * is the inclination angle and phi is the azimuthal angle, and transforms it
 * into cartesian (X, Y, Z) coordiantes
 * NOTE: despite residing in "CommonMath.h", these functions are based on the
 * physics convention for spherical coordinate naming conventions, not the
 * math convention (where theta is the azimuthal angle).
 *
 * @param xyz [in] A spherical vector
 * @return The vector in cartesian coordinates
 */
template <typename T>
inline Vector3 spherical_to_cartesian(Magnum::Math::Vector3<T> rtp)
{
    T r = rtp.x();
    T theta = rtp.y();
    T phi = rtp.z();

    T x = r * cos(theta) * cos(phi);
    T y = r * sin(theta) * sin(phi);
    T z = r * cos(theta);

    return {x, y, z};
}

} // namespace osp::math

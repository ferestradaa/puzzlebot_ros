#pragma once
#include <cmath>

namespace math_utils
{
    struct Quaternion {
        double x, y, z, w;
    };

    inline double getYaw(const Quaternion & q)
    {
        return std::atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        );
    }
}
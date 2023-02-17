#pragma once

#include <array>
#include <cmath>

namespace ganisakta
{
    namespace type
    {
        struct radian
        {
            static double normalize(double rad);
            static double normalize2PI(double rad);
            static double toDegree(double rad);
        };

        struct degree
        {
            static double normalize(double deg);
            static double normalize360(double deg);
            static double toRadian(double deg);
        };
    }
}
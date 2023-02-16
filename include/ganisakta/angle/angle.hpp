#pragma once

#include <array>
#include <cmath>
#include "ganisakta/type/eulerangles.hpp"

namespace ganisakta
{
    namespace angle
    {
        #define S_TWO_PI 6.283185307179586
        #define S_THREEHALF_PI 4.71238898038
        
        double normalizeRad(double rad);
        double normalizeDeg(double deg);
        double radToDeg(double rad);
        double degToRad(double deg);
    }
}
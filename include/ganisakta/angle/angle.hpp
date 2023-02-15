#pragma once

#include <array>
#include <cmath>

namespace ganisakta
{
    namespace angle
    {
        struct Quaternion
        {
            double w, x, y, z;
        };

        struct EulerAngles
        {
            double yaw, pitch, roll;
        };

        #define S_TWO_PI 6.283185307179586
        #define S_THREEHALF_PI 4.71238898038

        EulerAngles quaternionToEulerInDeg(double w, double x, double y, double z);
        EulerAngles quaternionToEulerInRad(double w, double x, double y, double z);

        Quaternion eulerToQuaternionInDeg(double yaw, double pitch, double roll);
        Quaternion eulerToQuaternionInRad(double yaw, double pitch, double roll);

        double normalizeRad(double rad);
        double normalizeDeg(double deg);
        double radToDeg(double rad);
        double degToRad(double deg);
    }
}
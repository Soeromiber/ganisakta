#include "ganisakta/type/angle.hpp"

namespace ganisakta
{
    namespace type
    {
        // Convert radian into 0 - +2pi range
        double radian::normalize2PI(double rad){
            rad = fmod(rad, M_PI * 2);
            if (rad < 0)
                rad += M_PI * 2;
            return rad;
        }

        // Convert radian into -pi - +pi range
        double radian::normalize(double rad){
            rad = fmod(rad + M_PI,M_PI * 2);
            if (rad < 0)
                rad += M_PI * 2;
            return rad - M_PI;
        }

        //Convert degree into 0 - +360 range
        double degree::normalize360(double deg){
            deg = fmod(deg,360);
            if (deg < 0)
                deg += 360;
            return deg;
        }

        //Convert degree into -180 - +180 range
        double degree::normalize(double deg){
            deg = fmod(deg + 180,360);
            if (deg < 0)
                deg += 360;
            return deg - 180;
        }

        double radian::toDegree(double rad){
            return rad * 180 / M_PI;
        }

        double degree::toRadian(double deg){
            return deg * M_PI / 180;
        }
    }
}
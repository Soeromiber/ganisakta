#include "ganisakta/angle/angle.hpp"

namespace ganisakta
{
    namespace angle
    {
        // Source https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        //Convert radian into 0 - 2pi range
        double normalizeRad(double rad){
            while(rad >= S_TWO_PI){
                rad -= S_TWO_PI;
            }
            while(rad < 0){
                rad += S_TWO_PI;
            }
            return rad;
        }

        //Convert degree into 0 - 360 range
        double normalizeDeg(double deg){
            while(deg >= 360){
                deg -= 360;
            }
            while(deg < 0){
                deg += 360;
            }
            return deg;
        }

        double radToDeg(double rad){
            return rad * 180 / M_PI;
        }

        double degToRad(double deg){
            return deg * M_PI / 180;
        }
    }
}
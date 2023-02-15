#include "ganisakta/angle/angle.hpp"

namespace ganisakta
{
    namespace angle
    {
        // Source https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        EulerAngles quaternionToEulerInDeg(double w, double x, double y, double z)
        {
            double q2sqr = y * y;
            double t0 = -2.0 * (q2sqr + z * z) + 1.0;
            double t1 = +2.0 * (x * y + w * z);
            double t2 = -2.0 * (x * z - w * y);
            double t3 = +2.0 * (y * z + w * x);
            double t4 = -2.0 * (x * x + q2sqr) + 1.0;

            EulerAngles e;
            e.pitch = radToDeg(asin(t2));
            e.roll = radToDeg(atan2(t3, t4));
            e.yaw = radToDeg(atan2(t1, t0));
            return e;
        }

        EulerAngles quaternionToEulerInRad(double w, double x, double y, double z)
        {
            double q2sqr = y * y;
            double t0 = -2.0 * (q2sqr + z * z) + 1.0;
            double t1 = +2.0 * (x * y + w * z);
            double t2 = -2.0 * (x * z - w * y);
            double t3 = +2.0 * (y * z + w * x);
            double t4 = -2.0 * (x * x + q2sqr) + 1.0;

            EulerAngles e;
            e.pitch = asin(t2);
            e.roll = atan2(t3, t4);
            e.yaw = atan2(t1, t0);
            return e;
        }

        Quaternion eulerToQuaternionInDeg(double yaw, double pitch, double roll)
        {
            double cy = cos(degToRad(yaw) * 0.5);
            double sy = sin(degToRad(yaw) * 0.5);
            double cp = cos(degToRad(pitch) * 0.5);
            double sp = sin(degToRad(pitch) * 0.5);
            double cr = cos(degToRad(roll) * 0.5);
            double sr = sin(degToRad(roll) * 0.5);

            Quaternion q;
            q.w = cy * cp * cr + sy * sp * sr;
            q.x = cy * cp * sr - sy * sp * cr;
            q.y = sy * cp * sr + cy * sp * cr;
            q.z = sy * cp * cr - cy * sp * sr;
            return q;
        }

        Quaternion eulerToQuaternionInRad(double yaw, double pitch, double roll)
        {
            double cy = cos(yaw * 0.5);
            double sy = sin(yaw * 0.5);
            double cp = cos(pitch * 0.5);
            double sp = sin(pitch * 0.5);
            double cr = cos(roll * 0.5);
            double sr = sin(roll * 0.5);

            Quaternion q;
            q.w = cy * cp * cr + sy * sp * sr;
            q.x = cy * cp * sr - sy * sp * cr;
            q.y = sy * cp * sr + cy * sp * cr;
            q.z = sy * cp * cr - cy * sp * sr;
            return q;
        }

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
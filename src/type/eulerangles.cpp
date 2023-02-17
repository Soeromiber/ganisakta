#include "ganisakta/type/eulerangles.hpp"

namespace ganisakta
{
    namespace type
    {
        // Static
        EulerAngles EulerAngles::fromQuaternion(const Quaternion& quaternion)
        {
            EulerAngles angles;

            // roll (x-axis rotation)
            double sinr_cosp = 2 * (quaternion.w * quaternion.x + quaternion.y * quaternion.z);
            double cosr_cosp = 1 - 2 * (quaternion.x * quaternion.x + quaternion.y * quaternion.y);
            angles.roll = std::atan2(sinr_cosp, cosr_cosp);

            // pitch (y-axis rotation)
            double sinp = 2 * (quaternion.w * quaternion.y - quaternion.z * quaternion.x);
            if (std::abs(sinp) >= 1)
                angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
            else
                angles.pitch = std::asin(sinp);

            // yaw (z-axis rotation)
            double siny_cosp = 2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y);
            double cosy_cosp = 1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z);
            angles.yaw = std::atan2(siny_cosp, cosy_cosp);

            radian::normalize(angles.roll);
            radian::normalize(angles.pitch);
            radian::normalize(angles.yaw);

            // std::cerr << "EulerAngles::fromQuaternion: " << angles.roll << ", " << angles.pitch << ", " << angles.yaw << std::endl;

            return angles;
        }

        EulerAngles EulerAngles::toEulerAnglesDegree(const EulerAngles& ypr)
        {
            return EulerAngles(ypr.roll * 180 / M_PI, ypr.pitch * 180 / M_PI, ypr.yaw * 180 / M_PI);
        }        
            
        // Non-static
        EulerAngles::EulerAngles()
        {
            roll = 0;
            pitch = 0;
            yaw = 0;
        }

        EulerAngles::EulerAngles(double roll, double pitch, double yaw)
        {
            this->roll = roll;
            this->pitch = pitch;
            this->yaw = yaw;
        }

        EulerAngles::EulerAngles(const geometry_msgs::Vector3& ypr)
        {
            roll = ypr.x;
            pitch = ypr.y;
            yaw = ypr.z;
        }

        EulerAngles::EulerAngles(const std::array<double, AXES>& rpy)
        {
            roll = rpy[0];
            pitch = rpy[1];
            yaw = rpy[2];
        }

        EulerAngles EulerAngles::operator+(const EulerAngles& ypr)
        {
            return EulerAngles(roll + ypr.roll, pitch + ypr.pitch, yaw + ypr.yaw);
        }

        EulerAngles EulerAngles::operator-(const EulerAngles& ypr)
        {
            return EulerAngles(roll - ypr.roll, pitch - ypr.pitch, yaw - ypr.yaw);
        }

        EulerAngles EulerAngles::operator*(const double& scalar)
        {
            return EulerAngles(roll * scalar, pitch * scalar, yaw * scalar);
        }

        EulerAngles EulerAngles::operator/(const double& scalar)
        {
            return EulerAngles(roll / scalar, pitch / scalar, yaw / scalar);
        }

        EulerAngles EulerAngles::operator+=(const EulerAngles& ypr)
        {
            roll += ypr.roll;
            pitch += ypr.pitch;
            yaw += ypr.yaw;
            return *this;
        }

        EulerAngles EulerAngles::operator-=(const EulerAngles& ypr)
        {
            roll -= ypr.roll;
            pitch -= ypr.pitch;
            yaw -= ypr.yaw;
            return *this;
        }

        EulerAngles EulerAngles::operator*=(const double& scalar)
        {
            roll *= scalar;
            pitch *= scalar;
            yaw *= scalar;
            return *this;
        }

        EulerAngles EulerAngles::operator/=(const double& scalar)
        {
            roll /= scalar;
            pitch /= scalar;
            yaw /= scalar;
            return *this;
        }

        bool EulerAngles::operator==(const EulerAngles& ypr) const
        {
            return roll == ypr.roll && pitch == ypr.pitch && yaw == ypr.yaw;
        }

        bool EulerAngles::operator!=(const EulerAngles& ypr)
        {
            return roll != ypr.roll || pitch != ypr.pitch || yaw != ypr.yaw;
        }

        double& EulerAngles::operator[](const int& index)
        {
            switch (index)
            {
                case 0:
                    return roll;
                case 1:
                    return pitch;
                case 2:
                    return yaw;
                default:
                    throw std::out_of_range("Euler angles index out of range");
            }
        }

        geometry_msgs::Vector3 EulerAngles::toVector3()
        {
            geometry_msgs::Vector3 ypr;
            ypr.x = roll;
            ypr.y = pitch;
            ypr.z = yaw;
            return ypr;
        }

        std::array<double, AXES> EulerAngles::toArray()
        {
            std::array<double, AXES> rpy;
            rpy[0] = roll;
            rpy[1] = pitch;
            rpy[2] = yaw;
            return rpy;
        }

        Quaternion EulerAngles::toQuaternion()
        {
            double cy = cos(yaw * 0.5);
            double sy = sin(yaw * 0.5);
            double cp = cos(pitch * 0.5);
            double sp = sin(pitch * 0.5);
            double cr = cos(roll * 0.5);
            double sr = sin(roll * 0.5);

            Quaternion q;
            q.w = cr * cp * cy + sr * sp * sy;
            q.x = sr * cp * cy - cr * sp * sy;
            q.y = cr * sp * cy + sr * cp * sy;
            q.z = cr * cp * sy - sr * sp * cy;
            return q;
        }

        EulerAngles EulerAngles::toEulerAnglesDegree()
        {
            return EulerAngles(roll * 180 / M_PI, pitch * 180 / M_PI, yaw * 180 / M_PI);
        }
    }
}
#include "ganisakta/quaternion/quaternion.hpp"

namespace ganisakta
{
    namespace quaternion
    {
        angle::Angle Quaternion::toEulerAngles(const Quaternion& quaternion)
        {
            angle::Angle angle;

            // roll (x-axis rotation)
            double sinr_cosp = 2 * (quaternion.w * quaternion.x + quaternion.y * quaternion.z);
            double cosr_cosp = 1 - 2 * (quaternion.x * quaternion.x + quaternion.y * quaternion.y);
            angle.roll = std::atan2(sinr_cosp, cosr_cosp);

            // pitch (y-axis rotation)
            double sinp = 2 * (quaternion.w * quaternion.y - quaternion.z * quaternion.x);
            if (std::abs(sinp) >= 1)
                angle.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
            else
                angle.pitch = std::asin(sinp);

            // yaw (z-axis rotation)
            double siny_cosp = 2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y);
            double cosy_cosp = 1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z);
            angle.yaw = std::atan2(siny_cosp, cosy_cosp);

            return angle;
        }

        geometry_msgs::Quaternion Quaternion::toQuaternion(const Quaternion& quaternion)
        {
            geometry_msgs::Quaternion q;
            q.x = quaternion.x;
            q.y = quaternion.y;
            q.z = quaternion.z;
            q.w = quaternion.w;
            return q;
        }
        
        std::array<double, AXES> Quaternion::toArray(const Quaternion& quaternion)
        {
            angle::Angle angle = Quaternion::toEulerAngles(quaternion);
            return angle::Angle::toArray(angle);
        }

        Quaternion::Quaternion(const std::array<double, AXES>& rpy)
        {
            double cy = cos(rpy[2] * 0.5);
            double sy = sin(rpy[2] * 0.5);
            double cp = cos(rpy[1] * 0.5);
            double sp = sin(rpy[1] * 0.5);
            double cr = cos(rpy[0] * 0.5);
            double sr = sin(rpy[0] * 0.5);

            x = sr * cp * cy - cr * sp * sy;
            y = cr * sp * cy + sr * cp * sy;
            z = cr * cp * sy - sr * sp * cy;
            w = cr * cp * cy + sr * sp * sy;
        }

        Quaternion::Quaternion(const angle::Angle& rpy)
        {
            double cy = cos(rpy.yaw * 0.5);
            double sy = sin(rpy.yaw * 0.5);
            double cp = cos(rpy.pitch * 0.5);
            double sp = sin(rpy.pitch * 0.5);
            double cr = cos(rpy.roll * 0.5);
            double sr = sin(rpy.roll * 0.5);

            x = sr * cp * cy - cr * sp * sy;
            y = cr * sp * cy + sr * cp * sy;
            z = cr * cp * sy - sr * sp * cy;
            w = cr * cp * cy + sr * sp * sy;
        }

        Quaternion Quaternion::operator+(const Quaternion& quaternion)
        {
            return Quaternion(x + quaternion.x, y + quaternion.y, z + quaternion.z, w + quaternion.w);
        }

        Quaternion Quaternion::operator-(const Quaternion& quaternion)
        {
            return Quaternion(x - quaternion.x, y - quaternion.y, z - quaternion.z, w - quaternion.w);
        }

        Quaternion Quaternion::operator*(const double& scalar)
        {
            return Quaternion(x * scalar, y * scalar, z * scalar, w * scalar);
        }

        Quaternion Quaternion::operator/(const double& scalar)
        {
            return Quaternion(x / scalar, y / scalar, z / scalar, w / scalar);
        }

        Quaternion& Quaternion::operator+=(const Quaternion& quaternion)
        {
            x += quaternion.x;
            y += quaternion.y;
            z += quaternion.z;
            w += quaternion.w;
            return *this;
        }

        Quaternion& Quaternion::operator-=(const Quaternion& quaternion)
        {
            x -= quaternion.x;
            y -= quaternion.y;
            z -= quaternion.z;
            w -= quaternion.w;
            return *this;
        }

        Quaternion& Quaternion::operator*=(const double& scalar)
        {
            x *= scalar;
            y *= scalar;
            z *= scalar;
            w *= scalar;
            return *this;
        }

        Quaternion& Quaternion::operator/=(const double& scalar)
        {
            x /= scalar;
            y /= scalar;
            z /= scalar;
            w /= scalar;
            return *this;
        }

        bool Quaternion::operator==(const Quaternion& quaternion) const
        {
            return std::fabs(x - quaternion.x) <= 0.001 && std::fabs(y - quaternion.y) <= 0.001 && std::fabs(z - quaternion.z) <= 0.001 && std::fabs(w - quaternion.w) <= 0.001;
        }

        bool Quaternion::operator!=(const Quaternion& quaternion)
        {
            return x != quaternion.x || y != quaternion.y || z != quaternion.z || w != quaternion.w;
        }

        double& Quaternion::operator[](const int& index)
        {
            switch (index)
            {
                case 0:
                    return x;
                case 1:
                    return y;
                case 2:
                    return z;
                case 3:
                    return w;
                default:
                    throw std::out_of_range("Index out of range");
            }
        }

        geometry_msgs::Quaternion Quaternion::toQuaternion()
        {
            geometry_msgs::Quaternion quaternion;
            quaternion.x = x;
            quaternion.y = y;
            quaternion.z = z;
            quaternion.w = w;
            return quaternion;
        }

        std::array<double, AXES> Quaternion::toArray()
        {
            std::array<double, AXES> quaternion;
            quaternion[0] = x;
            quaternion[1] = y;
            quaternion[2] = z;
            quaternion[3] = w;
            return quaternion;
        }

        angle::Angle Quaternion::toEulerAngles()
        {
            return Quaternion::toEulerAngles(*this);
        }

        Quaternion Quaternion::conjugate()
        {
            x = -x;
            y = -y;
            z = -z;
            return *this;
        }
    }
}
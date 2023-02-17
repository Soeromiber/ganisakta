#include "ganisakta/type/quaternion.hpp"

namespace ganisakta
{
    namespace type
    {
        // Static
        EulerAngles Quaternion::toEulerAngles(const Quaternion& quaternion)
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

            return angles;
        }

        // Non-static
        Quaternion::Quaternion()
        {
            x = 0;
            y = 0;
            z = 0;
            w = 0;
        }

        Quaternion::Quaternion(double x, double y, double z, double w)
        {
            this->x = x;
            this->y = y;
            this->z = z;
            this->w = w;
        }

        Quaternion::Quaternion(const EulerAngles& rpy)
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

        Quaternion::Quaternion(const geometry_msgs::Quaternion& quaternion)
        {
            x = quaternion.x;
            y = quaternion.y;
            z = quaternion.z;
            w = quaternion.w;
        }

        Quaternion::Quaternion(const std::array<double, AXES>& quaternion)
        {
            x = quaternion[0];
            y = quaternion[1];
            z = quaternion[2];
            w = quaternion[3];
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

        EulerAngles Quaternion::toEulerAngles()
        {
           
            EulerAngles angles;

            // roll (x-axis rotation)
            double sinr_cosp = 2 * (w * x + y * z);
            double cosr_cosp = 1 - 2 * (x * x + y * y);
            angles.roll = std::atan2(sinr_cosp, cosr_cosp);

            // pitch (y-axis rotation)
            double sinp = std::sqrt(1 + 2 * (w * y - x * z));
            double cosp = std::sqrt(1 - 2 * (w * y - x * z));
            angles.pitch = 2 * std::atan2(sinp, cosp) - M_PI / 2;

            // yaw (z-axis rotation)
            double siny_cosp = 2 * (w * z + x * y);
            double cosy_cosp = 1 - 2 * (y * y + z * z);
            angles.yaw = std::atan2(siny_cosp, cosy_cosp);

            return angles;
        }

        Quaternion Quaternion::conjugate()
        {
            x = -x;
            y = -y;
            z = -z;
            return *this;
        }

        // std::ostream& operator<<(std::ostream& os, const Quaternion& quaternion)
        // {
        //     os << "Quaternion(" << quaternion.w << ", " << quaternion.x << ", " << quaternion.y << ", " << quaternion.z << ")";
        //     return os;
        // }
    }
}
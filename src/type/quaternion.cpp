#include "ganisakta/type/quaternion.hpp"

namespace ganisakta
{
    namespace type
    {
        // Static
        EulerAngles Quaternion::toEulerAngles(const Quaternion& quaternion)
        {
            double roll = atan2(2 * (quaternion.w * quaternion.x + quaternion.y * quaternion.z), 1 - 2 * (quaternion.x * quaternion.x + quaternion.y * quaternion.y));
            double pitch = asin(2 * (quaternion.w * quaternion.y - quaternion.z * quaternion.x));
            double yaw = atan2(2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y), 1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z));
            return EulerAngles(roll, pitch, yaw);
        }

        // Non-static
        Quaternion::Quaternion()
        {
            w = 0;
            x = 0;
            y = 0;
            z = 0;
        }

        Quaternion::Quaternion(double w, double x, double y, double z)
        {
            this->w = w;
            this->x = x;
            this->y = y;
            this->z = z;
        }

        Quaternion::Quaternion(const geometry_msgs::Quaternion& quaternion)
        {
            w = quaternion.w;
            x = quaternion.x;
            y = quaternion.y;
            z = quaternion.z;
        }

        Quaternion::Quaternion(const std::array<double, AXES>& quaternion)
        {
            w = quaternion[0];
            x = quaternion[1];
            y = quaternion[2];
            z = quaternion[3];
        }

        Quaternion Quaternion::operator+(const Quaternion& quaternion)
        {
            return Quaternion(w + quaternion.w, x + quaternion.x, y + quaternion.y, z + quaternion.z);
        }

        Quaternion Quaternion::operator-(const Quaternion& quaternion)
        {
            return Quaternion(w - quaternion.w, x - quaternion.x, y - quaternion.y, z - quaternion.z);
        }

        Quaternion Quaternion::operator*(const double& scalar)
        {
            return Quaternion(w * scalar, x * scalar, y * scalar, z * scalar);
        }

        Quaternion Quaternion::operator/(const double& scalar)
        {
            return Quaternion(w / scalar, x / scalar, y / scalar, z / scalar);
        }

        Quaternion Quaternion::operator+=(const Quaternion& quaternion)
        {
            w += quaternion.w;
            x += quaternion.x;
            y += quaternion.y;
            z += quaternion.z;
            return *this;
        }

        Quaternion Quaternion::operator-=(const Quaternion& quaternion)
        {
            w -= quaternion.w;
            x -= quaternion.x;
            y -= quaternion.y;
            z -= quaternion.z;
            return *this;
        }

        Quaternion Quaternion::operator*=(const double& scalar)
        {
            w *= scalar;
            x *= scalar;
            y *= scalar;
            z *= scalar;
            return *this;
        }

        Quaternion Quaternion::operator/=(const double& scalar)
        {
            w /= scalar;
            x /= scalar;
            y /= scalar;
            z /= scalar;
            return *this;
        }

        bool Quaternion::operator==(const Quaternion& quaternion)
        {
            return w == quaternion.w && x == quaternion.x && y == quaternion.y && z == quaternion.z;
        }

        bool Quaternion::operator!=(const Quaternion& quaternion)
        {
            return w != quaternion.w || x != quaternion.x || y != quaternion.y || z != quaternion.z;
        }

        double& Quaternion::operator[](const int& index)
        {
            switch (index)
            {
                case 0:
                    return w;
                case 1:
                    return x;
                case 2:
                    return y;
                case 3:
                    return z;
                default:
                    throw std::out_of_range("Index out of range");
            }
        }

        geometry_msgs::Quaternion Quaternion::toQuaternion()
        {
            geometry_msgs::Quaternion quaternion;
            quaternion.w = w;
            quaternion.x = x;
            quaternion.y = y;
            quaternion.z = z;
            return quaternion;
        }

        std::array<double, AXES> Quaternion::toArray()
        {
            std::array<double, AXES> quaternion;
            quaternion[0] = w;
            quaternion[1] = x;
            quaternion[2] = y;
            quaternion[3] = z;
            return quaternion;
        }

        EulerAngles Quaternion::toEulerAngles()
        {
            EulerAngles euler_angles;
            euler_angles.roll = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
            euler_angles.pitch = asin(2 * (w * y - z * x));
            euler_angles.yaw = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
            return euler_angles;
        }

        // std::ostream& operator<<(std::ostream& os, const Quaternion& quaternion)
        // {
        //     os << "Quaternion(" << quaternion.w << ", " << quaternion.x << ", " << quaternion.y << ", " << quaternion.z << ")";
        //     return os;
        // }
    }
}
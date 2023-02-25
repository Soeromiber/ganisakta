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

        Quaternion Quaternion::conjugate(const Quaternion& quaternion)
        {
            Quaternion q;
            q.x = -quaternion.x;
            q.y = -quaternion.y;
            q.z = -quaternion.z;
            q.w = quaternion.w;
            return q;
        }
        
        double Quaternion::norm(const Quaternion& quaternion)
        {
            return std::sqrt(quaternion.x * quaternion.x + quaternion.y * quaternion.y + quaternion.z * quaternion.z + quaternion.w * quaternion.w);
        }

        Quaternion Quaternion::normalize(const Quaternion& quaternion)
        {
            double n = Quaternion::norm(quaternion);
            Quaternion q;
            q.x = quaternion.x / n;
            q.y = quaternion.y / n;
            q.z = quaternion.z / n;
            q.w = quaternion.w / n;
            return q;
        }

        Quaternion Quaternion::createRotationQuat(const double& x, const double& y, const double& z, const double& a)
        {
            Quaternion q;
            double factor = std::sin( a / 2.0 );
            q.x = x * factor;
            q.y = y * factor;
            q.z = z * factor;
            q.w = std::cos(a / 2);
            return q;
        }

        Quaternion Quaternion::inverse(const Quaternion& quaternion)
        {
            Quaternion conjugate = Quaternion::conjugate(quaternion);
            double norm = Quaternion::norm(quaternion);
            return conjugate / (norm * norm);
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

        Quaternion Quaternion::operator*(const Quaternion& quaternion)
        {
            // Quaternion q;
            // q.x = w * quaternion.x + x * quaternion.w + y * quaternion.z - z * quaternion.y;
            // q.y = w * quaternion.y + y * quaternion.w + z * quaternion.x - x * quaternion.z;
            // q.z = w * quaternion.z + z * quaternion.w + x * quaternion.y - y * quaternion.x;
            // q.w = w * quaternion.w - x * quaternion.x - y * quaternion.y - z * quaternion.z;
            // return q;

            // return Quaternion(
            //                 w*quaternion.x + x*quaternion.w + y*quaternion.z - z*quaternion.y,
            //                 w*quaternion.y - x*quaternion.z + y*quaternion.w + z*quaternion.x,
            //                 w*quaternion.z + x*quaternion.y - y*quaternion.x + z*quaternion.w,
            //                 w*quaternion.w - x*quaternion.x - y*quaternion.y - z*quaternion.z
            // );

            // Quaternion q;
            // q.w = w * quaternion.w - x * quaternion.x - y * quaternion.y - z * quaternion.z;
            // q.x = w * quaternion.x + x * quaternion.w + y * quaternion.z - z * quaternion.y;
            // q.y = w * quaternion.y - x * quaternion.z + y * quaternion.w + z * quaternion.x;
            // q.z = w * quaternion.z + x * quaternion.y - y * quaternion.x + z * quaternion.w;
            // return q;

            // return Quaternion(
            //     w*quaternion.x + x*quaternion.w + y*quaternion.z - z*quaternion.y,
            //     w*quaternion.y - x*quaternion.z + y*quaternion.w + z*quaternion.x,
            //     w*quaternion.z + x*quaternion.y - y*quaternion.x + z*quaternion.w,
            //     w*quaternion.w - x*quaternion.x - y*quaternion.y - z*quaternion.z
            // );

            return Quaternion(
                w*quaternion.x + x*quaternion.w + y*quaternion.z - z*quaternion.y,
                w*quaternion.y - x*quaternion.z + y*quaternion.w + z*quaternion.x,
                w*quaternion.z + x*quaternion.y - y*quaternion.x + z*quaternion.w,
                w*quaternion.w - x*quaternion.x - y*quaternion.y - z*quaternion.z
            );
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

        Quaternion& Quaternion::operator*=(const Quaternion& quaternion)
        {
            *this = *this * quaternion;
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

        std::ostream& operator<<(std::ostream& os, const Quaternion& quaternion)
        {
            os << "Quaternion(" << quaternion.x << ", " << quaternion.y << ", " << quaternion.z << ", " << quaternion.w << ")";
            return os;
        }

        bool Quaternion::operator==(const Quaternion& quaternion) const
        {
            return std::fabs(x - quaternion.x) <= 0.001 && std::fabs(y - quaternion.y) <= 0.001 && std::fabs(z - quaternion.z) <= 0.001 && std::fabs(w - quaternion.w) <= 0.001;
        }

        bool Quaternion::operator!=(const Quaternion& quaternion) const
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

        double Quaternion::norm()
        {
            return Quaternion::norm(*this);
        }

        Quaternion Quaternion::normalize()
        {
            Quaternion normalized = Quaternion::normalize(*this);
            x = normalized.x;
            y = normalized.y;
            z = normalized.z;
            w = normalized.w;
            return *this;
        }

        Quaternion Quaternion::inverse()
        {
            Quaternion inverse = Quaternion::inverse(*this);
            x = inverse.x;
            y = inverse.y;
            z = inverse.z;
            w = inverse.w;
            return *this;
        }

        Quaternion Quaternion::rotate(const double& rotX, const double& rotY, const double& rotZ)
        {
            // Quaternion qx = Quaternion::createRotationQuat(1, 0, 0, rotX).normalize();
            // std::cerr << "qx: " << qx << std::endl;
            // Quaternion qy = Quaternion::createRotationQuat(0, 1, 0, rotY).normalize();
            // std::cerr << "qy: " << qy << std::endl;
            // Quaternion qz = Quaternion::createRotationQuat(0, 0, 1, rotZ).normalize();
            // std::cerr << "qz: " << qz << std::endl;
            // *this = qz * qy * qx * (*this) * qx.conjugate() * qy.conjugate() * qz.conjugate();

            Quaternion qx = Quaternion::createRotationQuat(1, 0, 0, rotX).normalize();
            std::cerr << "qx: " << qx << std::endl;
            Quaternion qy = Quaternion::createRotationQuat(0, 1, 0, rotY).normalize();
            std::cerr << "qy: " << qy << std::endl;
            Quaternion qz = Quaternion::createRotationQuat(0, 0, 1, rotZ).normalize();
            std::cerr << "qz: " << qz << std::endl;

            *this = qz * (*this) * Quaternion::conjugate(qz);
            std::cerr << "(*x): " << (*this) << std::endl;
            *this = qy * (*this) * Quaternion::conjugate(qy);
            std::cerr << "(*y): " << (*this) << std::endl;
            *this = qx * (*this) * Quaternion::conjugate(qx);
            std::cerr << "(*z): " << (*this) << std::endl;

            return *this;
        }
    }
}
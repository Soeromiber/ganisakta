#include "ganisakta/angle/angle.hpp"

namespace ganisakta
{
    namespace angle
    {
        geometry_msgs::Vector3 Angle::toVector3(const Angle& angle)
        {
            geometry_msgs::Vector3 vector3;
            vector3.x = angle.roll;
            vector3.y = angle.pitch;
            vector3.z = angle.yaw;
            return vector3;
        }

        std::array<double, AXES> Angle::toArray(const Angle& angle)
        {
            std::array<double, AXES> array;
            array[0] = angle.roll;
            array[1] = angle.pitch;
            array[2] = angle.yaw;
            return array;
        }

        Angle Angle::fromVector3(const geometry_msgs::Vector3& vector3)
        {
            return Angle(vector3.x, vector3.y, vector3.z);
        }

        Angle Angle::fromArray(const std::array<double, AXES>& array)
        {
            return Angle(array[0], array[1], array[2]);
        }

        Angle Angle::normalize(const Angle& angle)
        {
            return Angle(
                std::fmod(angle.roll, 2 * M_PI),
                std::fmod(angle.pitch, 2 * M_PI),
                std::fmod(angle.yaw, 2 * M_PI)
            );
        }

        quaternion::Quaternion Angle::toQuaternion(const Angle& angle)
        {
            double cy = cos(angle.yaw * 0.5);
            double sy = sin(angle.yaw * 0.5);
            double cp = cos(angle.pitch * 0.5);
            double sp = sin(angle.pitch * 0.5);
            double cr = cos(angle.roll * 0.5);
            double sr = sin(angle.roll * 0.5);

            quaternion::Quaternion q;
            q.w = cr * cp * cy + sr * sp * sy;
            q.x = sr * cp * cy - cr * sp * sy;
            q.y = cr * sp * cy + sr * cp * sy;
            q.z = cr * cp * sy - sr * sp * cy;
            return q;
        }

        Angle Angle::fromQuaternion(const quaternion::Quaternion& quaternion)
        {
            Angle angle = quaternion::Quaternion::toEulerAngles(quaternion);
            return angle;
        }

        Angle Angle::toEulerAnglesDegree(const Angle& angle)
        {
            return Angle(angle.roll * 180 / M_PI, angle.pitch * 180 / M_PI, angle.yaw * 180 / M_PI);
        }

        Angle Angle::operator+(const Angle& angle)
        {
            return Angle(this->roll + angle.roll, this->pitch + angle.pitch, this->yaw + angle.yaw);
        }

        Angle Angle::operator-(const Angle& angle)
        {
            return Angle(this->roll - angle.roll, this->pitch - angle.pitch, this->yaw - angle.yaw);
        }

        Angle Angle::operator*(const double& scalar)
        {
            return Angle(this->roll * scalar, this->pitch * scalar, this->yaw * scalar);
        }

        Angle Angle::operator/(const double& scalar)
        {
            return Angle(this->roll / scalar, this->pitch / scalar, this->yaw / scalar);
        }

        Angle Angle::operator+=(const Angle& angle)
        {
            this->roll += angle.roll;
            this->pitch += angle.pitch;
            this->yaw += angle.yaw;
            return *this;
        }

        Angle Angle::operator-=(const Angle& angle)
        {
            this->roll -= angle.roll;
            this->pitch -= angle.pitch;
            this->yaw -= angle.yaw;
            return *this;
        }

        Angle Angle::operator*=(const double& scalar)
        {
            this->roll *= scalar;
            this->pitch *= scalar;
            this->yaw *= scalar;
            return *this;
        }

        Angle Angle::operator/=(const double& scalar)
        {
            this->roll /= scalar;
            this->pitch /= scalar;
            this->yaw /= scalar;
            return *this;
        }

        bool Angle::operator==(const Angle& angle)
        {
            return std::fabs(this->roll - angle.roll) < EPSILON && std::fabs(this->pitch - angle.pitch) < EPSILON && std::fabs(this->yaw - angle.yaw) < EPSILON;
        }

        bool Angle::operator!=(const Angle& angle)
        {
            return !(*this == angle);
        }

        double& Angle::operator[](const int& index)
        {
            switch (index)
            {
                case 0:
                    return this->roll;
                case 1:
                    return this->pitch;
                case 2:
                    return this->yaw;
                default:
                    throw std::out_of_range("Angle index out of range");
            }
        }

        geometry_msgs::Vector3 Angle::toVector3()
        {
            return Angle::toVector3(*this);
        }

        std::array<double, AXES> Angle::toArray()
        {
            return Angle::toArray(*this);
        }

        quaternion::Quaternion Angle::toQuaternion()
        {
            return Angle::toQuaternion(*this);
        }

        Angle Angle::toEulerAnglesDegree()
        {
            return Angle::toEulerAnglesDegree(*this);
        }

        Angle Angle::normalize()
        {
            return Angle::normalize(*this);
        }
    }
}
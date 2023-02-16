#include "ganisakta/type/euler.hpp"

namespace ganisakta
{
    namespace type
    {
        // Static
        const Euler Euler::ZERO = Euler(0, 0, 0);
        const Euler Euler::ONE = Euler(1, 1, 1);
        const Euler Euler::FORWARD = Euler(1, 0, 0);
        const Euler Euler::BACKWARD = Euler(-1, 0, 0);
        const Euler Euler::UP = Euler(0, 0, 1);
        const Euler Euler::DOWN = Euler(0, 0, -1);
        const Euler Euler::LEFT = Euler(0, 1, 0);
        const Euler Euler::RIGHT = Euler(0, -1, 0);

        double Euler::distance(const Euler& euler1, const Euler& euler2)
        {
            return sqrt(pow(euler1.x - euler2.x, 2) + pow(euler1.y - euler2.y, 2) + pow(euler1.z - euler2.z, 2));
        }

        double Euler::dot(const Euler& euler1, const Euler& euler2)
        {
            return euler1.x * euler2.x + euler1.y * euler2.y + euler1.z * euler2.z;
        }

        Euler Euler::cross(const Euler& euler1, const Euler& euler2)
        {
            return Euler(euler1.y * euler2.z - euler1.z * euler2.y, euler1.z * euler2.x - euler1.x * euler2.z, euler1.x * euler2.y - euler1.y * euler2.x);
        }

        double Euler::angle(const Euler& euler1, const Euler& euler2)
        {
            return acos(dot(euler1, euler2) / (distance(euler1, Euler::ZERO) * distance(euler2, Euler::ZERO)));
        }

        Euler Euler::normalize(const Euler& euler)
        {
            double length = sqrt(pow(euler.x, 2) + pow(euler.y, 2) + pow(euler.z, 2));
            return Euler(euler.x / length, euler.y / length, euler.z / length);
        }

        geometry_msgs::Vector3 Euler::toVector3(const Euler& euler)
        {
            geometry_msgs::Vector3 vector;
            vector.x = euler.x;
            vector.y = euler.y;
            vector.z = euler.z;
            return vector;
        }

        std::array<double, AXES> Euler::toArray(const Euler& euler)
        {
            std::array<double, AXES> array;
            array[0] = euler.x;
            array[1] = euler.y;
            array[2] = euler.z;
            return array;
        }

        // Non-static

        Euler::Euler()
        {
            this->x = 0;
            this->y = 0;
            this->z = 0;
        }

        Euler::Euler(double x, double y, double z)
        {
            this->x = x;
            this->y = y;
            this->z = z;
        }

        Euler::Euler(const geometry_msgs::Vector3& euler)
        {
            this->x = euler.x;
            this->y = euler.y;
            this->z = euler.z;
        }

        Euler::Euler(const std::array<double, AXES>& euler)
        {
            this->x = euler[0];
            this->y = euler[1];
            this->z = euler[2];
        }

        Euler Euler::zero()
        {
            return Euler(0, 0, 0);
        }

        Euler Euler::operator+(const Euler& euler)
        {
            return Euler(this->x + euler.x, this->y + euler.y, this->z + euler.z);
        }

        Euler Euler::operator-(const Euler& euler)
        {
            return Euler(this->x - euler.x, this->y - euler.y, this->z - euler.z);
        }

        Euler Euler::operator*(const double& scalar)
        {
            return Euler(this->x * scalar, this->y * scalar, this->z * scalar);
        }

        Euler Euler::operator/(const double& scalar)
        {
            return Euler(this->x / scalar, this->y / scalar, this->z / scalar);
        }

        Euler Euler::operator+=(const Euler& euler)
        {
            this->x += euler.x;
            this->y += euler.y;
            this->z += euler.z;
            return *this;
        }

        Euler Euler::operator-=(const Euler& euler)
        {
            this->x -= euler.x;
            this->y -= euler.y;
            this->z -= euler.z;
            return *this;
        }

        Euler Euler::operator*=(const double& scalar)
        {
            this->x *= scalar;
            this->y *= scalar;
            this->z *= scalar;
            return *this;
        }

        Euler Euler::operator/=(const double& scalar)
        {
            this->x /= scalar;
            this->y /= scalar;
            this->z /= scalar;
            return *this;
        }

        bool Euler::operator==(const Euler& euler)
        {
            return this->x == euler.x && this->y == euler.y && this->z == euler.z;
        }

        bool Euler::operator!=(const Euler& euler)
        {
            return this->x != euler.x || this->y != euler.y || this->z != euler.z;
        }

        double& Euler::operator[](const int& index)
        {
            switch (index)
            {
                case 0:
                    return this->x;
                case 1:
                    return this->y;
                case 2:
                    return this->z;
                default:
                    throw std::out_of_range("Euler index out of range");
            }
        }

        geometry_msgs::Vector3 Euler::toVector3()
        {
            geometry_msgs::Vector3 vector3;
            vector3.x = this->x;
            vector3.y = this->y;
            vector3.z = this->z;
            return vector3;
        }

        std::array<double, AXES> Euler::toArray()
        {
            std::array<double, AXES> array;
            array[0] = this->x;
            array[1] = this->y;
            array[2] = this->z;
            return array;
        }

        void Euler::normalize()
        {
            double length = sqrt(pow(this->x, 2) + pow(this->y, 2) + pow(this->z, 2));
            this->x /= length;
            this->y /= length;
            this->z /= length;
        }
    }
}
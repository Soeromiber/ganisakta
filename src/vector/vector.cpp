#include "ganisakta/vector/vector.hpp"

namespace ganisakta
{
    namespace vector
    {
        geometry_msgs::Vector3 Vector::toVector3(const Vector& vector)
        {
            geometry_msgs::Vector3 vector3;
            vector3.x = vector.x;
            vector3.y = vector.y;
            vector3.z = vector.z;
            return vector3;
        }

        std::array<double, AXES> Vector::toArray(const Vector& vector)
        {
            std::array<double, AXES> array;
            array[0] = vector.x;
            array[1] = vector.y;
            array[2] = vector.z;
            return array;
        }

        Vector Vector::fromVector3(const geometry_msgs::Vector3& vector3)
        {
            return Vector(vector3.x, vector3.y, vector3.z);
        }

        Vector Vector::fromArray(const std::array<double, AXES>& array)
        {
            return Vector(array[0], array[1], array[2]);
        }

        double Vector::dot(const Vector& vector1, const Vector& vector2)
        {
            return vector1.x * vector2.x + vector1.y * vector2.y + vector1.z * vector2.z;
        }

        Vector Vector::cross(const Vector& vector1, const Vector& vector2)
        {
            return Vector(vector1.y * vector2.z - vector1.z * vector2.y,
                          vector1.z * vector2.x - vector1.x * vector2.z,
                          vector1.x * vector2.y - vector1.y * vector2.x);
        }

        Vector Vector::normalize(const Vector& vector)
        {
            double magnitude = Vector::magnitude(vector);
            return Vector(vector.x / magnitude, vector.y / magnitude, vector.z / magnitude);
        }

        double Vector::magnitude(const Vector& vector)
        {
            return sqrt(vector.x * vector.x + vector.y * vector.y + vector.z * vector.z);
        }

        Vector Vector::resultant(const Vector& vector1, const Vector& vector2)
        {
            return Vector(vector1.x + vector2.x, vector1.y + vector2.y, vector1.z + vector2.z);
        }

        Vector Vector::operator+(const Vector& vector)
        {
            return Vector(x + vector.x, y + vector.y, z + vector.z);
        }

        Vector Vector::operator-(const Vector& vector)
        {
            return Vector(x - vector.x, y - vector.y, z - vector.z);
        }

        Vector Vector::operator*(const double& scalar)
        {
            return Vector(x * scalar, y * scalar, z * scalar);
        }

        Vector Vector::operator/(const double& scalar)
        {
            return Vector(x / scalar, y / scalar, z / scalar);
        }

        Vector Vector::operator+=(const Vector& vector)
        {
            x += vector.x;
            y += vector.y;
            z += vector.z;
            return *this;
        }

        Vector Vector::operator-=(const Vector& vector)
        {
            x -= vector.x;
            y -= vector.y;
            z -= vector.z;
            return *this;
        }

        Vector Vector::operator*=(const double& scalar)
        {
            x *= scalar;
            y *= scalar;
            z *= scalar;
            return *this;
        }

        Vector Vector::operator/=(const double& scalar)
        {
            x /= scalar;
            y /= scalar;
            z /= scalar;
            return *this;
        }

        bool Vector::operator==(const Vector& vector)
        {
            return (fabs(x - vector.x) < EPSILON) && (fabs(y - vector.y) < EPSILON) && (fabs(z - vector.z) < EPSILON);
        }

        bool Vector::operator!=(const Vector& vector)
        {
            return !(*this == vector);
        }

        // std::ostream& operator<<(std::ostream& os, const Vector& vector)
        // {
        //     os << "(" << vector.x << ", " << vector.y << ", " << vector.z << ")";
        //     return os;
        // }

        // std::istream& operator>>(std::istream& is, Vector& vector)
        // {
        //     is >> vector.x >> vector.y >> vector.z;
        //     return is;
        // }
        
        geometry_msgs::Vector3 Vector::toVector3()
        {
            return Vector::toVector3(*this);
        }

        std::array<double, AXES> Vector::toArray()
        {
            return Vector::toArray(*this);
        }

        double Vector::dot(const Vector& vector)
        {
            return Vector::dot(*this, vector);
        }

        Vector Vector::cross(const Vector& vector)
        {
            return Vector::cross(*this, vector);
        }

        Vector Vector::normalize()
        {
            return Vector::normalize(*this);
        }

        double Vector::magnitude()
        {
            return Vector::magnitude(*this);
        }

        Vector Vector::resultant(const Vector& vector)
        {
            return Vector::resultant(*this, vector);
        }
    }
}
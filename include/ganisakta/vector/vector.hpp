#pragma once

#include <stdexcept>
#include <geometry_msgs/Vector3.h>

namespace ganisakta
{
    namespace vector
    {
        #define AXES 3
        #define EPSILON 0.00001

        struct Vector
        {
            double x, y, z;

            static geometry_msgs::Vector3 toVector3(const Vector& vector);
            static std::array<double, AXES> toArray(const Vector& vector);
            static Vector fromVector3(const geometry_msgs::Vector3& vector3);
            static Vector fromArray(const std::array<double, AXES>& array);
            static double dot(const Vector& vector1, const Vector& vector2);
            static Vector cross(const Vector& vector1, const Vector& vector2);
            static Vector normalize(const Vector& vector);
            static double magnitude(const Vector& vector);
            static Vector resultant(const Vector& vector1, const Vector& vector2);

            Vector(double x, double y, double z) : x(x), y(y), z(z) {}
            Vector(const geometry_msgs::Vector3& vector) : x(vector.x), y(vector.y), z(vector.z) {}
            Vector(const std::array<double, AXES>& array) : x(array[0]), y(array[1]), z(array[2]) {}

            Vector operator+(const Vector& vector);
            Vector operator-(const Vector& vector);
            Vector operator*(const double& scalar);
            Vector operator/(const double& scalar);
            Vector operator+=(const Vector& vector);
            Vector operator-=(const Vector& vector);
            Vector operator*=(const double& scalar);
            Vector operator/=(const double& scalar);
            bool operator==(const Vector& vector);
            bool operator!=(const Vector& vector);
            double& operator[](const int& index);

            geometry_msgs::Vector3 toVector3();
            std::array<double, AXES> toArray();
            double dot(const Vector& vector);
            Vector cross(const Vector& vector);
            Vector normalize();
            double magnitude();
            Vector resultant(const Vector& vector);
        };
    }
}
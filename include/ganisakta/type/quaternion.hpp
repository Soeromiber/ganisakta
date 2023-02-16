#pragma once

#include <stdexcept>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include "ganisakta/type/eulerangles.hpp"

namespace ganisakta
{
    namespace type
    {
        #define AXES 3

        struct Quaternion
        {
            static EulerAngles toEulerAngles(const Quaternion& quaternion);

            Quaternion();
            Quaternion(double w, double x, double y, double z);
            Quaternion(const geometry_msgs::Quaternion& quaternion);
            Quaternion(const std::array<double, AXES>& quaternion);
            Quaternion operator+(const Quaternion& quaternion);
            Quaternion operator-(const Quaternion& quaternion);
            Quaternion operator*(const double& scalar);
            Quaternion operator/(const double& scalar);
            Quaternion operator+=(const Quaternion& quaternion);
            Quaternion operator-=(const Quaternion& quaternion);
            Quaternion operator*=(const double& scalar);
            Quaternion operator/=(const double& scalar);
            bool operator==(const Quaternion& quaternion);
            bool operator!=(const Quaternion& quaternion);
            double& operator[](const int& index);
            geometry_msgs::Quaternion toQuaternion();
            std::array<double, AXES> toArray();
            EulerAngles toEulerAngles();
            double w, x, y, z;
        };
    }
}
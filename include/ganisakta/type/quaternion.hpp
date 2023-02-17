#pragma once

#include <stdexcept>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include "ganisakta/type/eulerangles.hpp"
#include "ganisakta/type/angle.hpp"

namespace ganisakta
{
    namespace type
    {
        #define AXES 3
        struct EulerAngles; // Forward declaration
        struct Quaternion
        {
            static EulerAngles toEulerAngles(const Quaternion& quaternion);

            Quaternion();
            Quaternion(double x, double y, double z, double w);
            Quaternion(const EulerAngles& rpy);
            Quaternion(const geometry_msgs::Quaternion& quaternion);
            Quaternion(const std::array<double, AXES>& quaternion);
            Quaternion operator+(const Quaternion& quaternion);
            Quaternion operator-(const Quaternion& quaternion);
            Quaternion operator*(const double& scalar);
            Quaternion operator/(const double& scalar);
            Quaternion& operator+=(const Quaternion& quaternion);
            Quaternion& operator-=(const Quaternion& quaternion);
            Quaternion& operator*=(const double& scalar);
            Quaternion& operator/=(const double& scalar);
            bool operator==(const Quaternion& quaternion) const;
            bool operator!=(const Quaternion& quaternion);
            double& operator[](const int& index);
            geometry_msgs::Quaternion toQuaternion();
            std::array<double, AXES> toArray();
            EulerAngles toEulerAngles();
            Quaternion conjugate();
            double w, x, y, z;
        };
    }
}
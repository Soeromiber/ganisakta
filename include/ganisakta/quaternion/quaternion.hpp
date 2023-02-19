#pragma once

#include <stdexcept>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include "ganisakta/angle/angle.hpp"
#include "ganisakta/angle/angle.utils.hpp"

namespace ganisakta
{
    namespace angle
    {
        struct Angle; // Forward declaration
    }
    namespace quaternion
    {
        #define AXES 3
        struct Quaternion
        {
            double w, x, y, z;
            static angle::Angle toEulerAngles(const Quaternion& quaternion);
            geometry_msgs::Quaternion toQuaternion(const Quaternion& quaternion);
            std::array<double, AXES> toArray(const Quaternion& quaternion);

            Quaternion(): x(0), y(0), z(0), w(1) {};
            Quaternion(double x, double y, double z, double w): x(x), y(y), z(z), w(w) {};
            Quaternion(const angle::Angle& rpy);
            Quaternion(const geometry_msgs::Quaternion& quaternion): x(quaternion.x), y(quaternion.y), z(quaternion.z), w(quaternion.w) {};
            Quaternion(const std::array<double, AXES>& rpy);

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
            angle::Angle toEulerAngles();
            Quaternion conjugate();
        };
    }
}
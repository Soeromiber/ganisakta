#pragma once

#include <stdexcept>
#include <geometry_msgs/Vector3.h>
#include "ganisakta/type/quaternion.hpp"
#include "ganisakta/type/angle.hpp"

namespace ganisakta
{
    namespace type
    {
        #define AXES 3
        struct Quaternion; // Forward declaration
        struct EulerAngles
        {
            static EulerAngles toEulerAnglesDegree(const EulerAngles& ypr);
            static EulerAngles fromQuaternion(const Quaternion& quaternion);

            EulerAngles();
            EulerAngles(double roll, double pitch, double yaw);
            EulerAngles(const geometry_msgs::Vector3& ypr);
            EulerAngles(const std::array<double, AXES>& rpy);
            EulerAngles operator+(const EulerAngles& ypr);
            EulerAngles operator-(const EulerAngles& ypr);
            EulerAngles operator*(const double& scalar);
            EulerAngles operator/(const double& scalar);
            EulerAngles operator+=(const EulerAngles& ypr);
            EulerAngles operator-=(const EulerAngles& ypr);
            EulerAngles operator*=(const double& scalar);
            EulerAngles operator/=(const double& scalar);
            bool operator==(const EulerAngles& ypr) const;
            bool operator!=(const EulerAngles& ypr);
            double& operator[](const int& index);
            geometry_msgs::Vector3 toVector3();
            std::array<double, AXES> toArray();
            Quaternion toQuaternion();
            EulerAngles toEulerAnglesDegree();
            double roll;  // rotation around x-axis
            double pitch; // rotation around y-axis
            double yaw;   // rotation around z-axis
        };
    }
}
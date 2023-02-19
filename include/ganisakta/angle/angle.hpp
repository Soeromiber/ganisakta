#pragma once

#include <stdexcept>
#include <geometry_msgs/Vector3.h>
#include "ganisakta/quaternion/quaternion.hpp"

namespace ganisakta
{
    namespace quaternion
    {
        struct Quaternion; // Forward declaration
    }
    namespace angle
    {
        #define AXES 3
        #define EPSILON 0.00001

        // "XYZ" (roll, pitch, yaw), "ZYX" (yaw, pitch, roll), and "ZYZ" (yaw, pitch, yaw)
        struct Angle
        {
            double roll, pitch, yaw;

            static geometry_msgs::Vector3 toVector3(const Angle& angle);
            static std::array<double, AXES> toArray(const Angle& angle);
            static Angle fromVector3(const geometry_msgs::Vector3& vector3);
            static Angle fromArray(const std::array<double, AXES>& array);
            static Angle normalize(const Angle& angle);
            static quaternion::Quaternion toQuaternion(const Angle& angle);
            static Angle fromQuaternion(const quaternion::Quaternion& quaternion);
            static Angle toEulerAnglesDegree(const Angle& angle);

            Angle() : roll(0), pitch(0), yaw(0) {}
            Angle(double roll, double pitch, double yaw) : roll(roll), pitch(pitch), yaw(yaw) {}
            Angle(const geometry_msgs::Vector3& angle) : roll(angle.x), pitch(angle.y), yaw(angle.z) {}
            Angle(const std::array<double, AXES>& angle) : roll(angle[0]), pitch(angle[1]), yaw(angle[2]) {}

            Angle operator+(const Angle& angle);
            Angle operator-(const Angle& angle);
            Angle operator*(const double& scalar);
            Angle operator/(const double& scalar);
            Angle operator+=(const Angle& angle);
            Angle operator-=(const Angle& angle);
            Angle operator*=(const double& scalar);
            Angle operator/=(const double& scalar);
            bool operator==(const Angle& angle);
            bool operator!=(const Angle& angle);
            double& operator[](const int& index);

            geometry_msgs::Vector3 toVector3();
            std::array<double, AXES> toArray();
            quaternion::Quaternion toQuaternion();
            Angle toEulerAnglesDegree();
            Angle normalize();
        };
    }
}
#pragma once

#include <stdexcept>
#include <geometry_msgs/Vector3.h>

namespace ganisakta
{
    namespace type
    {
        #define AXES 3

        struct Euler
        {
            static const Euler ZERO;
            static const Euler ONE;
            static const Euler UP;
            static const Euler DOWN;
            static const Euler LEFT;
            static const Euler RIGHT;
            static const Euler FORWARD;
            static const Euler BACKWARD;
            static double distance(const Euler& euler1, const Euler& euler2);
            static double dot(const Euler& euler1, const Euler& euler2);
            static Euler cross(const Euler& euler1, const Euler& euler2);
            static double angle(const Euler& euler1, const Euler& euler2);
            static Euler normalize(const Euler& euler);
            static geometry_msgs::Vector3 toVector3(const Euler& euler);
            static std::array<double, AXES> toArray(const Euler& euler);

            double x, y, z;

            Euler();
            Euler(double x, double y, double z);
            Euler(const geometry_msgs::Vector3& euler);
            Euler(const std::array<double, AXES>& euler);
            Euler zero();
            Euler operator+(const Euler& euler);
            Euler operator-(const Euler& euler);
            Euler operator*(const double& scalar);
            Euler operator/(const double& scalar);
            Euler operator+=(const Euler& euler);
            Euler operator-=(const Euler& euler);
            Euler operator*=(const double& scalar);
            Euler operator/=(const double& scalar);
            bool operator==(const Euler& euler);
            bool operator!=(const Euler& euler);
            double& operator[](const int& index);
            geometry_msgs::Vector3 toVector3();
            std::array<double, AXES> toArray();
            void normalize();
        };
    }
}
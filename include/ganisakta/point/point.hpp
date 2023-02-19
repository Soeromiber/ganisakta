#pragma once

#include <stdexcept>
#include <geometry_msgs/Vector3.h>

namespace ganisakta
{
    namespace point
    {
        #define AXES 3
        #define EPSILON 0.00001

        struct Point
        {
            double x, y, z;

            static const Point ZERO;
            static const Point ONE;
            static const Point UP;
            static const Point DOWN;
            static const Point LEFT;
            static const Point RIGHT;
            static const Point FORWARD;
            static const Point BACKWARD;

            static double distance(const Point& point1, const Point& point2);
            static Point normalize(const Point& point);

            static geometry_msgs::Vector3 toVector3(const Point& point);
            static std::array<double, AXES> toArray(const Point& point);
            static Point fromVector3(const geometry_msgs::Vector3& vector3);
            static Point fromArray(const std::array<double, AXES>& array);

            Point(double x, double y, double z) : x(x), y(y), z(z) {}
            Point(const geometry_msgs::Vector3& point) : x(point.x), y(point.y), z(point.z) {}
            Point(const std::array<double, AXES>& point) : x(point[0]), y(point[1]), z(point[2]) {}

            Point operator+(const Point& euler);
            Point operator-(const Point& euler);
            Point operator*(const double& scalar);
            Point operator/(const double& scalar);
            Point operator+=(const Point& euler);
            Point operator-=(const Point& euler);
            Point operator*=(const double& scalar);
            Point operator/=(const double& scalar);
            bool operator==(const Point& euler);
            bool operator!=(const Point& euler);
            double& operator[](const int& index);

            geometry_msgs::Vector3 toVector3();
            std::array<double, AXES> toArray();
            
            Point normalize();
            double distance(const Point& point);
        };
    }
}
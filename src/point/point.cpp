#include "ganisakta/point/point.hpp"

namespace ganisakta
{
    namespace point
    {
        const Point Point::ZERO = Point(0, 0, 0);
        const Point Point::ONE = Point(1, 1, 1);
        const Point Point::UP = Point(0, 0, 1);
        const Point Point::DOWN = Point(0, 0, -1);
        const Point Point::LEFT = Point(-1, 0, 0);
        const Point Point::RIGHT = Point(1, 0, 0);
        const Point Point::FORWARD = Point(0, 1, 0);
        const Point Point::BACKWARD = Point(0, -1, 0);

        double Point::distance(const Point& point1, const Point& point2)
        {
            return sqrt(pow(point1.x - point2.x, 2) + pow(point1.y - point2.y, 2) + pow(point1.z - point2.z, 2));
        }

        Point Point::normalize(const Point& point)
        {
            double distance = Point::distance(point, Point::ZERO);
            return Point(point.x / distance, point.y / distance, point.z / distance);
        }

        geometry_msgs::Vector3 Point::toVector3(const Point& point)
        {
            geometry_msgs::Vector3 vector3;
            vector3.x = point.x;
            vector3.y = point.y;
            vector3.z = point.z;
            return vector3;
        }

        std::array<double, AXES> Point::toArray(const Point& point)
        {
            std::array<double, AXES> array;
            array[0] = point.x;
            array[1] = point.y;
            array[2] = point.z;
            return array;
        }

        Point Point::fromVector3(const geometry_msgs::Vector3& vector3)
        {
            return Point(vector3.x, vector3.y, vector3.z);
        }

        Point Point::fromArray(const std::array<double, AXES>& array)
        {
            return Point(array[0], array[1], array[2]);
        }

        Point Point::operator+(const Point& point)
        {
            return Point(this->x + point.x, this->y + point.y, this->z + point.z);
        }

        Point Point::operator-(const Point& point)
        {
            return Point(this->x - point.x, this->y - point.y, this->z - point.z);
        }

        Point Point::operator*(const double& scalar)
        {
            return Point(this->x * scalar, this->y * scalar, this->z * scalar);
        }

        Point Point::operator/(const double& scalar)
        {
            return Point(this->x / scalar, this->y / scalar, this->z / scalar);
        }

        Point Point::operator+=(const Point& point)
        {
            this->x += point.x;
            this->y += point.y;
            this->z += point.z;
            return *this;
        }

        Point Point::operator-=(const Point& point)
        {
            this->x -= point.x;
            this->y -= point.y;
            this->z -= point.z;
            return *this;
        }

        Point Point::operator*=(const double& scalar)
        {
            this->x *= scalar;
            this->y *= scalar;
            this->z *= scalar;
            return *this;
        }

        Point Point::operator/=(const double& scalar)
        {
            this->x /= scalar;
            this->y /= scalar;
            this->z /= scalar;
            return *this;
        }

        bool Point::operator==(const Point& point)
        {
            return Point::distance(*this, point) < 0.0001;
        }

        bool Point::operator!=(const Point& point)
        {
            return !(*this == point);
        }

        double& Point::operator[](const int& index)
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
                    throw std::out_of_range("Index out of range");
            }
        }

        geometry_msgs::Vector3 Point::toVector3()
        {
            return Point::toVector3(*this);
        }

        std::array<double, AXES> Point::toArray()
        {
            return Point::toArray(*this);
        }

        Point Point::normalize()
        {
            return Point::normalize(*this);
        }

        double Point::distance(const Point& point)
        {
            return Point::distance(*this, point);
        }
    }
}
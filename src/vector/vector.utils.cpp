#include "ganisakta/vector/vector.utils.hpp"

namespace ganisakta 
{
    namespace vector
    {
        Rotation::Rotation(){};

        Rotation::Rotation(const std::array<std::array<double, AXES>, AXES>& from, const std::array<std::array<double, AXES>, AXES>& to)
        {
            this->from = from;
            this->to = to;
            this->transformationMatrix = getTransformationMatrix(this->from, this->to);
        }

        std::array<double, AXES> Rotation::transform(const std::array<double, AXES>& v, 
                                        const std::array<std::array<double, AXES>, AXES>& T)
        {
            std::array<double, AXES> result;

            for (int i = 0; i < AXES; ++i)
            {
                result[i] = 0;
                for (int j = 0; j < AXES; ++j)
                    result[i] += v[j] * T[j][i];
            }

            return result;
        }

        std::array<std::array<double, AXES>, AXES> Rotation::getTransformationMatrix(const std::array<std::array<double, AXES>, AXES>& original_coordinate_system,
                                                                    const std::array<std::array<double, AXES>, AXES>& target_coordinate_system)
        {
            std::array<std::array<double, AXES>, AXES> T;

            // T = target_coordinate_system * original_coordinate_system^-1 = target_coordinate_system * original_coordinate_system^T
            // This is used to transform a point/vector from the original coordinate system to the target coordinate system
            // And then we can use the transform function to transform a point from the original coordinate system to the target coordinate system
            // The code work by multiplying the target coordinate system with the transpose of the original coordinate system
            // and then we get the transformation matrix
            for (int i = 0; i < AXES; ++i)
            {
                for (int j = 0; j < AXES; ++j)
                {
                    T[i][j] = 0;
                    for (int k = 0; k < AXES; ++k)
                        T[i][j] += target_coordinate_system[i][k] * original_coordinate_system[k][j];
                }
            }

            return T;
        }

        std::array<double, AXES> Rotation::transform(const std::array<double, AXES>& point)
        {
            return this->transform(point, this->transformationMatrix);
        }

        ganisakta::point::Point Rotation::transform(const ganisakta::point::Point& point)
        {
            std::array<double, AXES> point_array = ganisakta::point::Point(point).toArray();
            std::array<double, AXES> transformed_euler_array = this->transform(point_array, this->transformationMatrix);
            return ganisakta::point::Point(transformed_euler_array);
        }

        ganisakta::vector::Vector Rotation::transform(const ganisakta::vector::Vector& vector)
        {
            std::array<double, AXES> vector_array = ganisakta::vector::Vector(vector).toArray();
            std::array<double, AXES> transformed_euler_array = this->transform(vector_array, this->transformationMatrix);
            return ganisakta::vector::Vector(transformed_euler_array);
        }
    }
}
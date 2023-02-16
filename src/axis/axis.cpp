#include "ganisakta/axis/axis.hpp"

namespace ganisakta 
{
    namespace axis
    {
        AXIS::AXIS(){};

        AXIS::AXIS(const std::array<std::array<double, AXES>, AXES>& from, const std::array<std::array<double, AXES>, AXES>& to)
        {
            this->from = from;
            this->to = to;
            this->transformationMatrix = getTransformationMatrix(this->from, this->to);
        }

        std::array<double, AXES> AXIS::transform(const std::array<double, AXES>& v, 
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

        std::array<std::array<double, AXES>, AXES> AXIS::getTransformationMatrix(const std::array<std::array<double, AXES>, AXES>& original_coordinate_system,
                                                                    const std::array<std::array<double, AXES>, AXES>& target_coordinate_system)
        {
            std::array<std::array<double, AXES>, AXES> T;

            // T = target_coordinate_system * original_coordinate_system^-1 = target_coordinate_system * original_coordinate_system^T
            // This is used to transform a vector from the original coordinate system to the target coordinate system
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

        std::array<double, AXES> AXIS::transformPoint(const std::array<double, AXES>& point)
        {
            return this->transform(point, this->transformationMatrix);
        }

        ganisakta::type::Euler AXIS::transformPoint(const ganisakta::type::Euler& euler)
        {
            std::array<double, AXES> euler_array = ganisakta::type::Euler(euler).toArray();
            std::array<double, AXES> transformed_euler_array = this->transform(euler_array, this->transformationMatrix);
            return ganisakta::type::Euler(transformed_euler_array);
        }
    }
}
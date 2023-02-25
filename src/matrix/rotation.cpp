#include "ganisakta/matrix/rotation.hpp"

namespace ganisakta
{
    namespace matrix
    {
        Rotation::Rotation() {
            rotationMatrix = {{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
        }

        Rotation::Rotation(double theta, const std::array<double, AXES>& axis) {
            // double rad = theta * M_PI / 180;
            double c = std::cos(theta);
            double s = std::sin(theta);
            double x = axis[0];
            double y = axis[1];
            double z = axis[2];

            std::array<std::array<double, AXES>, AXES> matrix = {{
                {c + x*x*(1-c), x*y*(1-c)-z*s, x*z*(1-c)+y*s},
                {y*x*(1-c)+z*s, c+y*y*(1-c), y*z*(1-c)-x*s},
                {z*x*(1-c)-y*s, z*y*(1-c)+x*s, c+z*z*(1-c)}
            }};

            rotationMatrix = matrix;
        }

        std::array<double, AXES> Rotation::rotate(const std::array<double, AXES>& v) {
            std::array<double, AXES> result;
            for (int i = 0; i < AXES; i++) {
                result[i] = 0;
                for (int j = 0; j < AXES; j++) {
                    result[i] += rotationMatrix[i][j] * v[j];
                }
            }
            return result;
        }
    }
}
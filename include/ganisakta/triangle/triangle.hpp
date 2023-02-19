#pragma once

#include <stdexcept>
#include <cmath>

namespace ganisakta
{
    namespace type
    {
        /*  A (alpha)
            |\
            | \
            |  \
          b |   \ c
            |    \
            |_____\
    (gamma)C   a   B (beta)
        */
        struct Triangle
        {
            Triangle();
            Triangle(double a, double b, double c, double alpha, double beta, double gamma);
            bool operator==(const Triangle& triangle);
            bool operator!=(const Triangle& triangle);
            double& operator[](const int& index);
            void calculateAll();
            double a, b, c, alpha, beta, gamma;
        };
    }
}
// Copyright 2023 Soero
#pragma once

#include <iostream>
#include <array>
#include <cmath>

namespace ganisakta
{
    namespace matrix 
    {

        #define AXES 3

        class Rotation
        {
            private:
                std::array<std::array<double, AXES>, AXES> rotationMatrix;

            public:
                Rotation();
                Rotation(double theta, const std::array<double, AXES>& axis);

                std::array<double, AXES> rotate(const std::array<double, AXES>& v);
        };
    }
}
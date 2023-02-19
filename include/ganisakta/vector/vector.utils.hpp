// Copyright 2023 Soero
#pragma once

#include <iostream>
#include <array>
#include "ganisakta/vector/vector.hpp"
#include "ganisakta/point/point.hpp"

namespace ganisakta
{
    namespace vector 
    {

        #define AXES 3

        // Gazebo
        // x-axis front positive, y-axis left positive, and z-axis up positive
        #define XF_YL_ZU {{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}}
        // Drone Relative Coordinate System
        // x-axis front positive, y-axis left negative, and z-axis up positive
        #define XF_YR_ZU {{{1, 0, 0}, {0, -1, 0}, {0, 0, 1}}}
        // x-axis front positive, y-axis left positive, and z-axis up negative
        #define XF_YL_ZD {{{1, 0, 0}, {0, 1, 0}, {0, 0, -1}}}
        // x-axis front positive, y-axis left negative, and z-axis up negative
        #define XF_YR_ZD {{{1, 0, 0}, {0, -1, 0}, {0, 0, -1}}}
        // x-axis front negative, y-axis left positive, and z-axis up positive
        #define XB_YL_ZU {{{-1, 0, 0}, {0, 1, 0}, {0, 0, 1}}}
        // Real World Current Soeromiber, Realsense back
        // x-axis front negative, y-axis left negative, and z-axis up positive
        #define XB_YR_ZU {{{-1, 0, 0}, {0, -1, 0}, {0, 0, 1}}}
        // x-axis front negative, y-axis left positive, and z-axis up negative
        #define XB_YL_ZD {{{-1, 0, 0}, {0, 1, 0}, {0, 0, -1}}}
        // x-axis front negative, y-axis left negative, and z-axis up negative
        #define XB_YR_ZD {{{-1, 0, 0}, {0, -1, 0}, {0, 0, -1}}}
        // x-axis right positive, y-axis front positive, and z-axis up positive
        #define YF_ZU_XR {{{0, 1, 0}, {0, 0, 1}, {1, 0, 0}}}
        // x-axis right positive, y-axis front negative, and z-axis up positive
        #define YB_ZU_XR {{{0, -1, 0}, {0, 0, 1}, {1, 0, 0}}}
        // x-axis right positive, y-axis front positive, and z-axis up negative
        #define YF_ZD_XR {{{0, 1, 0}, {0, 0, -1}, {1, 0, 0}}}
        // x-axis right positive, y-axis front negative, and z-axis up negative
        #define YB_ZD_XR {{{0, -1, 0}, {0, 0, -1}, {1, 0, 0}}}
        // x-axis right negative, y-axis front positive, and z-axis up positive
        #define YF_ZU_XL {{{0, 1, 0}, {0, 0, 1}, {-1, 0, 0}}}
        // x-axis right negative, y-axis front negative, and z-axis up positive
        #define YB_ZU_XL {{{0, -1, 0}, {0, 0, 1}, {-1, 0, 0}}}
        // x-axis right negative, y-axis front positive, and z-axis up negative
        #define YF_ZD_XL {{{0, 1, 0}, {0, 0, -1}, {-1, 0, 0}}}
        // x-axis right negative, y-axis front negative, and z-axis up negative
        #define YB_ZD_XL {{{0, -1, 0}, {0, 0, -1}, {-1, 0, 0}}}
        // x-axis right positive, y-axis up positive, and z-axis front positive
        #define ZF_YU_XR {{{0, 0, 1}, {1, 0, 0}, {0, 1, 0}}}
        // x-axis right positive, y-axis up negative, and z-axis front positive
        #define ZF_YD_XR {{{0, 0, 1}, {-1, 0, 0}, {0, 1, 0}}}
        // x-axis right positive, y-axis up positive, and z-axis front negative
        #define ZB_YU_XR {{{0, 0, -1}, {1, 0, 0}, {0, 1, 0}}}
        // x-axis right positive, y-axis up negative, and z-axis front negative
        #define ZB_YD_XR {{{0, 0, -1}, {-1, 0, 0}, {0, 1, 0}}}
        // x-axis right negative, y-axis up positive, and z-axis front positive
        #define ZF_YU_XL {{{0, 0, 1}, {1, 0, 0}, {0, -1, 0}}}
        // x-axis right negative, y-axis up negative, and z-axis front positive
        #define ZF_YD_XL {{{0, 0, 1}, {-1, 0, 0}, {0, -1, 0}}}
        // x-axis right negative, y-axis up positive, and z-axis front negative
        #define ZB_YU_XL {{{0, 0, -1}, {1, 0, 0}, {0, -1, 0}}}
        // x-axis right negative, y-axis up negative, and z-axis front negative
        #define ZB_YD_XL {{{0, 0, -1}, {-1, 0, 0}, {0, -1, 0}}}
        // x-axis front positive, y-axis up positive, and z-axis right positive
        #define YF_XU_ZR {{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}}
        // x-axis front positive, y-axis up negative, and z-axis right positive
        #define YB_XU_ZR {{{1, 0, 0}, {0, -1, 0}, {0, 0, 1}}}
        // x-axis front positive, y-axis up positive, and z-axis right negative
        #define YF_XD_ZR {{{1, 0, 0}, {0, 1, 0}, {0, 0, -1}}}
        // x-axis front positive, y-axis up negative, and z-axis right negative
        #define YB_XD_ZR {{{1, 0, 0}, {0, -1, 0}, {0, 0, -1}}}
        // x-axis front negative, y-axis up positive, and z-axis right positive
        #define YF_XU_ZL {{{-1, 0, 0}, {0, 1, 0}, {0, 0, 1}}}
        // x-axis front negative, y-axis up negative, and z-axis right positive
        #define YB_XU_ZL {{{-1, 0, 0}, {0, -1, 0}, {0, 0, 1}}}
        // x-axis front negative, y-axis up positive, and z-axis right negative
        #define YF_XD_ZL {{{-1, 0, 0}, {0, 1, 0}, {0, 0, -1}}}
        // x-axis front negative, y-axis up negative, and z-axis right negative
        #define YB_XD_ZL {{{-1, 0, 0}, {0, -1, 0}, {0, 0, -1}}}
        // x-axis up positive, y-axis front positive, and z-axis right positive
        #define ZF_XU_YR {{{1, 0, 0}, {0, 0, 1}, {0, 1, 0}}}
        // x-axis up positive, y-axis front negative, and z-axis right positive
        #define ZB_XU_YR {{{1, 0, 0}, {0, 0, -1}, {0, 1, 0}}}
        // x-axis up positive, y-axis front positive, and z-axis right negative
        #define ZF_XD_YR {{{1, 0, 0}, {0, 0, 1}, {0, -1, 0}}}
        // x-axis up positive, y-axis front negative, and z-axis right negative
        #define ZB_XD_YR {{{1, 0, 0}, {0, 0, -1}, {0, -1, 0}}}
        // x-axis up negative, y-axis front positive, and z-axis right positive
        #define ZF_XU_YL {{{-1, 0, 0}, {0, 0, 1}, {0, 1, 0}}}
        // x-axis up negative, y-axis front negative, and z-axis right positive
        #define ZB_XU_YL {{{-1, 0, 0}, {0, 0, -1}, {0, 1, 0}}}
        // x-axis up negative, y-axis front positive, and z-axis right negative
        #define ZF_XD_YL {{{-1, 0, 0}, {0, 0, 1}, {0, -1, 0}}}
        // x-axis up negative, y-axis front negative, and z-axis right negative
        #define ZB_XD_YL {{{-1, 0, 0}, {0, 0, -1}, {0, -1, 0}}}

        class Rotation
        {
            private:
                std::array<std::array<double, AXES>, AXES> from;
                std::array<std::array<double, AXES>, AXES> to;
                std::array<std::array<double, AXES>, AXES> transformationMatrix;

            public:
                Rotation();
                Rotation(const std::array<std::array<double, AXES>, AXES>& from, const std::array<std::array<double, AXES>, AXES>& to);
                std::array<double, AXES> transform(const std::array<double, AXES>& v, 
                                                const std::array<std::array<double, AXES>, AXES>& T);
                std::array<std::array<double, AXES>, AXES> getTransformationMatrix(const std::array<std::array<double, AXES>, AXES>& original_coordinate_system,
                                                                            const std::array<std::array<double, AXES>, AXES>& target_coordinate_system);
                std::array<double, AXES> transform(const std::array<double, AXES>& point);
                ganisakta::point::Point transform(const ganisakta::point::Point& point);
                ganisakta::vector::Vector transform(const ganisakta::vector::Vector& vector);
        };
    }
}
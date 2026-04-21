#pragma once
#include "point_2d.hpp"

class RoboMath
{
    public:
        RoboMath();

        Point2D find_midpoint(Point2D &point_1, Point2D point_2);

        Point2D find_2d_coords_from_hypotenuse(float &r, float &theta);

        float calculate_vel_by_distance(float meters, std::chrono::seconds time);
};
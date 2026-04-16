#pragma once
#include "point_2d.hpp"

class RobotMath
{
    RobotMath();

    Point2D find_midpoint(float &x2, float &x1, float &y2, float &y1);

    Point2D find_2d_coords_from_hypotenuse(float &r, float &theta);
}
#include "robo_math.hpp"
#include "point_2d.hpp"

RoboMath::RoboMath()
{}

Point2D RoboMath::find_2d_coords_from_hypotenuse(float &r, float &theta)
{
    double x = r * std::cos(theta);
    double y = r * std::sin(theta);
    Point2D point;
    point.x = x;
    point.y = y;
    return point;
}


Point2D RoboMath::find_midpoint(Point2D &point_1, Point2D point_2)
{
    double midpoint_x = (point_2.x + point_1.x) / 2;
    double midpoint_y = (point_2.y + point_1.y) / 2;
    Point2D point;
    point.x = midpoint_x;
    point.y = midpoint_y;
    return point;
}

float RoboMath::calculate_vel_by_distance(float meters, std::chrono::seconds time)
{
    float velocity = meters / time;
    return velocity;
}
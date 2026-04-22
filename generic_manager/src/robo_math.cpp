#include "robo_math.hpp"
#include "point_2d.hpp"

RoboMath::RoboMath()
{}

Point2D RoboMath::find_2d_coords_from_hypotenuse(float &r, float &theta)
{
    double x = r * std::cos(theta);
    double y = r * std::sin(theta);
    Point2D point;
    point.x_ = x;
    point.y_ = y;
    return point;
}


Point2D RoboMath::find_midpoint(Point2D &point_1, Point2D point_2)
{
    double midpoint_x = (point_2.x_ + point_1.x_) / 2;
    double midpoint_y = (point_2.y_ + point_1.y_) / 2;
    Point2D point;
    point.x_ = midpoint_x;
    point.y_ = midpoint_y;
    return point;
}

float RoboMath::calculate_vel_by_distance(float meters, float seconds)
{
    float velocity = meters / seconds;
    return velocity;
}
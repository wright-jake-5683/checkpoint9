#include "odom_manager.hpp"

OdomManager::OdomManager() 
{}

RPY OdomManager::get_rpy(const nav_msgs::msg::Odometry::SharedPtr msg)
{
        RPY rpy;
        // --- Convert orientations ---
        tf2::Quaternion q(
                msg->pose.pose.orientation.x,
                msg->pose.pose.orientation.y,
                msg->pose.pose.orientation.z,
                msg->pose.pose.orientation.w);
        
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        rpy.roll = roll;
        rpy.pitch = pitch;
        rpy.yaw = yaw;

        return rpy;
}

float OdomManager::convert_degrees_to_radians(float degrees)
{
    return degrees * (M_PI / 180.0);
}

float OdomManager::convert_radians_to_degrees(float radians)
{
    return radians * (180.0 / M_PI);
}

float OdomManager::normalize_angle(float angle)
{
    return std::atan2(std::sin(angle), std::cos(angle));
}


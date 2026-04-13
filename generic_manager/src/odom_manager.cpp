#include "odom_manager.hpp"

OdomManager::OdomManager() 
{}

RPY OdomManager::get_rpl(const nav_msgs::msg::Odometry::SharedPtr msg)
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



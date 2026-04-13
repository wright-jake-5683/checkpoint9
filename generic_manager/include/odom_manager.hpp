#pragma once

#include <nav_msgs/msg/odometry.hpp> 
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "rpy.hpp"

class OdomManager
{
public:
    OdomManager();

    RPY get_rpl(const nav_msgs::msg::Odometry::SharedPtr msg);
};
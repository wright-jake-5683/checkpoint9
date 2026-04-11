#pragma once

#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

class DiffDriveManager
{
public:
    DiffDriveManager(
        rclcpp_lifecycle::LifecycleNode::SharedPtr node,
        rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr pub
    );

    void publish_cmd_vel(float linear_x, float angular_z);

private:
    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
};
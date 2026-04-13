#pragma once

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

class DiffDriveManager
{
public:
    DiffDriveManager(rclcpp_lifecycle::LifecycleNode::SharedPtr node, std::string topic);

    void publish_cmd_vel(float linear_x, float angular_z);

    void change_publisher_state(const int &state);
private:
    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    std::string topic_;
};
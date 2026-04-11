#include "diff_drive_manager.hpp"

DiffDriveManager::DiffDriveManager(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node, rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr pub)
    : node_(node), pub_(pub) {}

void DiffDriveManager::publish_cmd_vel(float linear_x, float angular_z)
{
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = linear_x;
    msg.angular.z = angular_z;
    pub_->publish(msg);
    RCLCPP_INFO(node_->get_logger(),
        "Publishing cmd_vel --> x: %.2f, z: %.2f on topic: %s",
        linear_x, angular_z, pub_->get_topic_name());
}
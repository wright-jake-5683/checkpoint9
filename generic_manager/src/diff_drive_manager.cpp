#include "diff_drive_manager.hpp"

DiffDriveManager::DiffDriveManager(rclcpp_lifecycle::LifecycleNode::SharedPtr node, std::string topic)
    : node_(node), topic_(topic) 
    {
        pub_ =  node->create_publisher<geometry_msgs::msg::Twist>(
                topic,
                10);
    }

void DiffDriveManager::publish_cmd_vel(float linear_x, float angular_z)
{
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = linear_x;
    msg.angular.z = angular_z;
    pub_->publish(msg);
}

void DiffDriveManager::change_publisher_state(const int &state)
{
    switch(state) {
        case 1:
            pub_->on_activate();
            break;
        case 2:
            pub_->on_deactivate();
            break;
        case 3:
            pub_.reset();
    }
}
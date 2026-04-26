#include "my_components/pre_approach_component.hpp"

#include <chrono>
#include <iostream>
#include <memory>
#include <utility>

using namespace std::chrono_literals;

namespace my_components
{
    PreApproach::PreApproach(const rclcpp::NodeOptions & options)
    : Node("pre_approach_node", options)
    {   

        // Defer everything until after the node is fully loaded
        init_timer_ = this->create_wall_timer(
            2s, // give simulation time to start publishing
            std::bind(&PreApproach::initialize, this));
    }

    void PreApproach::initialize()
    {
        init_timer_->cancel(); // one-shot

        pub_ = create_publisher<geometry_msgs::msg::Twist>("/diffbot_base_controller/cmd_vel_unstamped", 10);

        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",
            10,
            std::bind(&PreApproach::laser_callback, this, std::placeholders::_1)
        );

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom",
            10,
            std::bind(&PreApproach::odom_callback, this, std::placeholders::_1)
        );

        timer_ = create_wall_timer(50ms, std::bind(&PreApproach::to_go_position, this));

        laser_helper_ = std::make_shared<LaserManager>();
        odom_helper_ = std::make_shared<OdomManager>();
    }

    void PreApproach::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if (!subscription_enabled_) return;
        front_laser_reading_ = laser_helper_->read_front_laser(msg);
        RCLCPP_INFO(this->get_logger(), "Laser Reading: %.2f", front_laser_reading_);
    }

    void PreApproach::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        rpy_ = odom_helper_->get_rpy(msg);
    }

    void PreApproach::to_go_position()
    {
        auto move_msg = geometry_msgs::msg::Twist();
        auto stop_msg = geometry_msgs::msg::Twist(); // zero-initialized

        switch (state_)
        {
            case State::MOVING_FORWARD:
                if (front_laser_reading_ > 0.3 && !destination_reached_) {
                    move_msg.linear.x = 0.5;
                    pub_->publish(move_msg);
                } else {
                    pub_->publish(stop_msg);
                    destination_reached_ = true;
                    target_angle_ = odom_helper_->normalize_angle(
                        odom_helper_->convert_degrees_to_radians(-90.0));
                    state_ = State::ROTATING;
                }
                break;

            case State::ROTATING:
                if (rpy_.yaw > target_angle_) {
                    move_msg.angular.z = -0.2;
                    pub_->publish(move_msg);
                } else {
                    pub_->publish(stop_msg);
                    state_ = State::DONE;
                    RCLCPP_INFO(this->get_logger(), "Task Complete");
                    timer_->cancel();
                    subscription_enabled_ = false;
                }
                break;

            case State::DONE:
                break;
        }
    }   
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(my_components::PreApproach)
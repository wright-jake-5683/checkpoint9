#ifndef COMPOSITION__MOVEROBOT_COMPONENT_HPP_
#define COMPOSITION__MOVEROBOT_COMPONENT_HPP_

#include "my_components/visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>
#include <memory>
#include "rclcpp/callback_group.hpp"
#include "odom_manager.hpp"
#include "diff_drive_manager.hpp"
#include "laser_manager.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <nav_msgs/msg/odometry.hpp> 
#include "rpy.hpp"

namespace my_components
{
    enum class State { MOVING_FORWARD, ROTATING, DONE };
    State state_ = State::MOVING_FORWARD;
    float target_angle_;

    class PreApproach : public rclcpp::Node
    {
        public:
        COMPOSITION_PUBLIC
        explicit PreApproach(const rclcpp::NodeOptions &options);

        protected:
        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
        void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
        void to_go_position();
        void initialize();

        private:
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::TimerBase::SharedPtr init_timer_;
        std::shared_ptr<DiffDriveManager> diff_drive_helper_;
        std::shared_ptr<LaserManager> laser_helper_;
        std::shared_ptr<OdomManager> odom_helper_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::CallbackGroup::SharedPtr callback_group_laser_;
        rclcpp::CallbackGroup::SharedPtr callback_group_odom_;
        rclcpp::CallbackGroup::SharedPtr callback_group_timer_;
        bool destination_reached_ = false;
        float front_laser_reading_;
        RPY rpy_;
        bool subscription_enabled_ = true;
    };
}  // namespace composition

#endif  // COMPOSITION__MOVEROBOT_COMPONENT_HPP_
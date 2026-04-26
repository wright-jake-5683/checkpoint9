#ifndef COMPOSITION__SERVER_COMPONENT_HPP_
#define COMPOSITION__SERVER_COMPONENT_HPP_

#include "geometry_msgs/msg/twist.hpp"
#include "my_components/visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "my_components/srv/go_to_loading.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cstddef>
#include <chrono>
#include <cstdint>
#include <vector>
#include "point_2d.hpp"
#include "laser_manager.hpp"
#include "laser_readings.hpp"
#include "leg_data.hpp"
#include "rcl/client.h"
#include "robo_math.hpp"
#include "tf_manager.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <std_msgs/msg/string.hpp>

#include <memory>

using Loading = my_components::srv::GoToLoading;

namespace my_components {

    class AttachServer : public rclcpp::Node 
    {
        public:
        COMPOSITION_PUBLIC
        explicit AttachServer(const rclcpp::NodeOptions &options);
        void init();

        protected:

        private:
        void service_callback(const std::shared_ptr<Loading::Request> request, std::shared_ptr<Loading::Response> response);
        std::vector<LegData> detect_shelf_legs(sensor_msgs::msg::LaserScan laser_data);
        void create_cart_frame(std::vector<LegData> legs);
        void move_to_cart();
        void center_under_cart();

        std::string service_name_;
        rclcpp::Service<Loading>::SharedPtr service_;
        LaserManager laser_helper_;
        RoboMath robo_math_helper_;
        std::shared_ptr<TfManager> tf_manager_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr shelf_publisher_;
        bool cart_approach_complete_ = false;
    };

} // namespace my_components

#endif // COMPOSITION__SERVER_COMPONENT_HPP_
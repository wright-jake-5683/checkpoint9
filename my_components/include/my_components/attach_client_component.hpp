#ifndef COMPOSITION__CLIENT_COMPONENT_HPP_
#define COMPOSITION__CLIENT_COMPONENT_HPP_

#include "my_components/visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "ros2_service_manager.hpp"
#include "my_components/srv/go_to_loading.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using Loading = my_components::srv::GoToLoading;

namespace my_components {

class AttachClient : public rclcpp::Node {
	public:
	    COMPOSITION_PUBLIC
	    explicit AttachClient(const rclcpp::NodeOptions &options);
	
	private:
        void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
        void final_approach();
        void handle_response(rclcpp::Client<Loading>::SharedFuture future);

        rclcpp::Client<Loading>::SharedPtr client_;
	    std::shared_ptr<ServiceManager> service_manager_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
        sensor_msgs::msg::LaserScan laser_data_;
        std::shared_ptr<rclcpp::TimerBase> timer_;
        
};

} // namespace composition

#endif // COMPOSITION__CLIENT_COMPONENT_HPP_
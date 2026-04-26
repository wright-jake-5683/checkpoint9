#include "my_components/attach_client_component.hpp"

using namespace std::chrono_literals;
using Loading = my_components::srv::GoToLoading;

namespace my_components {

    AttachClient::AttachClient(const rclcpp::NodeOptions &options)
        : Node("approach_shelf_client_node", options) 
    {
        std::string service_name = "/approach_shelf";
        client_ = this->create_client<Loading>(service_name);
        while (!client_->wait_for_service(1s)) 
        {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the %s service. Exiting...", service_name.c_str());
                break;
            }
            RCLCPP_INFO(this->get_logger(), "%s not available, waiting again...", service_name.c_str());
        }

        RCLCPP_INFO(this->get_logger(), "%s client ready!", service_name.c_str());

        timer_ = this->create_wall_timer(2s, std::bind(&AttachClient::final_approach, this));
    }

    void AttachClient::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        laser_data_ = *msg;
    }

    void AttachClient::final_approach()
    {
        // Cancel the timer so this only fires once
        timer_->cancel();

        auto request = std::make_shared<Loading::Request>();
        request->attach_to_shelf = true;
        request->laser_data = laser_data_;

        // Pass a response callback — no blocking wait, the executor handles delivery
        client_->async_send_request(
            request,
            [this](rclcpp::Client<Loading>::SharedFuture future) {
                handle_response(future);
            }
        );
    }

    void AttachClient::handle_response(rclcpp::Client<Loading>::SharedFuture future)
    {
        auto result = future.get();        // safe here — future is already ready
        bool final_approach_complete = result->complete;

        if (final_approach_complete) {
            RCLCPP_INFO(this->get_logger(), "Final Approach Complete");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Final Approach Failed");
        }

        rclcpp::shutdown();
    }

} // namespace my_components

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(my_components::AttachClient)
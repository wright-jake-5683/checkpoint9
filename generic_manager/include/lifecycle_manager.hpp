#include <chrono>
#include <string>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class MyLifecycleServiceClient
{
public:
    MyLifecycleServiceClient(rclcpp_lifecycle::LifecycleNode::SharedPtr node);

    bool change_state(std::string state);

    std::shared_ptr<lifecycle_msgs::msg::State> get_state();

private:
    std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> client_change_state_;
    std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> client_get_state_;
    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;

    bool send_change_request(std::uint8_t transition, std::chrono::seconds time_out = 3s);

    std::shared_ptr<lifecycle_msgs::msg::State> send_get_state_request(std::chrono::seconds time_out = 3s);


    template<typename FutureT, typename WaitTimeT>
    std::future_status wait_for_result(FutureT &future, WaitTimeT time_to_wait)
    {
        auto end = std::chrono::steady_clock::now() + time_to_wait;
        std::chrono::milliseconds wait_period(100);
        std::future_status status = std::future_status::timeout;

        do {
            auto now = std::chrono::steady_clock::now();
            auto time_left = end - now;

            if (time_left <= std::chrono::seconds(0)) {break;}

            status = future.wait_for((time_left < wait_period) ? time_left : wait_period);
        }
        while (rclcpp::ok() && status != std::future_status::ready);

        return status;
    }
};
#include <chrono>
#include <memory>
#include <string>
#include <thread>

//#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"

#include "rclcpp/rclcpp.hpp"

class MyLifecycleServiceClient
{
public:
  explicit MyLifecycleServiceClient(const std::string &change_state_topic)
  {
    client_change_state_ = this->create_client<lifecycle_msgs::srv::ChangeState>(change_state_topic); 
  }

  void change_state(std:string &state)
  {
    for (auto &x : state)
    {
        x = tolower(x);
    }
    cout << state;

    if (state == "configure")
    {
        send_change_request(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    }
    else if (state == "activate")
    {
        send_change_request(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
    }
    else if (state == "deactivate")
    {
        send_change_request(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
    }
    else if (state == "cleanup")
    {
        send_change_request(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
    }
    else if (state == "shutdown")
    {
        send_change_request(lifecycle_msgs::msg::Transition::TRANSITION_SHUTDOWN);
    }
  }



private:
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> client_change_state_;

  void send_change_request(std::uint8_t transition, std::chrono::seconds time_out = 3s)
  {
        auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
        request->transition.id = transition;

        auto future_result = client_change_state_->async_send_request(request);

        
  }

};
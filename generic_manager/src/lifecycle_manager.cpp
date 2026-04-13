#include "lifecycle_manager.hpp"

    MyLifecycleServiceClient::MyLifecycleServiceClient(rclcpp_lifecycle::LifecycleNode::SharedPtr node)
    : node_(node)
    {
        client_change_state_ = node_->create_client<lifecycle_msgs::srv::ChangeState>(std::string(node_->get_name()) + "/change_state");
        client_get_state_ = node_->create_client<lifecycle_msgs::srv::GetState>(std::string(node_->get_name()) + "/get_state");
    }

    std::shared_ptr<lifecycle_msgs::msg::State> MyLifecycleServiceClient::get_state()
    {
        return send_get_state_request();
    }


    std::shared_ptr<lifecycle_msgs::msg::State> MyLifecycleServiceClient::send_get_state_request(std::chrono::seconds time_out)
    {
        if (!client_get_state_->wait_for_service(time_out)) {
            RCUTILS_LOG_INFO_NAMED(node_->get_name(), "Service %s is not available.", client_get_state_->get_service_name());
            return nullptr;
        }

        auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
        auto future_result = client_get_state_->async_send_request(request);
        auto future_status = wait_for_result(future_result, time_out);

        if (future_status != std::future_status::ready) {
            RCUTILS_LOG_ERROR_NAMED(node_->get_name(), "Server time out while setting new lifecycle state on service: %s", client_get_state_->get_service_name());
            return nullptr;
        }

        if (future_status == std::future_status::ready) {
            RCUTILS_LOG_INFO_NAMED(node_->get_name(), "Successfully retrieved lifecycle state on service: %s: ",client_get_state_->get_service_name());
            
            auto state = std::make_shared<lifecycle_msgs::msg::State>();
            state->id = future_result.get()->current_state.id;
            state->label = future_result.get()->current_state.label;
            return state;
        } 
        else 
        {
            RCUTILS_LOG_WARN_NAMED(node_->get_name(), "Failed to retrieve lifecycle state on service: %s: ", client_get_state_->get_service_name());
            return nullptr;
        }
    }

    bool MyLifecycleServiceClient::change_state(std::string &state)
    {
        auto current_state = get_state();
        if (!current_state)
        {
            RCUTILS_LOG_ERROR_NAMED(node_->get_name(), "No current state found when changing state on service: %s", client_change_state_->get_service_name() );
            return false;
        }

        for (auto &x : state)
        {
            x = tolower(x);
        }
        
        RCUTILS_LOG_INFO_NAMED(node_->get_name(), "state: %s", state.c_str());

        if (state == "configure" && current_state->label == "unconfigured")
        {
            return send_change_request(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
        }
        else if (state == "activate" && current_state->label == "inactive")
        {
            return send_change_request(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);   
        }
        else if (state == "deactivate" && current_state->label == "active")
        {
            return send_change_request(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
        }
        else if (state == "cleanup")
        {
            return send_change_request(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
        }
        else if (state == "shutdown" && current_state->label == "unconfigured")
        {
            return send_change_request(lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN);
        }
        else if (state == "shutdown" && current_state->label == "inactive")
        {
            return send_change_request(lifecycle_msgs::msg::Transition::TRANSITION_INACTIVE_SHUTDOWN);
        }
        else if (state == "shutdown" && current_state->label == "active")
        {
            return send_change_request(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVE_SHUTDOWN);
        }
        else 
        {
            RCUTILS_LOG_INFO_NAMED(node_->get_name(), "Invalid transition request");
            return false;
        }
    }

    bool MyLifecycleServiceClient::send_change_request(std::uint8_t transition, std::chrono::seconds time_out)
    {
            if (!client_change_state_->wait_for_service(time_out)) {
                RCUTILS_LOG_INFO_NAMED(node_->get_name(), "Service %s is not available.", client_change_state_->get_service_name());
                return false;
            }

            auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
            request->transition.id = transition;

            auto future_result = client_change_state_->async_send_request(request);
            auto future_status = wait_for_result(future_result, time_out);

            if (future_status != std::future_status::ready) {
                RCUTILS_LOG_ERROR_NAMED(node_->get_name(), "Server time out while setting new lifecycle state on service: %s", client_change_state_->get_service_name());
                return false;
            }

            if (future_result.get()->success) {
                RCUTILS_LOG_INFO_NAMED(node_->get_name(), "Transition %d successfully triggered on service: %s: ", static_cast<int>(transition), client_change_state_->get_service_name());
                return true;
            } 
            else 
            {
                RCUTILS_LOG_WARN_NAMED(node_->get_name(), "Failed to trigger transition %u on service: %s: ", static_cast<unsigned int>(transition), client_change_state_->get_service_name());
                return false;
            }
    }
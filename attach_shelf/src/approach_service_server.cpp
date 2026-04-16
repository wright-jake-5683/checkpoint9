#include "srv/go_to_loading.hpp"
#include "rclcpp/node.hpp"
#include <cstddef>
#include <cstdint>
#include <rclcpp/rclcpp.hpp>

struct LegData
{
    float distance;
    float angle;
};

class ApproachShelfService : public rclcpp::Node {
public:
  ApproachShelfService() : Node("approach_shelf_service_node") {

    service_name_ = "/approach_shelf";
    service_ = this->create_service<attach_shelf::srv::GoToLoading>(
        service_name_,
        std::bind(&ApproachShelfService::service_callback, this,
                  std::placeholders::_1, std::placeholders::_2)
    );

    RCLCPP_INFO(this->get_logger(), "%s is ready...", service_name_);
  }

private:
    std::string service_name_

    void service_callback(const std::shared_ptr<custom_interfaces::srv::GetDirection::Request> request,
        std::shared_ptr<custom_interfaces::srv::GetDirection::Response> response) 
        {
            try 
            {
                RCLCPP_INFO(this->get_logger(), "%s has been requested...", service_name_);

                if (request->attach_to_shelf)
                {
                    LegData[] legs = detect_shelf_legs(request->laser_data);
                    if (legs.empty())
                    {
                        RCLCPP_INFO(this->get_logger(), "Cannot detect shelf legs. Aborting task...");
                        response->completed = false;
                        break;
                    }



                    
                }
                else
                {
                    response->complete = false;
                }

            catch(const std::exception& e)
            {
                RCLCPP_ERROR(this->get_logger(), "Approach Shelf Service Exception: %s", e.what());
            }
        }

    
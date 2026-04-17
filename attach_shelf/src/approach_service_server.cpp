#include "attach_shelf/srv/go_to_loading.hpp"
#include "rclcpp/node.hpp"
#include <cstddef>
#include <cstdint>
#include <rclcpp/rclcpp.hpp>
#include "point_2d.hpp"
#include "laser_manager.hpp"
#include "laser_readings.hpp"
#include "leg_data.hpp"
#include "robo_math.hpp"


class ApproachShelfService : public rclcpp::Node {
public:
  ApproachShelfService() : Node("approach_shelf_service_node") {

    service_name_ = "/approach_shelf";
    service_ = this->create_service<attach_shelf::srv::GoToLoading>(
        service_name_,
        std::bind(&ApproachShelfService::service_callback, this,
                  std::placeholders::_1, std::placeholders::_2)
    );

    laser_helper_ = std::make_shared<LaserManager>();
    robo_math_helper_ = std::make_shared<RoboMath>();


    RCLCPP_INFO(this->get_logger(), "%s is ready...", service_name_.c_str());
  }

private:
    std::string service_name_;
    rclcpp::Service<attach_shelf::srv::GoToLoading>::SharedPtr service_;
    std::shared_ptr<LaserManager> laser_helper_;
    std::shared_ptr<RoboMath> robo_math_helper_;

    void service_callback(const std::shared_ptr<attach_shelf::srv::GoToLoading::Request> request,
        std::shared_ptr<attach_shelf::srv::GoToLoading::Response> response) 
        {
            try 
            {
                RCLCPP_INFO(this->get_logger(), "%s has been requested...", service_name_.c_str());

                if (request->attach_to_shelf)
                {
                    auto legs = detect_shelf_legs(request->laser_data);
                    if (legs.empty())
                    {
                        RCLCPP_INFO(this->get_logger(), "Cannot detect shelf legs. Aborting task...");
                        response->complete = false;
                        return;
                    }
                    
                    for (leg : legs)
                    {
                        RCLCPP_INFO(this->get_logger(), "leg index: %i \n
                                                         leg distance: %.2f
                                                         leg angle: %.2f
                                                         leg point x: %f
                                                         lef point y: %f", 
                                                         leg.index,
                                                         leg.distance,
                                                         leg.angle,
                                                         leg point.x,
                                                         leg.point.y
                        );
                    }

                }
                else
                {
                    response->complete = false;
                }
            }
            catch(const std::exception& e)
            {
                RCLCPP_ERROR(this->get_logger(), "Approach Shelf Service Exception: %s", e.what());
            }
        }
    
    std::vector<LegData> detect_shelf_legs(sensor_msgs::msg::LaserScan laser_data)
    {
        std::vector<std::vector<LaserReadings>> clusters = laser_helper_->cluster_laser_data(laser_data.intensities);
        if (clusters.size() < 2)
        {
            return {};
        }

        // Extract average index from clusters
        std::vector<LegData> legs;
        for (const auto &cluster : clusters)
        {
            long unsigned int sum = 0;
            for (const auto &reading : cluster)
            {
                sum += reading.index;
            }
            size_t middle_index = sum / cluster.size();
            LegData leg{};
            leg.index = middle_index;
            legs.push_back(leg);
        }

        // Extract ranges from leg indidies
        for (const auto &leg : legs )
        {
            leg.distance = laser_data.ranges[leg.index];
        }

        // Get angle of each laser reading
        for (const auto &leg : legs)
        {
            leg.angle = laser_helper_->find_angle_from_laser_reading(laser_data, leg.index);
        }

        for (const auto &leg : leg)
        {
            leg.point = robo_math_helper_->find_2d_coords_from_hypotenuse(leg.distance, leg.angle);
        }

        return legs;
    }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ApproachShelfService>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

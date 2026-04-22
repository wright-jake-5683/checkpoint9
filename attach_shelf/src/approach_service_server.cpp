#include "attach_shelf/srv/go_to_loading.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cstddef>
#include <cstdint>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include "point_2d.hpp"
#include "laser_manager.hpp"
#include "laser_readings.hpp"
#include "leg_data.hpp"
#include "robo_math.hpp"
#include "tf_manager.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::literals::chrono_literals;

class ApproachShelfService : public rclcpp::Node {
public:
  ApproachShelfService() : Node("approach_shelf_service_node") {

    service_name_ = "/approach_shelf";
    service_ = this->create_service<attach_shelf::srv::GoToLoading>(
        service_name_,
        std::bind(&ApproachShelfService::service_callback, this,
                  std::placeholders::_1, std::placeholders::_2)
    );

    timer_ = this->create_wall_timer(50ms, std::bind(&ApproachShelfService::timer_callback, this));

    tf_manager_ = std::make_shared<TfManager>(shared_from_this());

    cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/diffbot_base_controller/cmd_vel_unstamped", 10);

    RCLCPP_INFO(this->get_logger(), "%s is ready...", service_name_.c_str());
  }

private:
    std::string service_name_;
    rclcpp::Service<attach_shelf::srv::GoToLoading>::SharedPtr service_;
    std::shared_ptr<rclcpp::TimerBase> timer_;
    LaserManager laser_helper_;
    RoboMath robo_math_helper_;
    std::shared_ptr<TfManager> tf_manager_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
    bool final_approach_ready_ = false;
    bool cart_approach_complete_ = false;

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
                    
                    create_reference_frame();
                    create_cart_frame(legs);

                    bool cart_frame_exists = tf_manager_->check_if_tf_exists("/reference_frame", "/cart_frame");
                    bool rb1_base_footprint_exists = tf_manager_->check_if_tf_exists("/reference_frame", "/robot_base_footprint");

                    if (cart_frame_exists && rb1_base_footprint_exists)
                    {
                        final_approach_ready_ = true;
                    }

                    if (cart_approach_complete_)
                    {
                        float velocity = robo_math_helper_.calculate_vel_by_distance(0.3, 1);
                        auto msg = geometry_msgs::msg::Twist();
                        msg.linear.x = velocity;
                        cmd_publisher_->publish(msg);
                    }

                    response->complete = true;
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

    void timer_callback()
    {
        if (final_approach_ready_)
        {
            std::shared_ptr<Coordinates> rb1 = tf_manager_->get_tf_coords_parent_to_child("/reference_frame", "/robot_base_footprint");
            std::shared_ptr<Coordinates> cart = tf_manager_->get_tf_coords_parent_to_child("/reference_frame", "/cart_frame");

            double dx = cart->x_ - rb1->x_;
            double dy = cart->y_ - rb1->y_;
            double distance = std::sqrt(dx*dx + dy*dy);

            if (distance != 0.0) 
            {
                tf_manager_->move_subject_towards_target(rb1, cart);
            }
            else 
            {
                timer_->cancel();
                cart_approach_complete_ = true;
            }
        }
    }
    
    std::vector<LegData> detect_shelf_legs(sensor_msgs::msg::LaserScan laser_data)
        {
            std::vector<std::vector<LaserReadings>> clusters = laser_helper_.cluster_laser_data(laser_data.intensities);
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
            for (auto &leg : legs )
            {
            
                leg.distance = laser_data.ranges[leg.index];
            }

            // Get angle of each laser reading
            for (auto &leg : legs)
            {
                leg.angle = laser_helper_.find_angle_from_laser_reading(laser_data, leg.index);
            }

            for (auto &leg : legs)
            {
                leg.point = robo_math_helper_.find_2d_coords_from_hypotenuse(leg.distance, leg.angle);
            }

            return legs;
        }

        void create_reference_frame()
        {   
            Transform new_transform;
            new_transform.translation_x_ = 0;
            new_transform.translation_y_ = 0;
            new_transform.translation_z_ = 0;
            new_transform.parent_frame_ = "/reference_frame";
            new_transform.child_frame_ = "/robot_base_footprint";
            new_transform.roll_ = 0;
            new_transform.pitch_ = 0;
            new_transform.yaw_ = 0;

            tf_manager_->create_static_transform(new_transform);
        }

        void create_cart_frame(std::vector<LegData> legs)
        {
            auto midpoint = robo_math_helper_.find_midpoint(legs[0].point, legs[1].point);
            
            Transform new_transform;
            new_transform.translation_x_ = midpoint.x_;
            new_transform.translation_y_ = midpoint.y_;
            new_transform.translation_z_ = 0;
            new_transform.parent_frame_ = "/robot_front_laser_base_link";
            new_transform.child_frame_ = "/cart_frame";
            new_transform.roll_ = 0;
            new_transform.pitch_ = 0;
            new_transform.yaw_ = 0;

            tf_manager_->create_static_transform(new_transform);
        }    
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ApproachShelfService>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

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
#include "rcl/client.h"
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

        RCLCPP_INFO(this->get_logger(), "%s is ready...", service_name_.c_str());

        callback_group_timer_1_ = this->create_callback_group(
                rclcpp::CallbackGroupType::MutuallyExclusive);

        callback_group_timer_2_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        timer_1_ = this->create_wall_timer(50ms, std::bind(&ApproachShelfService::timer_callback_1, this), callback_group_timer_1_);
        timer_2_ = this->create_wall_timer(50ms, std::bind(&ApproachShelfService::timer_callback_2, this), callback_group_timer_2_);

        cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/diffbot_base_controller/cmd_vel_unstamped", 10);

        
    }

    void init()
    {
        /*  std::enable_shared_from_this (used internally by ROS 2 nodes) only becomes valid after the object is owned by a std::shared_ptr.
            Inside the constructor → ❌ not owned yet
            After make_shared returns → ✅ safe */
        tf_manager_ = std::make_shared<TfManager>(shared_from_this());
    }

private:
    std::string service_name_;
    rclcpp::Service<attach_shelf::srv::GoToLoading>::SharedPtr service_;
    std::shared_ptr<rclcpp::TimerBase> timer_1_;
    std::shared_ptr<rclcpp::TimerBase> timer_2_;
    rclcpp::CallbackGroup::SharedPtr callback_group_timer_1_;
    rclcpp::CallbackGroup::SharedPtr callback_group_timer_2_;
    LaserManager laser_helper_;
    RoboMath robo_math_helper_;
    std::shared_ptr<TfManager> tf_manager_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
    bool final_approach_ready_ = false;
    bool cart_approach_complete_ = false;
    bool elevator_ready_ = false;

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
                    
                    create_cart_frame(legs);
                    /*
                    bool cart_frame_exists = tf_manager_->check_if_tf_exists("reference_frame", "cart_frame");
                    bool rb1_base_footprint_exists = tf_manager_->check_if_tf_exists("reference_frame", "robot_base_footprint");

                    if (cart_frame_exists && rb1_base_footprint_exists)
                    {
                        final_approach_ready_ = true;
                    }
                    */
                    //rclcpp::sleep_for(std::chrono::milliseconds(3000));
                    final_approach_ready_ = true;

                    if (cart_approach_complete_)
                    {
                        float velocity = robo_math_helper_.calculate_vel_by_distance(0.3, 1);
                        auto msg = geometry_msgs::msg::Twist();
                        msg.linear.x = velocity;
                        cmd_publisher_->publish(msg);
                    }

                    while (!elevator_ready_)
                    {}

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

    void timer_callback_1()
    {
        if (final_approach_ready_)
        {
            RCLCPP_INFO(this->get_logger(), "timer 1 entered");
            std::shared_ptr<Coordinates> rb1 = tf_manager_->get_tf_coords_parent_to_child("odom", "robot_base_footprint");
            std::shared_ptr<Coordinates> cart = tf_manager_->get_tf_coords_parent_to_child("odom", "cart_frame");

            if (!rb1 || !cart) {return;}

            double dx = cart->x_ - rb1->x_;
            double dy = cart->y_ - rb1->y_;
            double distance = std::sqrt(dx*dx + dy*dy);
            RCLCPP_INFO(this->get_logger(), "Initial Distance: %.2f", distance);

            if (distance != 0.0) 
            {
                RCLCPP_INFO(this->get_logger(), "Distance: %.2f", distance);
                auto msg = tf_manager_->move_subject_towards_target(rb1, cart);
                cmd_publisher_->publish(msg);
            }
            else 
            {
                timer_1_->cancel();
                auto msg = geometry_msgs::msg::Twist();
                msg.linear.x = 0.0;
                msg.angular.z = 0.0;
                cmd_publisher_->publish(msg);

                cart_approach_complete_ = true;
            }
        }
    }

    void timer_callback_2()
    {
        if (cart_approach_complete_)
        {
            timer_2_->cancel();
            float velocity = robo_math_helper_.calculate_vel_by_distance(0.3, 1);
            auto msg = geometry_msgs::msg::Twist();
            msg.linear.x = velocity;
            cmd_publisher_->publish(msg);

            elevator_ready_ = true;
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

        void create_cart_frame(std::vector<LegData> legs)
        {
            auto midpoint = robo_math_helper_.find_midpoint(legs[0].point, legs[1].point);
            
            Transform new_transform;
            new_transform.translation_x_ = midpoint.x_;
            new_transform.translation_y_ = midpoint.y_;
            new_transform.translation_z_ = 0;
            new_transform.parent_frame_ = "robot_front_laser_base_link";
            new_transform.child_frame_ = "cart_frame";
            new_transform.roll_ = 0;
            new_transform.pitch_ = 0;
            new_transform.yaw_ = 0;

            tf_manager_->create_static_transform(new_transform);
        }    
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ApproachShelfService>();
  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

#include "my_components/attach_server_component.hpp"
#include <exception>

using Loading = my_components::srv::GoToLoading;
using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

namespace my_components 
{
    AttachServer::AttachServer(const rclcpp::NodeOptions &options)
        : Node("approach_shelf_service_node", options) 
    {
         // Defer everything until after the node is fully loaded
        init_timer_ = this->create_wall_timer(
            2s, // give simulation time to start publishing
            std::bind(&AttachServer::initialize, this));
    }

    void AttachServer::initialize()
    {
        init_timer_->cancel();
        service_name_ = "/approach_shelf";
        service_ = this->create_service<Loading>(
            service_name_,
            std::bind(&AttachServer::service_callback, this, _1, _2)
        );

        RCLCPP_INFO(this->get_logger(), "%s is ready...", service_name_.c_str());

        cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/diffbot_base_controller/cmd_vel_unstamped", 10);
        shelf_publisher_ = this->create_publisher<std_msgs::msg::String>("/elevator_up", 10);
        tf_manager_ = std::make_shared<TfManager>(shared_from_this());
    }

    void AttachServer::service_callback(const std::shared_ptr<Loading::Request> request, std::shared_ptr<Loading::Response> response) 
    {
        try 
        {
            RCLCPP_INFO(this->get_logger(), "%s has been requested...", service_name_.c_str());
            if(request->laser_data.ranges.empty())
            {
                RCLCPP_ERROR(this->get_logger(), "Laser data is empty");
            }

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

                while (!cart_approach_complete_)
                {
                    move_to_cart();
                }

                center_under_cart();

                auto msg = std_msgs::msg::String();
                shelf_publisher_->publish(msg);


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

    void AttachServer::move_to_cart()
    {
        std::shared_ptr<Coordinates> rb1 = tf_manager_->get_tf_coords_parent_to_child("odom", "robot_base_footprint");
        std::shared_ptr<Coordinates> cart = tf_manager_->get_tf_coords_parent_to_child("odom", "cart_frame");

        if (!rb1 || !cart) 
        {
            RCLCPP_INFO(this->get_logger(), "Either rb1 or cart frame couldn't be located");
            cart_approach_complete_ = true;
            return;
        }

        double dx = cart->x_ - rb1->x_;
        double dy = cart->y_ - rb1->y_;
        double distance = std::sqrt(dx*dx + dy*dy);
        if (distance > 0.1) 
        {
            auto msg = tf_manager_->move_subject_towards_target(rb1, cart);
            cmd_publisher_->publish(msg);
        }
        else 
        {
            auto msg = geometry_msgs::msg::Twist();
            msg.linear.x = 0.0;
            msg.angular.z = 0.0;
            cmd_publisher_->publish(msg);
            cart_approach_complete_ = true;
        }
    }

    void AttachServer::center_under_cart()
    {
        float velocity = robo_math_helper_.calculate_vel_by_distance(0.3, 5);
        auto start = std::chrono::steady_clock::now();
        auto duration = std::chrono::seconds(5); // run for 1 second

        while (std::chrono::steady_clock::now() - start < duration)
        {
            auto msg = geometry_msgs::msg::Twist();
            msg.linear.x = velocity * 2.5;
            cmd_publisher_->publish(msg);
        }
    }

    std::vector<LegData> AttachServer::detect_shelf_legs(sensor_msgs::msg::LaserScan laser_data)
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

    void AttachServer::create_cart_frame(std::vector<LegData> legs)
    {
        RCLCPP_INFO(this->get_logger(), "Creating Cart Frame");
        try
        {
            auto midpoint = robo_math_helper_.find_midpoint(legs[0].point, legs[1].point);

            // 1. First, transform the midpoint from laser frame → odom frame
            geometry_msgs::msg::PointStamped point_in_laser;
            point_in_laser.header.frame_id = "robot_front_laser_base_link";
            point_in_laser.header.stamp = this->now();
            point_in_laser.point.x = midpoint.x_;
            point_in_laser.point.y = midpoint.y_;
            point_in_laser.point.z = 0.0;

            auto point_in_odom = tf_manager_->transform_point(point_in_laser, "odom");
            if (!point_in_odom) return;


            // 2. Publish static transform with odom as parent
            Transform new_transform;
            new_transform.translation_x_ = point_in_odom->point.x;
            new_transform.translation_y_ = point_in_odom->point.y;
            new_transform.translation_z_ = 0;
            new_transform.parent_frame_ = "odom";  // world-fixed
            new_transform.child_frame_ = "cart_frame";
            new_transform.roll_ = 0;
            new_transform.pitch_ = 0;
            new_transform.yaw_ = 0;

            tf_manager_->create_static_transform(new_transform);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Exception cart frame: %s", e.what());
        }
    } 
} // namespace my_components

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(my_components::AttachServer)
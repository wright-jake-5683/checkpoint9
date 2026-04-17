#include <cmath>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "cpp_helper.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "std_msgs/msg/string.hpp"
#include "odom_manager.hpp"
#include "diff_drive_manager.hpp"
#include "laser_manager.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <nav_msgs/msg/odometry.hpp> 
#include "rpy.hpp"
#include "lifecycle_manager.hpp"
#include "cpp_helper.hpp"
#include "ros2_service_manager.hpp"

using rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;
using namespace std::chrono_literals;

class PreApproachNode : public rclcpp_lifecycle::LifecycleNode {
    public:
        explicit PreApproachNode(char **argv, const std::string &node_name, bool intra_process_comms = false)
        : rclcpp_lifecycle::LifecycleNode(node_name, rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms)), argv_(argv)
        {}

        CallbackReturn on_configure(const rclcpp_lifecycle::State &)
        {
            ////////////////////////////////////////
            //////// Extract Node Arguments ////////
            ///////////////////////////////////////
            argument_parsing();

            ////////////////////////////////////////
            //////// Create Callback Groups ////////
            ///////////////////////////////////////
            callback_group_laser_ = this->create_callback_group(
                rclcpp::CallbackGroupType::MutuallyExclusive);
            
            callback_group_odom_ = this->create_callback_group(
                rclcpp::CallbackGroupType::MutuallyExclusive);

            callback_group_timer_1_ = this->create_callback_group(
                rclcpp::CallbackGroupType::MutuallyExclusive);

            callback_group_timer_2_ = this->create_callback_group(
                rclcpp::CallbackGroupType::MutuallyExclusive);

            rclcpp::SubscriptionOptions laser_options;
            laser_options.callback_group = callback_group_laser_;  
            rclcpp::SubscriptionOptions odom_options;
            odom_options.callback_group = callback_group_odom_;  

            ////////////////////////////////////////
            //////// Create Subscriptions ////////
            ///////////////////////////////////////
            laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                "/scan",
                10,
                std::bind(&PreApproachNode::laser_callback, this, std::placeholders::_1),
                laser_options
            );

            odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                "/odom",
                10,
                std::bind(&PreApproachNode::odom_callback, this, std::placeholders::_1),
                odom_options
            );

            ////////////////////////////////////////
            //////// Declare Helper Classes ////////
            ///////////////////////////////////////
            lifecycle_manager_ = std::make_shared<MyLifecycleServiceClient>(shared_from_this());
            laser_helper_ = std::make_shared<LaserManager>();
            odom_helper_ = std::make_shared<OdomManager>();
            diff_drive_helper_ = std::make_shared<DiffDriveManager>(shared_from_this(), "/diffbot_base_controller/cmd_vel_unstamped");
            cpp_helper_ = std::make_shared<CppHelper>();
            service_manager_ = std::make_shared<ServiceManager>();

            ////////////////////////////////////////
            //////// Create Service Client ////////
            ///////////////////////////////////////
            std::string service_name = "/approach_shelf";
            client_ = this->create_client<attach_shelf::srv::GoToLoading>(serivce_name);
            // Wait for the service to be available (checks every second)
            while (!service_client_->wait_for_service(1s)) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the %s service. Exiting...", service_name.c_str());
                    return;
                }
                    RCLCPP_INFO(this->get_logger(), "%s not available, waiting again...", service_name.c_str());
                }

                RCLCPP_INFO(this->get_logger(), "%s client ready!", service_name.c_str());
            }


            return CallbackReturn::SUCCESS;
        }
        
        CallbackReturn on_activate(const rclcpp_lifecycle::State &)
        {
            diff_drive_helper_->change_publisher_state(1);
            
            timer_1_ = this->create_wall_timer(50ms, std::bind(&PreApproachNode::to_go_position, this), callback_group_timer_1_);
            timer_2_ = this->create_wall_timer(50ms, std::bind(&PreApproachNode::final_approach, this), callback_group_timer_2_);

            return CallbackReturn::SUCCESS;
        }
        
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &)
        {
            diff_drive_helper_->change_publisher_state(2);
            timer_1_.reset();
            timer_2_.reset();
            return CallbackReturn::SUCCESS;
        }
        
        CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)
        {
            diff_drive_helper_->change_publisher_state(3);
            timer_1_.reset();
            timer_2_.reset();
            return CallbackReturn::SUCCESS;
        }
        
        CallbackReturn on_shutdown(const rclcpp_lifecycle::State &)
        {
            diff_drive_helper_->change_publisher_state(3);
            timer_1_.reset();
            timer_2_.reset();
            RCLCPP_INFO(this->get_logger(), "%s shutting down...", this->get_name());
            return CallbackReturn::SUCCESS;
        }
        
        CallbackReturn on_error(const rclcpp_lifecycle::State &)
        {
            return CallbackReturn::FAILURE;
        }
        
    private:
        void argument_parsing()
        {
            obstacle_ = std::stof(argv_[2]);
            degrees_ = std::stof(argv_[4]);
            final_approach_ = cpp_helper_->convert_string_to_bool(argv_[6]);
        }

        void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
        {
            front_laser_reading_ = laser_helper_->read_front_laser(msg);
            laser_data_ = msg;
        }

        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
        {
            rpy_ = odom_helper_->get_rpy(msg);
        }

        void to_go_position()
        {   
            this->timer_1_->cancel();
            while (front_laser_reading_ > obstacle_ && !destination_reached_)
            {
                diff_drive_helper_->publish_cmd_vel(0.5, 0.0);
            }
            diff_drive_helper_->publish_cmd_vel(0.0, 0.0);
            destination_reached_ = true;

            float target_angle = odom_helper_->convert_degrees_to_radians(degrees_);
            target_angle = odom_helper_->normalize_angle(target_angle);
            while (rpy_.yaw > target_angle && destination_reached_)
            {
                diff_drive_helper_->publish_cmd_vel(0.0, -0.2);
            }
            diff_drive_helper_->publish_cmd_vel(0.0, 0.0);

            RCLCPP_INFO(this->get_logger(), "Task Complete");

            in_position_ = true;
        }

        void final_approach()
        {
            if (!final_approach_)
            {
                RCLCPP_WARN(this->get_logger() "Final Approach is set to False, will not perform after getting into position");
                break;
            }
            else if (!in_position_)
            {
                break;
            }
            else 
            {
                this->timer_2_->cancel();

                auto request = std::make_shared<attach_shelf::srv::GoToLoading::Request>();
                request.attach_shelf = final_approach_;
                request.laser_data = laser_data_;
                std::chrono::seconds time_out = 3s;
                auto future_result = client_->send_async_request(request);
                auto future_status = service_manager_->wait_for_result(future_result, time_out)

                if (future_status != std::future_status::ready) {
                    RCLCPP_ERROR(this->get_logger(), "Server time out while final approach service: %s", client_->get_service_name());
                }

                if (future_status == std::future_status::ready) {
                    auto result = future_result.get();
                    bool final_approach_complete = result->complete();
                    if (final_approach_complete)
                    {
                        RCLCPP_INFO(this->get_logger(), "Final Approach Complete");
                    }
                    else
                    {
                        RCLCPP_ERROR(this->get_logger(), "Final Approach Failed");
                    }
                    
                } 
                else 
                {
                    RCLCPP_WARN(this->get_logger(), "Failed to call final approach service: %s: ", client_->get_service_name());
                }
            }

        char **argv_;
        std::shared_ptr<MyLifecycleServiceClient> lifecycle_manager_;
        std::shared_ptr<DiffDriveManager> diff_drive_helper_;
        std::shared_ptr<LaserManager> laser_helper_;
        std::shared_ptr<OdomManager> odom_helper_;
        std::shared_ptr<CppHelper> cpp_helper_;
        std::shared_ptr<ServiceManager> service_manager_;

        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        std::shared_ptr<rclcpp::TimerBase> timer_1_;
        std::shared_ptr<rclcpp::TimerBase> timer_2_;
        rclcpp::Client<attach_shelf::srv::GoToLoading>::SharedPtr client_;

        rclcpp::CallbackGroup::SharedPtr callback_group_laser_;
        rclcpp::CallbackGroup::SharedPtr callback_group_odom_;
        rclcpp::CallbackGroup::SharedPtr callback_group_timer_1_;
        rclcpp::CallbackGroup::SharedPtr callback_group_timer_2_;

        sensor_msgs::msg::LaserScan::SharedPtr laser_data_;
        float obstacle_;
        float degrees_;
        bool final_approach_;
        bool destination_reached_ = false;
        float front_laser_reading_;
        RPY rpy_;
        bool in_position_ = false;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor exe;

    std::shared_ptr<PreApproachNode> node = 
        std::make_shared<PreApproachNode>(argv, "pre_approach_node_v2");

    exe.add_node(node->get_node_base_interface());

    exe.spin();

    rclcpp::shutdown();
    return 0;
}
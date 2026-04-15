#include <cmath>
#include <memory>
#include <rclcpp/rclcpp.hpp>
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

using rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;
using namespace std::chrono_literals;

class PreApproachNode : public rclcpp_lifecycle::LifecycleNode {
    public:
        explicit PreApproachNode(const std::string &node_name, bool intra_process_comms = false)
        : rclcpp_lifecycle::LifecycleNode(node_name, rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
        {}

        CallbackReturn on_configure(const rclcpp_lifecycle::State &)
        {
            callback_group_laser_ = this->create_callback_group(
                rclcpp::CallbackGroupType::MutuallyExclusive);
            
            callback_group_odom_ = this->create_callback_group(
                rclcpp::CallbackGroupType::MutuallyExclusive);

            callback_group_timer_ = this->create_callback_group(
                rclcpp::CallbackGroupType::MutuallyExclusive);

            rclcpp::SubscriptionOptions laser_options;
            laser_options.callback_group = callback_group_laser_;  
            rclcpp::SubscriptionOptions odom_options;
            odom_options.callback_group = callback_group_odom_;  

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

            laser_helper_ = std::make_shared<LaserManager>();
            odom_helper_ = std::make_shared<OdomManager>();
            diff_drive_helper_ = std::make_shared<DiffDriveManager>(shared_from_this(), "/diffbot_base_controller/cmd_vel_unstamped");

            return CallbackReturn::SUCCESS;
        }
        
        CallbackReturn on_activate(const rclcpp_lifecycle::State &)
        {
            diff_drive_helper_->change_publisher_state(1);
            
            timer_ = this->create_wall_timer(50ms, std::bind(&PreApproachNode::to_go_position, this), callback_group_timer_);

            return CallbackReturn::SUCCESS;
        }
        
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &)
        {
            diff_drive_helper_->change_publisher_state(2);
            return CallbackReturn::SUCCESS;
        }
        
        CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)
        {
            diff_drive_helper_->change_publisher_state(3);
            return CallbackReturn::SUCCESS;
        }
        
        CallbackReturn on_shutdown(const rclcpp_lifecycle::State &)
        {
            diff_drive_helper_->change_publisher_state(3);
            return CallbackReturn::SUCCESS;
        }
        
        CallbackReturn on_error(const rclcpp_lifecycle::State &)
        {
            return CallbackReturn::FAILURE;
        }
        
    private:
        std::shared_ptr<DiffDriveManager> diff_drive_helper_;
        std::shared_ptr<LaserManager> laser_helper_;
        std::shared_ptr<OdomManager> odom_helper_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        bool destination_reached_ = false;
        float front_laser_reading_;
        RPY rpy_;
        std::shared_ptr<rclcpp::TimerBase> timer_;
        bool in_position_ = false;
        rclcpp::CallbackGroup::SharedPtr callback_group_laser_;
        rclcpp::CallbackGroup::SharedPtr callback_group_odom_;
        rclcpp::CallbackGroup::SharedPtr callback_group_timer_;

        void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
        {
            front_laser_reading_ = laser_helper_->read_front_laser(msg);
            //RCLCPP_INFO(this->get_logger(), "front laser: %.2f", front_laser_reading_);
        }

        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
        {
            rpy_ = odom_helper_->get_rpy(msg);
        }

        bool to_go_position()
        {   
            while (front_laser_reading_ > .3 && !destination_reached_)
            {
                diff_drive_helper_->publish_cmd_vel(0.5, 0.0);
                //RCLCPP_INFO(this->get_logger(), "2: %.2f", front_laser_reading_);
            }
            diff_drive_helper_->publish_cmd_vel(0.0, 0.0);
            RCLCPP_INFO(this->get_logger(), "Stop 1");
            destination_reached_ = true;

            
            while (rpy_.yaw > -1.56 && destination_reached_)
            {
                diff_drive_helper_->publish_cmd_vel(0.0, -0.2);
                RCLCPP_INFO(this->get_logger(), "Yaw: %.2f", rpy_.yaw);
            }
            diff_drive_helper_->publish_cmd_vel(0.0, 0.0);
            RCLCPP_INFO(this->get_logger(), "Stop 2");

            in_position_ = true;
            
            return true;

        };
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor exe;

    std::shared_ptr<PreApproachNode> node = 
        std::make_shared<PreApproachNode>("pre_approach_node");

    exe.add_node(node->get_node_base_interface());

    exe.spin();

    rclcpp::shutdown();
    return 0;
}
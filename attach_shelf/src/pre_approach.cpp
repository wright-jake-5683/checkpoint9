#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"
#include "odom_manager.hpp"
#include "diff_drive_manager.hpp"
#include "laser_manager.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <nav_msgs/msg/odometry.hpp> 
#include "rpy.hpp"


class PreApproachNode : public rclcpp_lifecycle::LifecycleNode {
    public:
        explicit PreApproachNode(const std::string &node_name, bool intra_process_comms = false)
        : rclcpp_lifecycle::LifecycleNode(node_name, rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
        {}

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State &)
        {
            laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                "/scan",
                10,
                std::bind(&PreApproachNode::laser_callback, this, std::placeholders::_1)
            );

            odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                "/odom",
                10,
                std::bind(&PreApproachNode::odom_callback, this, std::placeholders::_1)
            );

            laser_helper_ = std::make_shared<LaserManager>();
            odom_helper_ = std::make_shared<OdomManager>();
            diff_drive_helper_ = std::make_shared<DiffDriveManager>(shared_from_this(), "/diffbot_base_controller/cmd_vel_unstamped");

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }
        
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State &)
        {
            diff_drive_helper_->change_publisher_state(1);
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }
        
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State &)
        {
            diff_drive_helper_->change_publisher_state(2);
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }
        
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_cleanup(const rclcpp_lifecycle::State &)
        {
            diff_drive_helper_->change_publisher_state(3);
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }
        
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_shutdown(const rclcpp_lifecycle::State &)
        {
            diff_drive_helper_->change_publisher_state(3);
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }
        
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_error(const rclcpp_lifecycle::State &)
        {
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
        }
        
    private:
        std::shared_ptr<DiffDriveManager> diff_drive_helper_;
        std::shared_ptr<LaserManager> laser_helper_;
        std::shared_ptr<OdomManager> odom_helper_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        bool destination_reached_;

        void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
        {
            bool pub_status = diff_drive_helper_->check_publisher_status();
            float front_laser_reading = laser_helper_->read_front_laser(msg);

            if (front_laser_reading < .75 && !destination_reached_ && pub_status)
            {
                diff_drive_helper_->publish_cmd_vel(0.0, 0.0);
                destination_reached_ = true;
            }
            else if (!destination_reached_ && pub_status)
            {
                diff_drive_helper_->publish_cmd_vel(1.0, 0.0);
                destination_reached_ = false;
            }
        }

        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
        {
            bool pub_status = diff_drive_helper_->check_publisher_status();
            RPY rpy = odom_helper_->get_rpl(msg);

            if (rpy.yaw > -1.56 && destination_reached_ && pub_status)
            {
                diff_drive_helper_->publish_cmd_vel(0.0, -0.2);
            }
            else if (rpy.yaw <= -1.56 && destination_reached_ && pub_status)
            {
                diff_drive_helper_->publish_cmd_vel(0.0, 0.0);
            }
            else {}
        }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exe;

    std::shared_ptr<PreApproachNode> node = 
        std::make_shared<PreApproachNode>("pre_approach_node");

    exe.add_node(node->get_node_base_interface());

    exe.spin();

    rclcpp::shutdown();
    return 0;
}
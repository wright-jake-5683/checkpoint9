#include <rclcpp/rclcpp.hpp>
#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <nav_msgs/msg/odometry.hpp> 
#include "diff_drive_manager.hpp"
#include "laser_manager.hpp"


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

            cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
                "/diffbot_base_controller/cmd_vel_unstamped",
                10
            );

            diff_drive_helper_ = std::make_shared<DiffDriveManager>(shared_from_this(), cmd_pub_);
            laser_helper_ = std::make_shared<LaserManager>();
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;


        }
        
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State &)
        {
            cmd_pub_->on_activate();
            
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }
        
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State &)
        {
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }
        
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_cleanup(const rclcpp_lifecycle::State &)
        {
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }
        
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_shutdown(const rclcpp_lifecycle::State &)
        {
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }
        
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_error(const rclcpp_lifecycle::State &)
        {
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
        }
        
    private:
        rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        std::shared_ptr<DiffDriveManager> diff_drive_helper_;
        std::shared_ptr<LaserManager> laser_helper_;

        void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
        {
            float front_laser_reading = laser_helper_->read_front_laser(msg);
            
            RCLCPP_INFO(this->get_logger(), "Front Laser Reading: %.2f", front_laser_reading);

        }

        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
        {
        
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
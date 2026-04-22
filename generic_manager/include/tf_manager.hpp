#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <memory>
#include <limits>
#include <cmath>
#include <string>
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_ros/buffer.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"

struct Coordinates
{
    public:
        double x_, y_, z_;
        double roll_, pitch_, yaw_;

        Coordinates():
                x_(std::numeric_limits<double>::quiet_NaN()), 
                y_(std::numeric_limits<double>::quiet_NaN()),
                z_(std::numeric_limits<double>::quiet_NaN()),
                roll_(std::numeric_limits<double>::quiet_NaN()),
                pitch_(std::numeric_limits<double>::quiet_NaN()),
                yaw_(std::numeric_limits<double>::quiet_NaN())
            {}

            bool isEmpty() const
            {
                return std::isnan(x_) && 
                    std::isnan(y_) &&
                    std::isnan(z_) && 
                    std::isnan(roll_) &&
                    std::isnan(pitch_) && 
                    std::isnan(yaw_);
            }
        Coordinates(double x, double y, double z, double roll, double pitch, double yaw)
            : x_(x), y_(y), z_(z), roll_(roll), pitch_(pitch), yaw_(yaw) {}
};

struct Transform
{
    std::string parent_frame_;
    std::string child_frame_; 

    double translation_x_ ;
    double translation_y_;
    double translation_z_;

    double roll_;
    double pitch_;
    double yaw_;

    Transform() : 
        translation_x_(std::numeric_limits<double>::quiet_NaN()), 
        translation_y_(std::numeric_limits<double>::quiet_NaN()),
        translation_z_(std::numeric_limits<double>::quiet_NaN()),
        roll_(std::numeric_limits<double>::quiet_NaN()),
        pitch_(std::numeric_limits<double>::quiet_NaN()),
        yaw_(std::numeric_limits<double>::quiet_NaN())
    {}

    bool isEmpty() const
    {
        return std::isnan(translation_x_) && 
               std::isnan(translation_y_) &&
               std::isnan(translation_z_) && 
               std::isnan(roll_) &&
               std::isnan(pitch_) && 
               std::isnan(yaw_) &&
               parent_frame_.empty() &&
               child_frame_.empty();
    }
};

class TfManager
{
    public:
        TfManager(rclcpp::Node::SharedPtr node);

        void create_static_transform(const Transform &new_transform);

        std::shared_ptr<Coordinates> get_tf_coords_parent_to_child(const std::string &parent_frame, const std::string &child_frame);

        geometry_msgs::msg::Twist move_subject_towards_target(std::shared_ptr<Coordinates> subject, std::shared_ptr<Coordinates> target);

        bool check_if_tf_exists(const std::string &parent_frame, const std::string &child_frame);

    private:
        tf2_ros::StaticTransformBroadcaster static_broadcaster_;
        rclcpp::Node::SharedPtr node_;
        tf2_ros::Buffer tf_buffer_;
};
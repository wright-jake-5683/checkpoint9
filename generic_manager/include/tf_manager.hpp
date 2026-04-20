#include "rclcpp/clock.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.h"
#include "tf2/LinearMath/Quaternion.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include <memory>

class TfManager
{
    public:
        TfManager(rclcpp::Clock::SharedPtr node_clock, rclcpp::Logger node_logger);

        void create_static_transform(const Transform &new_transform);

        std::shared_ptr<Coordinates> get_tf_coords_parent_to_child(const std::string &parent_frame, const std::string &child_frame)

        geometry_msgs::msg::Twist move_subject_towards_target(std::shared_ptr<Coordinates> subject std::shared_ptr<Coordinates> target)

    private:
        tf2_ros::StaticTransformBroadcaster static_broadcaster_;
        rclcpp::Clock clock_;
        rclcpp::Logger logger_;
};


struct Coordinates
{
    public:
        double x, y, z;
        double roll, pitch, yaw;

        Coordinates():
                x(std::numeric_limits<double>::quiet_NaN()), 
                y(std::numeric_limits<double>::quiet_NaN()),
                z(std::numeric_limits<double>::quiet_NaN()),
                roll(std::numeric_limits<double>::quiet_NaN()),
                pitch(std::numeric_limits<double>::quiet_NaN()),
                yaw(std::numeric_limits<double>::quiet_NaN())
            {}

            bool isEmpty() const
            {
                return std::isnan(x) && 
                    std::isnan(y) &&
                    std::isnan(z) && 
                    std::isnan(roll) &&
                    std::isnan(pitch) && 
                    std::isnan(yaw)
            }
};

struct Transform
{
    std::string parent_frame;
    std::string child_frame; 

    double translation_x ;
    double translation_y;
    double translation_z;

    double roll;
    double pitch;
    double yaw;

    Transform() : 
        translation_x(std::numeric_limits<double>::quiet_NaN()), 
        translation_y(std::numeric_limits<double>::quiet_NaN()),
        translation_z(std::numeric_limits<double>::quiet_NaN()),
        roll(std::numeric_limits<double>::quiet_NaN()),
        pitch(std::numeric_limits<double>::quiet_NaN()),
        yaw(std::numeric_limits<double>::quiet_NaN())
    {}

    bool isEmpty() const
    {
        return std::isnan(translation_x) && 
               std::isnan(translation_y) &&
               std::isnan(translation_z) && 
               std::isnan(roll) &&
               std::isnan(pitch) && 
               std::isnan(yaw) &&
               parent_frame.isEmpty() &&
               child_frame.isEmpty
    }
};
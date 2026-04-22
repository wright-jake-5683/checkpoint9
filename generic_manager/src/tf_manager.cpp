#include "tf_manager.hpp"

TfManager::TfManager(rclcpp::Node::SharedPtr node) : static_broadcaster_(node), node_(node), tf_buffer_(node->get_clock())
{
}

void TfManager::create_static_transform(const Transform &new_transform)
{
    geometry_msgs::msg::TransformStamped static_transformStamped;

    static_transformStamped.header.stamp = node_->get_clock()->now();

    static_transformStamped.header.frame_id = new_transform.parent_frame_;
    static_transformStamped.child_frame_id = new_transform.child_frame_;

    static_transformStamped.transform.translation.x = new_transform.translation_x_;
    static_transformStamped.transform.translation.y = new_transform.translation_y_;
    static_transformStamped.transform.translation.z = new_transform.translation_z_;

    tf2::Quaternion q;
    q.setRPY(new_transform.roll_, new_transform.pitch_, new_transform.yaw_);
    static_transformStamped.transform.rotation.x = q.x();
    static_transformStamped.transform.rotation.y = q.y();
    static_transformStamped.transform.rotation.z = q.z();
    static_transformStamped.transform.rotation.w = q.w();

    try
    {
        static_broadcaster_.sendTransform(static_transformStamped);
        RCLCPP_INFO(node_->get_logger(), "New Transform published from %s to %s", new_transform.parent_frame_.c_str(), new_transform.child_frame_.c_str());
    }
    catch(const std::exception &e)
    {
        RCLCPP_ERROR(node_->get_logger(), "Exception thrown while publishing new transform: %s", e.what());
    }
}

std::shared_ptr<Coordinates> TfManager::get_tf_coords_parent_to_child(const std::string &parent_frame, const std::string &child_frame)
{
     geometry_msgs::msg::TransformStamped transform;

    try
    {
        transform = tf_buffer_.lookupTransform(parent_frame.c_str(), child_frame.c_str(), tf2::TimePointZero, tf2::durationFromSec(1.0));
    }
    catch (const tf2::TransformException &e)
    {
        RCLCPP_ERROR(node_->get_logger(), "Could not find transfrom from %s to %s. \nException: %s", parent_frame.c_str(), child_frame.c_str(), e.what());
        return nullptr;
    }

    auto &translation = transform.transform.translation;
    auto &rotation = transform.transform.rotation;

    auto pose = std::make_shared<geometry_msgs::msg::Pose>();
    pose->position.x = translation.x;
    pose->position.y = translation.y;
    pose->position.z = translation.z;
    pose->orientation.x = rotation.x;
    pose->orientation.y = rotation.y;
    pose->orientation.z = rotation.z;
    pose->orientation.w = rotation.w;

    if (!pose) return nullptr;

    // --- Convert orientations ---
    tf2::Quaternion target_q(
        pose->orientation.x,
        pose->orientation.y,
        pose->orientation.z,
        pose->orientation.w
    );
    
    double roll, pitch, yaw;
    tf2::Matrix3x3(target_q).getRPY(roll, pitch, yaw);


    return std::make_shared<Coordinates>(
        pose->position.x, 
        pose->position.y, 
        pose->position.z,
        roll, 
        pitch, 
        yaw
    );
}

geometry_msgs::msg::Twist TfManager::move_subject_towards_target(std::shared_ptr<Coordinates> subject, std::shared_ptr<Coordinates> target)
{
    // --- Position error ---
    double dx = target->x_ - subject->x_;
    double dy = target->y_ - subject->y_;
    double distance = std::sqrt(dx*dx + dy*dy);


    // --- Angle to target position ---
    double dyaw = std::atan2(dy, dx);


    // --- Heading error, normalized to [-pi, pi] ---
    double yaw_error = dyaw - subject->yaw_;
    while (yaw_error >  M_PI) yaw_error -= 2.0 * M_PI;
    while (yaw_error < -M_PI) yaw_error += 2.0 * M_PI;

    //constexpr tells complier kp_yaw & kp_distance values at complie time instead of at runtime
    constexpr double kp_yaw = 1;
    double kp_distance;
    if (yaw_error > -0.1 && yaw_error < 0.1)
    {
        kp_distance = 0.25;
    }
    else 
    {
        kp_distance = 0.10;
    }

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = kp_distance * distance;
    cmd.angular.z = kp_yaw * yaw_error;

    return cmd;
}

bool TfManager::check_if_tf_exists(const std::string &parent_frame, const std::string &child_frame)
{
    if (tf_buffer_.canTransform(child_frame, parent_frame, tf2::TimePointZero))
    {
        return true;
    }
    else
    {
        return false;
    }
}
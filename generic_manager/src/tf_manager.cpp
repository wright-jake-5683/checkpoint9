#include "tf_manager.hpp"

TfManager::TfManager() : clock_(node_clock), logger_(node_logger)
{}

void TfManager::create_static_transform(const Transform &new_transform)
{
    geometry_msgs::msg::TransformStamped static_transformStamped;

    static_transformStamped.header.stamp = clock_->now();

    static_transformStamped.header.frame_id = new_transform.parent_frame;
    static_transformStamped.header.child_frame_id = new_transform.child_frame;

    static_transformStamped.transform.translation.x = new_transform.translation_x;
    static_transformStamped.transform.translation.y = new_transform.translation_y;
    static_transformStamped.transform.translation.z = new_transform.translation_z;

    tf2::Quaternion q;
    q.setRPY(new_transform.roll, new_transform.pitch, new_transform.yaw)
    static_transformStamped.transform.rotation.x = q.x();
    static_transformStamped.transform.rotation.y = q.y();
    static_transformStamped.transform.rotation.z = q.z();
    static_transformStamped.transform.rotation.w = q.w();

    try
    {
        static_broadcaster_.sendTransform(static_transformStamped);
        RCLCPP_INFO(logger_, "New Transform published from %s to %s", new_transform.parent_frame.c_str(), new_transform.child_frame.c_str());
    }
    catch(const std::Exception e)
    {
        RCLCPP_ERROR(logger_, "Exception thrown while publishing new transform: %s", e.what());
    }
}

std::shared_ptr<Coordinates> TfManager::get_tf_coords_parent_to_child(const std::string &parent_frame, const std::string &child_frame)
{
     geometry_msgs::msg::TransformStamped transform;
     std::shared_ptr<tf2_ros::Buffer> tf_buffer = std::make_shared<tf2_ros::Buffer>(clock_->get_clock());;

    try
    {
        transform = tf_buffer->lookupTransform(parent_frame.c_str(), child_frame.c_str(), tf2::TimePointZero, tf2::durationFromSec(1.0));
    }
    catch (const tf2::TransformException &e)
    {
        RCLCPP_ERROR(logger_, "Could not find transfrom from %s to %s. \nException: %s", parent_frame.c_str(), child_frame.c_str(), e.what());
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
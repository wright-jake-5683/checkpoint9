#ifndef TRANSFORM_HPP_
#define TRANSFORM_HPP_

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

#endif
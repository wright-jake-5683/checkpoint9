#include "laser_manager.hpp"

LaserManager::LaserManager()
{}

float LaserManager::read_front_laser(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    if (msg->ranges.empty()) {
          throw std::runtime_error("No laser data received yet");
    }

    int middle_index = std::round(msg->ranges.size() / 2);
    return msg->ranges[middle_index];
}

float LaserManager::find_angle_from_laser_reading(sensor_msgs::msg::LaserScan msg, int index)
{
    float angle = msg.angle_min + (index * msg.angle_increment);
    return angle;
}

std::vector<std::vector<LaserReadings>> LaserManager::cluster_laser_data(const std::vector<float> &readings)
{
    std::vector<std::vector<LaserReadings>> clusters;
    std::vector<LaserReadings> current_cluster;

    const float INTENSITY_THRESHOLD = 7000.0;

    for (size_t i = 0; i < readings.size(); ++i)
    {
        if (readings[i] > INTENSITY_THRESHOLD)
        {
            if (current_cluster.empty() || i == current_cluster.back().index + 1)
            {
                LaserReadings reading = {i, readings[i]};
                current_cluster.push_back(reading);
            }
            else
            {
                clusters.push_back(current_cluster);
                current_cluster.clear();
                LaserReadings reading = {i, readings[i]};
                current_cluster.push_back(reading);
            }
        }
    }

    if (!current_cluster.empty())
    {
        clusters.push_back(current_cluster);
    }

    return clusters;
}
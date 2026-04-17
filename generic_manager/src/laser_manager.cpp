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

float LaserManager::find_angle_from_laser_reading(sensor_msgs::msg::LaserScan::SharedPtr msg, int index)
{
    float angle = msg->angle_min + (index * msg->angle_increment);
    return angle;
}

std::vector<std::vector<size_t>> LaserManager::cluster_laser_data(float[] &readings)
{
    std::vector<std::vector<size_t>> clusters;
    std::vector<size_t> current_cluster;

    const float INTENSITY_THRESHOLD = 1000.0;

    for (size_t i = 0; i < readings.size(); ++i)
    {
        if (readings[i] > INTENSITY_THRESHOLD)
        {
            if (current_cluster.empty() || i == current_cluster.back() + 1)
            {
                current_cluster.push_back(i);
            }
            else
            {
                clusters.push_back(current_cluster);
                current_cluster.clear();
                current_cluster.push_back(i);
            }
        }
    }

    if (!current_cluster.empty())
    {
        clusters.push_back(current_cluster);
    }

    return clusters;
}
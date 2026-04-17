#ifndef LEG_DATA_HPP_
#define LEG_DATA_HPP_

#include "point_2d.hpp"

struct LegData
{
    int index;
    float distance;
    float angle;
    Point2D point;

    LegData() : index(-1), distance(-1.0f), angle(std::numeric_limits<float>::quiet_NaN()), point{}
    {}

    bool isEmpty() const
    {
        return index < 0 &&
               distance < 0.0f && 
               std::isnan(angle); 
    }
};

#endif
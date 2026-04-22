#ifndef POINT2D_HPP_
#define POINT2D_HPP_

#include <cstddef>
#include <limits>
#include <cmath>

struct Point2D
{
    double x_;
    double y_;

    Point2D() : x_(std::numeric_limits<double>::quiet_NaN()), y_(std::numeric_limits<double>::quiet_NaN())
    {}

    bool isEmpty() const
    {
        return std::isnan(x_) && 
               std::isnan(y_);
    }
};

#endif
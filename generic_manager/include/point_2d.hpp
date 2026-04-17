#ifndef POINT2D_HPP_
#define POINT2D_HPP_

#include <cstddef>
#include <limits>
#include <cmath>

struct Point2D
{
    double x;
    double y;

    Point2D() : x(std::numeric_limits<double>::quiet_NaN()), y(std::numeric_limits<double>::quiet_NaN())
    {}

    bool isEmpty() const
    {
        return std::isnan(x) && 
               std::isnan(y);
    }
};

#endif
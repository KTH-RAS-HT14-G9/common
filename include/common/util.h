#ifndef COMMON_UTIL_H
#define COMMON_UTIL_H

#include <ros/ros.h>

namespace common {

    template<typename T>
    inline T Clamp(T value, T min, T max)
    {
        if (value < min) return min;
        if (value > max) return max;
        return value;
    }

}

#endif // COMMON_UTIL_H

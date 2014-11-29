#ifndef COMMON_DEBUG_H
#define COMMON_DEBUG_H

//------------------------------------------------------------------------------
// Vision
#define ENABLE_VISUALIZATION_PLANES 0
#define ENABLE_VISUALIZATION_ROIS 0
#define ENABLE_VISUALIZATION_RECOGNITION 0

//------------------------------------------------------------------------------
// Time profiling
#define ENABLE_TIME_PROFILING 0

#define MEASURE_TIME(...) \
[&] { \
    ros::Time start = ros::Time::now(); \
    __VA_ARGS__; \
    return ros::Time::now().toSec()-start.toSec(); \
} ();

#endif // COMMON_DEBUG_H

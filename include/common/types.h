#ifndef COMMON_TYPES_H
#define COMMON_TYPES_H

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace common {

    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
    typedef PointCloud::ConstPtr SharedPointCloud;

}

#endif // TYPES_H

#ifndef COMMON_TYPES_H
#define COMMON_TYPES_H

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace common {

    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
    typedef PointCloud::ConstPtr SharedPointCloud;

    typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;
    typedef PointCloudRGB::ConstPtr SharedPointCloudRGB;


    class Color {
    public:
        Color(int cr, int cg, int cb)
            :r(cr),g(cg),b(cb) {}
        int r, g, b;
    };

}

#endif // TYPES_H

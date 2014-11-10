#ifndef COMMON_ROI_H
#define COMMON_ROI_H

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <opencv2/core/core.hpp>

namespace common {

    namespace vision {

        class ROI {
        public:
            ROI(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
                :pcloud_of_interest(cloud)
            {
            }

            pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pcloud_of_interest;
            cv::Rect img_roi;
        };

        typedef boost::shared_ptr<ROI> ROIPtr;
        typedef boost::shared_ptr<const ROI> ROIConstPtr;
        typedef boost::shared_ptr<std::vector<ROI> > ROIArrayPtr;

    }

}

#endif // COMMON_ROI_H

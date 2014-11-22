#ifndef COMMON_ROI_H
#define COMMON_ROI_H

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <opencv2/core/core.hpp>
#include <vision_msgs/ROI.h>
#include <pcl_conversions/pcl_conversions.h>

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

        void roiToMsg(const ROIArrayPtr& rois, vision_msgs::ROIPtr& msg) {
            for (int i = 0; i < rois->size(); ++i)
            {
                sensor_msgs::PointCloud2 msgcloud;

                const ROI& roi = rois->at(i);
                pcl::toROSMsg<pcl::PointXYZRGB>(*roi.pcloud_of_interest, msgcloud);

                msg->pointClouds.push_back(msgcloud);
            }
        }
    }

}

#endif // COMMON_ROI_H

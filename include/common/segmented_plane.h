#ifndef SEGMENTED_PLANE_H
#define SEGMENTED_PLANE_H

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/pcl_base.h>
#include <Eigen/Core>
#include <object_detector/Walls.h>

namespace common {
namespace vision {

    class SegmentedPlane {
    public:
        typedef boost::shared_ptr<object_detector::Walls> WallsMsgPtr;

        SegmentedPlane(const pcl::ModelCoefficientsConstPtr& coefficients,
                      const Eigen::Vector4d& centroid)
        {
            _coefficients = coefficients;
            _centroid = centroid;
        }

        SegmentedPlane(const WallsMsgPtr& msg)
        {
            //TODO
        }

        pcl::ModelCoefficientsConstPtr get_coefficients() { return _coefficients; }
        Eigen::Vector4d get_centroid() { return _centroid; }

        //typedefs
        typedef boost::shared_ptr<std::vector<SegmentedPlane> > ArrayPtr;

    protected:
        pcl::ModelCoefficientsConstPtr _coefficients;
        Eigen::Vector4d _centroid;
    };

    template<typename T>
    void append_all(std::vector<T>& a, const std::vector<T>& b)
    {
        a.insert(a.begin(), b.begin(), b.end());
    }

    SegmentedPlane::WallsMsgPtr segmentedPlaneToMsg(const SegmentedPlane::ArrayPtr& data)
    {
        SegmentedPlane::WallsMsgPtr msg(new object_detector::Walls);

        for(int i = 0; i < data->size(); ++i)
        {
            object_detector::Wall wall;
            pcl::ModelCoefficientsConstPtr coeff = data->at(i).get_coefficients();
            //pcl::PointIndicesConstPtr inliers = data->at(i).get_inliers();

            append_all<float>(wall.plane_coefficients, coeff->values);
            //append_all<int>(wall.point_cloud_inliers, inliers->indices);

            msg->walls.push_back(wall);
        }

        return msg;
    }

} //vision
} //common

#endif // SEGMENTED_PLANE_H

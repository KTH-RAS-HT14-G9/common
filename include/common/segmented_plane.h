#ifndef SEGMENTED_PLANE_H
#define SEGMENTED_PLANE_H

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/pcl_base.h>
#include <Eigen/Core>
#include <object_detector/Planes.h>
#include "obb.h"

namespace common {
namespace vision {

    class SegmentedPlane {
    public:
        typedef boost::shared_ptr<object_detector::Planes> PlanesMsgPtr;

        SegmentedPlane(const pcl::ModelCoefficientsConstPtr& coefficients,
                       const Eigen::Vector4d& centroid,
                       const OrientedBoundingBox& obb)
            :_coefficients(coefficients)
            ,_centroid(centroid)
            ,_obb(obb)
            ,_is_ground_plane(false)
        {
        }

        void set_as_ground_plane() { _is_ground_plane = true; }

        const pcl::ModelCoefficientsConstPtr& get_coefficients() { return _coefficients; }
        const Eigen::Vector4d& get_centroid() { return _centroid; }
        const OrientedBoundingBox& get_obb() { return _obb; }
        bool is_ground_plane() { return _is_ground_plane; }

        //typedefs
        typedef boost::shared_ptr<std::vector<SegmentedPlane> > ArrayPtr;

    protected:
        pcl::ModelCoefficientsConstPtr _coefficients;
        Eigen::Vector4d _centroid;
        OrientedBoundingBox _obb;
        bool _is_ground_plane;
    };

    template<typename T>
    void append_all(std::vector<T>& a, const std::vector<T>& b)
    {
        a.insert(a.begin(), b.begin(), b.end());
    }

    SegmentedPlane::PlanesMsgPtr segmentedPlaneToMsg(const SegmentedPlane::ArrayPtr& data)
    {
        SegmentedPlane::PlanesMsgPtr msg(new object_detector::Planes);

        for(int i = 0; i < data->size(); ++i)
        {
            object_detector::Plane plane;
            pcl::ModelCoefficientsConstPtr coeff = data->at(i).get_coefficients();

            append_all<float>(plane.plane_coefficients, coeff->values);

            msg->planes.push_back(plane);
        }

        return msg;
    }

} //vision
} //common

#endif // SEGMENTED_PLANE_H

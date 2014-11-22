#ifndef SEGMENTED_PLANE_H
#define SEGMENTED_PLANE_H

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/pcl_base.h>
#include <Eigen/Core>
#include <vision_msgs/Planes.h>
#include "obb.h"

namespace common {
namespace vision {

    class SegmentedPlane {
    public:

        SegmentedPlane(const pcl::ModelCoefficientsConstPtr& coefficients,
                       const OrientedBoundingBox& obb)
            :_coefficients(coefficients)
            ,_obb(obb)
            ,_is_ground_plane(false)
        {
        }

        void set_as_ground_plane() { _is_ground_plane = true; }

        const pcl::ModelCoefficientsConstPtr& get_coefficients() const { return _coefficients; }
        const OrientedBoundingBox& get_obb() const { return _obb; }
        bool is_ground_plane() const { return _is_ground_plane; }

        double distance(const Eigen::Vector3f& p) const
        {
            double dist = (_coefficients->values[0] * p(0) + _coefficients->values[1] * p(1) + _coefficients->values[2] * p(2) + _coefficients->values[3]);
            return dist;
        }

        //typedefs
        typedef boost::shared_ptr<std::vector<SegmentedPlane> > ArrayPtr;

    protected:
        pcl::ModelCoefficientsConstPtr _coefficients;
        OrientedBoundingBox _obb;
        bool _is_ground_plane;
    };

    template<typename T>
    void append_all(std::vector<T>& a, const std::vector<T>& b)
    {
        a.insert(a.begin(), b.begin(), b.end());
    }

    void planesToMsg(const SegmentedPlane::ArrayPtr& source, vision_msgs::PlanesPtr& msg)
    {        
        for(int i = 0; i < source->size(); ++i)
        {
            vision_msgs::Plane msg_plane;

            SegmentedPlane& seg_plane = source->at(i);
            const pcl::ModelCoefficientsConstPtr& coeff = seg_plane.get_coefficients();

            append_all<float>(msg_plane.plane_coefficients, coeff->values);

            seg_plane.get_obb().serialize(msg_plane.bounding_box);

            msg_plane.is_ground_plane = seg_plane.is_ground_plane();

            msg->planes.push_back(msg_plane);
        }
    }

    void msgToPlanes(const vision_msgs::PlanesConstPtr& msg, SegmentedPlane::ArrayPtr& target)
    {
        for(int i = 0; i < msg->planes.size(); ++i)
        {
            const vision_msgs::Plane& msg_plane = msg->planes[i];

            //copy over coefficients
            pcl::ModelCoefficientsPtr coeff(new pcl::ModelCoefficients);
            for(int i = 0; i < msg_plane.plane_coefficients.size(); ++i)
                coeff->values.push_back(msg_plane.plane_coefficients[i]);

            //copy bounding box
            OrientedBoundingBox obb = OrientedBoundingBox::deserialize(msg_plane.bounding_box);

            //create resulting plane
            SegmentedPlane seg_plane(coeff, obb);
            if (msg_plane.is_ground_plane) seg_plane.set_as_ground_plane();

            target->push_back(seg_plane);
        }
    }



} //vision
} //common

#endif // SEGMENTED_PLANE_H

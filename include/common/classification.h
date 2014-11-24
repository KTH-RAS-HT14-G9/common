#ifndef COMMON_CLASSIFICATION_H
#define COMMON_CLASSIFICATION_H

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <common/types.h>
#include <common/obb.h>

namespace common {

class Classification
{
public:

    Classification()
        :_name("undefined")
        ,_probability(std::numeric_limits<double>::quiet_NaN())
    {
    }

    Classification(const std::string& nam, double prob)
        :_name(nam)
        ,_probability(prob)
    {}

    bool is_undefined() const {
        return std::isnan(_probability);
    }

    void set_shape_attributes(const pcl::ModelCoefficientsConstPtr& coefficients,
                        const Eigen::Vector3f& centroid,
                        const OrientedBoundingBox::ConstPtr& obb)
    {
        _coefficients = coefficients;
        _centroid = centroid;
        _obb = obb;
    }

    const std::string& name() const {return _name;}
    const double probability() const {return _probability;}

    const pcl::ModelCoefficientsConstPtr& coefficients() const {return _coefficients;}
    const Eigen::Vector3f centroid() const {return _centroid;}
    const OrientedBoundingBox::ConstPtr& obb() const {return _obb;}

protected:
    std::string _name;
    double _probability;

    pcl::ModelCoefficientsConstPtr _coefficients;
    Eigen::Vector3f _centroid;
    OrientedBoundingBox::ConstPtr _obb;
};

}

#endif

#ifndef COMMON_OBJECT_CLASSIFICATION_H
#define COMMON_OBJECT_CLASSIFICATION_H

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <common/classification.h>

namespace common {

class ObjectClassification
{
public:

    ObjectClassification()
    {}

    ObjectClassification(const Classification& shape, const Classification& color)
        :_shape(shape)
        ,_color(color)
    {
        _text = _color.name() + " " + _shape.name();
    }

    const Classification& color() const {return _color;}
    const Classification& shape() const {return _shape;}

    const std::string& espeak_text() const {return _text; }

protected:
    Classification _shape;
    Classification _color;
    std::string _text;
};

}

#endif

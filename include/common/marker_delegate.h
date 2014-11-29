#ifndef MARKER_DELEGATE_H
#define MARKER_DELEGATE_H

#include <ros/publisher.h>
#include <ros/node_handle.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <vision_msgs/Planes.h>
#include <common/object_classification.h>
#include <math.h>

namespace common {

class MarkerDelegate
{
public:
    MarkerDelegate(const std::string& frame, const std::string& nmespace);

    void add(const common::ObjectClassification& object);
    void add(const vision_msgs::PlanesConstPtr& planes, bool add_ground = false);
    void add(visualization_msgs::Marker& marker);

    void clear();

    visualization_msgs::MarkerArrayConstPtr get() const;

protected:
    visualization_msgs::MarkerArrayPtr _marker;
    ros::Publisher _pub_marker;
    std::string _frame, _namespace;
    int _id;
};

MarkerDelegate::MarkerDelegate(const std::string &frame, const std::string &nmespace)
    :_frame(frame)
    ,_marker(new visualization_msgs::MarkerArray)
    ,_namespace(nmespace)
    ,_id(0)
{
}

void hueToRGB(double hue, float& r, float& g, float& b)
{
    double v = 1.0;
    double s = 1.0;

    double c = v*s;
    double x = c*(1.0-std::abs(fmod(hue/60.0,2.0) - 1.0));
    double m = v-c;

    double r_,g_,b_;

    if (hue < 60.0) {
        r_ = c;
        g_ = x;
        b_ = 0;
    }
    else if (hue < 120.0) {
        r_ = x;
        g_ = c;
        b_ = 0;
    }
    else if (hue < 180.0) {
        r_ = 0;
        g_ = c;
        b_ = x;
    }
    else if (hue < 240.0) {
        r_ = 0;
        g_ = x;
        b_ = c;
    }
    else if (hue < 300.0) {
        r_ = x;
        g_ = 0;
        b_ = c;
    }
    else {
        r_ = c;
        g_ = 0;
        b_ = x;
    }

    r = (float)r_;
    g = (float)g_;
    b = (float)b_;
}

void MarkerDelegate::add(const common::ObjectClassification &object)
{
    if (object.shape().is_undefined()) {
        ROS_ERROR("Cannot draw object which has no classified shape.");
        return;
    }

    visualization_msgs::Marker marker;
    float r=0.5,g=0.5,b=0.5;
    if (!object.color().is_undefined()) {
        hueToRGB(ObjectColorMap::instance().get(object.color().name()), r, g, b);
    }
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0f;

    marker.pose.orientation.w = 1.0;

    if(object.shape().name().compare("Cube") == 0) {
        marker.type = visualization_msgs::Marker::CUBE;

        const Eigen::Vector3f& centroid = object.shape().centroid();
        marker.pose.position.x = centroid(0);
        marker.pose.position.y = centroid(1);
        marker.pose.position.z = 0.02;

        marker.scale.x = 0.04;
        marker.scale.y = 0.04;
        marker.scale.z = 0.04;
    }
    else if (object.shape().name().compare("Sphere") == 0) {
        marker.type = visualization_msgs::Marker::SPHERE;

        const pcl::ModelCoefficientsConstPtr& coeff = object.shape().coefficients();
        marker.pose.position.x = coeff->values[0];
        marker.pose.position.y = coeff->values[1];
        marker.pose.position.z = coeff->values[2];

        marker.scale.x = coeff->values[3];
        marker.scale.y = coeff->values[3];
        marker.scale.z = coeff->values[3];
    }
    else if (object.shape().name().compare("Cylinder") == 0) {
        marker.type = visualization_msgs::Marker::CYLINDER;
        const pcl::ModelCoefficientsConstPtr& coeff = object.shape().coefficients();

        marker.pose.position.x = coeff->values[0];
        marker.pose.position.y = coeff->values[1];
        marker.pose.position.z = 0.02;

        marker.scale.x = coeff->values[3];
        marker.scale.y = coeff->values[3];
        marker.scale.z = 0.04;
    }
    else {
        ROS_ERROR("Cannot draw object of %s. Not defined yet.",object.shape().name().c_str());
        return;
    }

    add(marker);
}

void MarkerDelegate::add(const vision_msgs::PlanesConstPtr &planes, bool add_ground)
{
    _marker->markers.reserve(_marker->markers.size() + planes->planes.size());

    for (int i = 0; i < planes->planes.size(); ++i)
    {
        if (planes->planes[i].is_ground_plane && add_ground==false)
            continue;

        const vision_msgs::Plane& plane = planes->planes[i];
        visualization_msgs::Marker marker;
        marker.color.r = 200.0f/255.0f;
        marker.color.g = 150.0f/255.0f;
        marker.color.b = 90.0f /255.0f;
        marker.color.a = 1.0f;
        marker.type = visualization_msgs::Marker::CUBE;

        OrientedBoundingBox obb = OrientedBoundingBox::deserialize(plane.bounding_box);

        Eigen::Vector3f pos = obb.get_translation();
        marker.pose.position.x = pos(0);
        marker.pose.position.y = pos(1);
        marker.pose.position.z = pos(2);

        Eigen::Quaternionf rot = obb.get_rotation();
        marker.pose.orientation.w = rot.w();
        marker.pose.orientation.x = rot.x();
        marker.pose.orientation.y = rot.y();
        marker.pose.orientation.z = rot.z();

        marker.scale.x = obb.get_width();
        marker.scale.z = obb.get_depth();
        marker.scale.y = obb.get_height();

        add(marker);
    }
}

void MarkerDelegate::add(visualization_msgs::Marker& marker)
{
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = _id++;
    marker.header.frame_id = _frame;
    marker.header.stamp = ros::Time::now();
    marker.ns = _namespace;
    marker.lifetime = ros::Duration(60*15,0);
    _marker->markers.push_back(marker);
}

void MarkerDelegate::clear()
{
    _marker->markers.clear();
}

visualization_msgs::MarkerArrayConstPtr MarkerDelegate::get() const
{
    return _marker;
}

}

#endif // OBJECT_CONFIRMATION_H

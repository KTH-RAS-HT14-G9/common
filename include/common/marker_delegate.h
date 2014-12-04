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

    int add(const common::ObjectClassification& object, int id = -1);
    void add(const vision_msgs::PlanesConstPtr& planes, bool add_ground = false);
    int add(visualization_msgs::Marker& marker, int id = -1);

    int add_cube(float x, float y, float scale, int r, int g, int b, int id = -1);
    int add_line(float x0, float y0, float x1, float y1, float z, float thickness, int r, int g, int b, int id = -1);
    int add_circle(float x, float y, float z, float radius, int r, int g, int b, int a, int id = -1);
    int add_text(float x, float y, float z, const std::string& text, int r, int g, int b, int id = -1);

    void clear();

    visualization_msgs::MarkerArrayConstPtr get() const;

protected:

    int get_next_id();
    int index_from_id(int id);

    void set_position(visualization_msgs::Marker& marker, float x, float y, float z);
    void set_color(visualization_msgs::Marker& marker, int r, int g, int b);

    visualization_msgs::MarkerArrayPtr _marker;
    ros::Publisher _pub_marker;
    std::string _frame, _namespace;
    int _id;
    int _id_offset;
};

MarkerDelegate::MarkerDelegate(const std::string &frame, const std::string &nmespace)
    :_frame(frame)
    ,_marker(new visualization_msgs::MarkerArray)
    ,_namespace(nmespace)
    ,_id(0)
    ,_id_offset(0)
{
}

int MarkerDelegate::get_next_id()
{
    int next = (_id+_id_offset);
    ++_id;
    return next;
}

int MarkerDelegate::index_from_id(int id)
{
    return id-_id_offset;
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

void MarkerDelegate::set_position(visualization_msgs::Marker &marker, float x, float y, float z)
{
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;

    marker.pose.orientation.w = 1.0f;
    marker.pose.orientation.x = 0.0f;
    marker.pose.orientation.y = 0.0f;
    marker.pose.orientation.z = 0.0f;
}

void MarkerDelegate::set_color(visualization_msgs::Marker &marker, int r, int g, int b)
{
    marker.color.r = (float)r/255.0f;
    marker.color.g = (float)g/255.0f;
    marker.color.b = (float)b/255.0f;
    marker.color.a = 1.0f;
}

int MarkerDelegate::add(const common::ObjectClassification &object, int id)
{
    if (object.shape().is_undefined()) {
        ROS_ERROR("Cannot draw object which has no classified shape.");
        return -1;
    }

    visualization_msgs::Marker marker;
    visualization_msgs::Marker* marker_ptr = &marker;

    if (id >= 0) marker_ptr = &_marker->markers.at(index_from_id(id));

    float r=0.5,g=0.5,b=0.5;
    if (!object.color().is_undefined()) {
        hueToRGB(ObjectColorMap::instance().get(object.color().name()), r, g, b);
    }
    marker_ptr->color.r = r;
    marker_ptr->color.g = g;
    marker_ptr->color.b = b;
    marker_ptr->color.a = 1.0f;

    marker_ptr->pose.orientation.w = 1.0;

    if(object.shape().name().compare("Cube") == 0) {
        marker_ptr->type = visualization_msgs::Marker::CUBE;

        const Eigen::Vector3f& centroid = object.shape().centroid();
        marker_ptr->pose.position.x = centroid(0);
        marker_ptr->pose.position.y = centroid(1);
        marker_ptr->pose.position.z = 0.02;

        marker_ptr->scale.x = 0.04;
        marker_ptr->scale.y = 0.04;
        marker_ptr->scale.z = 0.04;
    }
    else if (object.shape().name().compare("Sphere") == 0) {
        marker_ptr->type = visualization_msgs::Marker::SPHERE;

        const pcl::ModelCoefficientsConstPtr& coeff = object.shape().coefficients();
        marker_ptr->pose.position.x = coeff->values[0];
        marker_ptr->pose.position.y = coeff->values[1];
        marker_ptr->pose.position.z = coeff->values[2];

        marker_ptr->scale.x = coeff->values[3];
        marker_ptr->scale.y = coeff->values[3];
        marker_ptr->scale.z = coeff->values[3];
    }
    else if (object.shape().name().compare("Cylinder") == 0) {
        marker_ptr->type = visualization_msgs::Marker::CYLINDER;
        const pcl::ModelCoefficientsConstPtr& coeff = object.shape().coefficients();

        marker_ptr->pose.position.x = coeff->values[0];
        marker_ptr->pose.position.y = coeff->values[1];
        marker_ptr->pose.position.z = 0.02;

        marker_ptr->scale.x = coeff->values[3];
        marker_ptr->scale.y = coeff->values[3];
        marker_ptr->scale.z = 0.04;
    }
    else {
        ROS_ERROR("Cannot draw object of %s. Not defined yet.",object.shape().name().c_str());
        return -1;
    }

    return add(*marker_ptr, id);
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

        set_color(marker, 200, 150, 90);
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

int MarkerDelegate::add(visualization_msgs::Marker& marker, int id)
{
    if (id >= 0) {
        visualization_msgs::Marker& ex_marker = _marker->markers.at(index_from_id(id));

        ex_marker.header.stamp = ros::Time::now();
        ex_marker.lifetime = ros::Duration(60*15,0);

        return id;
    }
    else {
        marker.action = visualization_msgs::Marker::ADD;
        marker.id = get_next_id();
        marker.header.frame_id = _frame;
        marker.header.stamp = ros::Time::now();
        marker.ns = _namespace;
        marker.lifetime = ros::Duration(60*15,0);

        _marker->markers.push_back(marker);

        return marker.id;
    }
}

int MarkerDelegate::add_text(float x, float y, float z, const std::string &text, int r, int g, int b, int id)
{
    visualization_msgs::Marker marker;
    visualization_msgs::Marker* marker_ptr = &marker;
    if (id >= 0) marker_ptr = &_marker->markers.at(index_from_id(id));

    marker_ptr->type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker_ptr->text = text;

    set_position(marker, x, y, z);
    set_color(marker, r, g, b);

    marker_ptr->scale.z = 0.05f;

    return add(*marker_ptr, id);

}

int MarkerDelegate::add_cube(float x, float y, float scale, int r, int g, int b, int id)
{
    visualization_msgs::Marker marker;
    visualization_msgs::Marker* marker_ptr = &marker;
    if (id >= 0) marker_ptr = &_marker->markers.at(index_from_id(id));

    marker_ptr->type = visualization_msgs::Marker::CUBE;

    set_position(marker, x, y, scale/2.0f);
    set_color(marker, r, g, b);

    marker_ptr->scale.x = scale;
    marker_ptr->scale.y = scale;
    marker_ptr->scale.z = scale;

    return add(*marker_ptr, id);
}

int MarkerDelegate::add_line(float x0, float y0, float x1, float y1, float z, float thickness, int r, int g, int b, int id)
{
    visualization_msgs::Marker marker;
    visualization_msgs::Marker* marker_ptr = &marker;
    if (id >= 0) marker_ptr = &_marker->markers.at(index_from_id(id));

    marker_ptr->type = visualization_msgs::Marker::LINE_LIST;

    set_position(marker, 0,0,0);
    set_color(marker, r,g,b);

    marker_ptr->scale.x = thickness;
    marker_ptr->points.resize(2);

    marker_ptr->points[0].x = x0;
    marker_ptr->points[0].y = y0;
    marker_ptr->points[0].z = z;

    marker_ptr->points[1].x = x1;
    marker_ptr->points[1].y = y1;
    marker_ptr->points[1].z = z;

    return add(*marker_ptr, id);
}

int MarkerDelegate::add_circle(float x, float y, float z, float radius, int r, int g, int b, int a, int id)
{
    visualization_msgs::Marker marker;
    visualization_msgs::Marker* marker_ptr = &marker;
    if (id >= 0) marker_ptr = &_marker->markers.at(index_from_id(id));

    marker_ptr->type = visualization_msgs::Marker::CYLINDER;

    set_position(*marker_ptr, x,y,z);
    set_color(*marker_ptr, r,g,b);
    marker_ptr->color.a = (float)a/255.0f;

    marker_ptr->scale.x = radius;
    marker_ptr->scale.y = radius;
    marker_ptr->scale.z = 0.0001f;

    return add(*marker_ptr,id);
}

void MarkerDelegate::clear()
{
    _marker->markers.clear();
    _id_offset = _id;
    _id = 0;
}

visualization_msgs::MarkerArrayConstPtr MarkerDelegate::get() const
{
    return _marker;
}

}

#endif // OBJECT_CONFIRMATION_H

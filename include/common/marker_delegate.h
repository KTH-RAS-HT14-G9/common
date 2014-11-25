#ifndef MARKER_DELEGATE_H
#define MARKER_DELEGATE_H

#include <ros/publisher.h>
#include <ros/node_handle.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <vision_msgs/Planes.h>
#include <common/object_classification.h>

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

void MarkerDelegate::add(const common::ObjectClassification &object)
{

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

        ROS_ERROR("Pos: %.3lf, %.3lf, %.3lf",pos(0),pos(1),pos(2));

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

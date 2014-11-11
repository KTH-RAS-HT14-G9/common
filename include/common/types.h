#ifndef COMMON_TYPES_H
#define COMMON_TYPES_H

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace common {

    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
    typedef PointCloud::ConstPtr SharedPointCloud;

    typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;
    typedef PointCloudRGB::ConstPtr SharedPointCloudRGB;


    class Color {
    public:
        Color(int cr, int cg, int cb)
            :r(cr),g(cg),b(cb) {}

        Color(int rgb) {
            r = (rgb >> 16) & 0x0000ff;
            g = (rgb >> 8)  & 0x0000ff;
            b = (rgb)       & 0x0000ff;
        }

        int r, g, b;
    };

    class Colors {
    public:
        Colors() {
            _colors.push_back(Color(255,0,0));
            _colors.push_back(Color(0,255,0));
            _colors.push_back(Color(0,0,255));
            _colors.push_back(Color(255,255,0));
            _colors.push_back(Color(0,255,255));
        }

        Color next() {
            if (_unusedIdx >= _colors.size()) {
                return Color(rand());
            }
            else {
                return _colors[_unusedIdx++];
            }
        }

        void reset() {
            _unusedIdx = 0;
        }

    protected:
        std::vector<Color> _colors;
        int _unusedIdx;
    };

    class Timer {
    public:
        void start() {
            _start = ros::Time::now();
        }

        double elapsed() {
            return ros::Time::now().toSec()-_start.toSec();
        }

    protected:
        ros::Time _start;
    };

}

#endif // TYPES_H

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

    class NameAndProbability
    {
    public:
        NameAndProbability()
            :_name("undefined")
            ,_probability(std::numeric_limits<double>::quiet_NaN())
        {
        }

        NameAndProbability(const std::string& nam, double prob)
            :_name(nam)
            ,_probability(prob)
        {}

        bool is_undefined() const {
            return std::isnan(_probability);
        }

        const std::string& name() const {return _name;}
        const double probability() const {return _probability;}

    protected:
        std::string _name;
        double _probability;
    };

    class ObjectColorMap {
    public:
        static ObjectColorMap& instance() {
            static ObjectColorMap instance;
            return instance;
        }

        double get(const std::string& name) {
            return _colormap.at(name);
        }

        const std::vector<std::string>& names() {
            return _names;
        }

    private:

        std::vector<std::string> _names;

        typedef std::pair<std::string, double> NameAndColor;
        std::map<std::string, double> _colormap;

        ObjectColorMap() {
            make("red",361.30828);
            make("yellow",34.643364);
            make("blue",215.820892);
            make("green",110.0);
            make("green_light",65.0);
            make("orange",20.0);
            make("plurple",280.0);
        }

        void make(const std::string& name, double value) {
            NameAndColor nc;
            nc.first = name;
            nc.second = value;

            _colormap.insert(nc);
            _names.push_back(name);
        }

        ObjectColorMap(ObjectColorMap const&);
        void operator=(ObjectColorMap const&);
    };

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

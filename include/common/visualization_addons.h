#ifndef VISUALIZATION_ADDONS_H
#define VISUALIZATION_ADDONS_H

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace pcl
{
    namespace visualization
    {
        //------------------------------------------------------------------------------
        // Point XYZ
        static void AddPointCloud(PCLVisualizer& vis,
                    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
                    const std::string& key,
                    pcl::visualization::PointCloudColorHandler<pcl::PointXYZ>& color)
        {
            vis.addPointCloud<pcl::PointXYZ>(cloud, color, key);
            vis.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, key);
        }

        static void AddPointCloud(PCLVisualizer& vis, const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud, const std::string& key)
        {
            PointCloudColorHandlerRandom<pcl::PointXYZ> color(cloud);
            AddPointCloud(vis,cloud,key,color);
        }

        static void AddPointCloud(PCLVisualizer& vis, const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud, const std::string& key, int r, int g, int b)
        {
            PointCloudColorHandlerCustom<pcl::PointXYZ> color(cloud,r,g,b);
            AddPointCloud(vis,cloud,key,color);
        }

        //------------------------------------------------------------------------------
        // Point XYZRGB
        static void AddPointCloud(PCLVisualizer& vis,
                    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud,
                    const std::string& key,
                    PointCloudColorHandler<pcl::PointXYZRGB>& color)
        {
            vis.addPointCloud<pcl::PointXYZRGB>(cloud, color, key);
            vis.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, key);
        }

        static void AddPointCloud(PCLVisualizer& vis, const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud, const std::string& key)
        {
            PointCloudColorHandlerRandom<pcl::PointXYZRGB> color(cloud);
            AddPointCloud(vis,cloud,key,color);
        }

        static void AddPointCloud(PCLVisualizer& vis, const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud, const std::string& key, int r, int g, int b)
        {
            PointCloudColorHandlerCustom<pcl::PointXYZRGB> color(cloud,r,g,b);
            AddPointCloud(vis,cloud,key,color);
        }

    }
}

#endif // VISUALIZATION_ADDONS_H

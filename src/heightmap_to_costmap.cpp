/*  Copyright (C) Amir Darwesh
 * 
 *  License: Modified BSD Software License 
 */


#include <string>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common_headers.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <geometry_msgs/Pose.h>

#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

class HeightmapToCostMap
{
public:
    HeightmapToCostMap();
    void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
    float RESOLUTION_ = 0.25; // meters
private:
    ros::NodeHandle nh_;
    std::string cloud_topic_; //default input
    std::string map_topic_;
    ros::Subscriber sub_;
    ros::Publisher cost_map_pub_;
};

HeightmapToCostMap::HeightmapToCostMap() : cloud_topic_("/velodyne_obstacles"), map_topic_("/costmap")
{
    sub_ = nh_.subscribe(cloud_topic_, 30,
                         &HeightmapToCostMap::cloud_cb, this);

    cost_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(map_topic_, 1);

    //print some info about the node
    ROS_INFO("[HeightmapToCostMap] Loaded!");
}

void HeightmapToCostMap::cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_xyz_(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*cloud_msg, *cloud_xyz_); // conver to pcl object

    // get bounds
    Eigen::Vector4f min_pt;
    Eigen::Vector4f max_pt;
    pcl::getMinMax3D(*cloud_xyz_, min_pt, max_pt);
    //ROS_INFO_THROTTLE(0.5, "Bounds x[%f, %f]. y[%f, %f], z[%f, %f]", min_pt[0], max_pt[0], min_pt[1], max_pt[1], min_pt[2], max_pt[2]);

    int width_ = int(max_pt[0] - min_pt[0] + 0.5f);
    width_ = int(width_ / RESOLUTION_ + 0.5f);

    int height_ = int(max_pt[1] - min_pt[1] + 0.5f);
    height_ = int(height_ / RESOLUTION_ + 0.5f);

    //ROS_INFO_THROTTLE(0.5, "Image Dimensions w %d x h %d", width_, height_);

    nav_msgs::MapMetaData mapMeta;

    mapMeta.resolution = RESOLUTION_;
    mapMeta.width = width_;
    mapMeta.height = height_;

    geometry_msgs::Pose oPose;
    oPose.position.x = min_pt[0] - RESOLUTION_/2;
    oPose.position.y = min_pt[1] - RESOLUTION_/2;
    mapMeta.origin = oPose;

    nav_msgs::OccupancyGrid oMap;
    oMap.info = mapMeta;
    oMap.data.resize(width_ * height_);
    oMap.header.frame_id = cloud_xyz_->header.frame_id;

    for (pcl::PointCloud<pcl::PointXYZI>::iterator it = cloud_xyz_->begin(); it != cloud_xyz_->end(); it++)
    {
        // check for nan points
        if (!(isnan(it->x) | isnan(it->y)))
        {

            int x = int ((it->x / RESOLUTION_) - (min_pt[0] / RESOLUTION_));
            int y = int ((it->y / RESOLUTION_) - (min_pt[1] / RESOLUTION_));
            //ROS_INFO("X %d, Y %d, x %f ,y %f", x, y, it->x, it->y);
            if (x < width_ && y < height_)
            {
                oMap.data[MAP_IDX(width_, x, y)] = 100;
            }
        }
    }
    cost_map_pub_.publish(oMap);
    return;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "heightmap_to_costmap");

    HeightmapToCostMap hcm; //this loads up the node
    ros::Rate r(20);
    while (ros::ok())
    {
        ros::spinOnce(); //where she stops nobody knows
        r.sleep();
    }
}

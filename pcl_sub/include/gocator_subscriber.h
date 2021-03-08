#ifndef GOCATOR_SUBSCRIBER
#define GOCATOR_SUBSCRIBER

#include <iostream>
#include <string>

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/crop_box.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   // TicToc

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
ros::Publisher pub;

// The point clouds we will be using
PointCloudT::Ptr cloud_in (new PointCloudT);  // Original point cloud
PointCloudT::Ptr cloud_icp (new PointCloudT);  // Icped point cloud
PointCloudT::Ptr cloud_tr (new PointCloudT);  // original point cloud

pcl::console::TicToc pcl_timer;

// Defining a rotation matrix and translation vector
Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();

void print4x4Matrix (const Eigen::Matrix4d & matrix);

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent& event,
                       void* nothing)
{
  //if (event.getKeySym () == "space" && event.keyDown ())
    //next_iteration = true;
}

#endif
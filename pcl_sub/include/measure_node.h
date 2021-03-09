#ifndef MEASURE_NODE_H
#define MEASURE_NODE_H

#include <iostream>
#include <string>

#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Empty.h>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/crop_box.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   // TicToc

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class measureNode
{
    protected:

        //ros node handle
        ros::NodeHandle nh_;
        double loop_rate_ = 1;
        
        //Subscriber. pcl data
        ros::Subscriber sub;

        //Publisher. Snapshot request are published through this topic
        ros::Publisher ohSnap; 

        // Cue to snap
        std_msgs::Empty myMsg;

		// The color we will be using
    	float bckgr_gray_level = 0.0;  // Black
    	float txt_gray_lvl;
    	std::stringstream ss;

		int iterations = 10;
        
    public:
        //constructor
        measureNode();
        
        //destructor
        ~measureNode();

        // The point clouds we will be using
		PointCloudT::Ptr cloud_in;  // Original point cloud
		PointCloudT::Ptr cloud_icp;  // Icped point cloud
		PointCloudT::Ptr cloud_tr;  // original point cloud

		pcl::IterativeClosestPoint<PointT, PointT> icp;
		pcl::console::TicToc pcl_timer;
		pcl::visualization::PCLVisualizer viewer;

        // Defining a rotation matrix and translation vector
        Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();

        boost::shared_ptr<int> is_send_request;

		void print4x4Matrix (const Eigen::Matrix4d & matrix);

		int init(const std::string& file_name);

		void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

		void initViewer();

		void displayViewer();

		void sendRequest();

		double rate()
		{
			return loop_rate_;
		}
};

#endif
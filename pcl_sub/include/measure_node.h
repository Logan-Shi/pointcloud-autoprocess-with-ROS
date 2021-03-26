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
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   // TicToc

#include <pcl/ModelCoefficients.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

enum KeyMode {WAIT = 0,NEW_SHOT,SAVE};

class measureNode
{
    protected:

        //ros node handle
        ros::NodeHandle nh_;
        double loop_rate_ = 0.1;
        
        //Subscriber. pcl data
        ros::Subscriber sub;

        //Publisher. Snapshot request are published through this topic
        ros::Publisher ohSnap; 

        // Cue to snap
        std_msgs::Empty myMsg;

		int iterations;

        PointCloudT::Ptr cloud_icp;  // Icped point cloud

        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

        boost::shared_ptr<KeyMode> request;

        std::string path;
        ofstream results;
        int capture_counter;

        pcl::ModelCoefficients::Ptr coefficients;
        
    public:

        //constructor
        measureNode();
        
        //destructor
        ~measureNode();

		int init();

		void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

		void initViewer();

		void updateViewer();

		void sendRequest();

		double rate()
		{
			return loop_rate_;
		}
};

#endif
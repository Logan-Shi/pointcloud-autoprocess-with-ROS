#ifndef MEASURE_NODE_H
#define MEASURE_NODE_H

#include <iostream>
#include <string>
#include <ctime>

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

#include <pcl/filters/extract_indices.h>
#include <pcl/features/don.h>

#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>

#include <pcl/features/normal_3d.h>
#include <pcl/filters/conditional_removal.h>

#include <pcl/filters/covariance_sampling.h>
#include <pcl/filters/normal_space.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/boundary.h>

#include <pcl/search/impl/search.hpp>
#ifndef PCL_NO_PRECOMPILE
#include <pcl/impl/instantiate.hpp>
#include <pcl/point_types.h>
PCL_INSTANTIATE(Search, PCL_POINT_TYPES)
#endif // PCL_NO_PRECOMPILE

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudNT;

enum RunMode {TARGET_BALL = 0,WORKPIECE};
enum KeyMode {WAIT = 0,NEW_SHOT,SAVE};

class measureNode
{
    protected:

        //ros node handle
        ros::NodeHandle nh_;
        double loop_rate_ = 0.1;

        RunMode runMode;
        
        //Subscriber. pcl data
        ros::Subscriber sub;

        //Publisher. Snapshot request are published through this topic
        ros::Publisher ohSnap; 

        // Cue to snap
        std_msgs::Empty myMsg;

		int iterations;

        PointCloudT::Ptr cloud_in;  // Icped point cloud

        pcl::console::TicToc pcl_timer;

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

		void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

		void measure_target_ball();

        void measure_workpiece();

		void checkResult();

		void sendRequest();

		double rate()
		{
			return loop_rate_;
		}
};

#endif
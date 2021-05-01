#ifndef MEASURE_NODE_H
#define MEASURE_NODE_H

#include <workpiece_measure.h>

#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Empty.h>
#include <pcl_conversions/pcl_conversions.h>

enum RunMode {TARGET_BALL = 0,WORKPIECE};
enum KeyMode {WAIT = 0,NEW_SHOT,SAVE};

class measureNode
{
    protected:

        //ros node handle
        ros::NodeHandle nh_;
        double loop_rate_ = 10;

        RunMode runMode;
        
        //Subscriber. pcl data
        ros::Subscriber sub;

        //Publisher. Snapshot request are published through this topic
        ros::Publisher ohSnap; 
        ros::Publisher moveIt;

        // Cue to snap
        std_msgs::Empty myMsg;

		int iterations = 10;
        int batch_size = 8;
        int is_save = 0;
        double diameter_1 = 4;
        double diameter_2 = 4;
        double buffer = 0.5;
        double plane_threshold = 0.05;
        double crop_size = 50;
        double threshold = 0.01;
        double radius_search_small = 0.5;
        double radius_search_large = 0.5;
        double angle_threshold = 4;

        pcl::console::TicToc pcl_timer;

        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

        boost::shared_ptr<KeyMode> request;

        std::string file_path;
        ofstream results;
        int capture_counter;
        
    public:

        //constructor
        measureNode();
        
        //destructor
        ~measureNode();

		void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

		void measure_target_ball(const PointCloudT::Ptr);

        void measure_workpiece(const PointCloudT::Ptr);

		void sendRequest();

		double rate()
		{
			return loop_rate_;
		}
};

#endif
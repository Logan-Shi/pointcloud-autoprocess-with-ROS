#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <gocator_publisher.h>
#include <ros/package.h>

int main (int argc, char **argv)
{
    ros::init (argc, argv, "pcl_publisher");
 
    ros::NodeHandle nh;
    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_output", 1);
    sensor_msgs::PointCloud2 output;
    
    // The point clouds we will be using
    PointCloudT::Ptr cloud (new PointCloudT);  // Original point cloud
    //Read PLYFile
    std::string path = ros::package::getPath("gocator_publisher");
    if (pcl::io::loadPLYFile(path + "/model/monkey.ply", *cloud) < 0)
    {
      PCL_ERROR ("Error loading cloud.\n");
      return (-1);
    }

    //Crop box filtering
    pcl::CropBox<PointT> box_filter(true); //crop out other points
    Eigen::Vector4f box_center(0,
                               -1,
                               0,
                               1.0);
    double box_size = 2;
    Eigen::Vector4f box_min(box_center(0) - box_size/2,
                            box_center(1) - box_size/2,
                            box_center(2) - box_size/2,
                            1.0);
    Eigen::Vector4f box_max(box_center(0) + box_size/2,
                            box_center(1) + box_size/2,
                            box_center(2) + box_size/2,
                            1.0);
    box_filter.setMin(box_min);
    box_filter.setMax(box_max);
    box_filter.setInputCloud(cloud);
    box_filter.filter(*cloud);

    //Convert the cloud to ROS message
    pcl::toROSMsg(*cloud, output);
    output.header.frame_id = "gocator_pcl";
 
    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        pcl_pub.publish(output);
        ros::spinOnce();
        loop_rate.sleep();
    }
 
    return 0;
}
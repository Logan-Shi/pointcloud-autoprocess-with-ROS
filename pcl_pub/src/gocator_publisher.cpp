#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <gocator_publisher.h>
#include <ros/package.h>
#include <pcl/console/time.h>   // TicToc

int main (int argc, char **argv)
{
    ros::init (argc, argv, "pcl_publisher");
 
    ros::NodeHandle nh;
    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("gocator_3200/pcl_output", 1);
    sensor_msgs::PointCloud2 output;
    bool is_first = 1;

    // The point clouds we will be using
    PointCloudT::Ptr cloud (new PointCloudT);  // Original point cloud
    //Read PLYFile
    std::string path = ros::package::getPath("gocator_publisher");
    std::string filename = "monkey.ply";
    bool is_cropped = false;
    bool is_transformed = false;
    bool is_once = true;
    int pub_rate = 1;
    double rotation_theta = 30;
    double translation = 1;

    nh.getParam("filename",filename);
    nh.getParam("is_cropped",is_cropped);
    nh.getParam("is_transformed",is_transformed);
    nh.getParam("rotation_theta",rotation_theta);
    nh.getParam("translation",translation);
    nh.getParam("pub_rate",pub_rate);
    nh.getParam("is_once",is_once);

    ros::Rate loop_rate(pub_rate);
    if (pcl::io::loadPLYFile(path + "/model/test/" + filename, *cloud) < 0)
    {
      PCL_ERROR ("Error loading cloud.\n");
      return (-1);
    }

    if (is_cropped)
    {
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
    }

    if (is_transformed)
    {
        // Defining a rotation matrix and translation vector
        Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();
      
        // A rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
        double theta = rotation_theta/180*M_PI;  // The angle of rotation in radians
        transformation_matrix (0, 0) = std::cos (theta);
        transformation_matrix (0, 1) = -std::sin (theta);
        transformation_matrix (1, 0) = std::sin (theta);
        transformation_matrix (1, 1) = std::cos (theta);
      
        // A translation on Z axis (mm)
        transformation_matrix (2, 3) = translation;
      
        // Display in terminal the transformation matrix
        std::cout << "Applying this rigid transformation to: cloud_in -> cloud_icp" << std::endl;
        print4x4Matrix (transformation_matrix);
      
        // Executing the transformation
        pcl::transformPointCloud (*cloud, *cloud, transformation_matrix);
    }

    //Convert the cloud to ROS message
    pcl::toROSMsg(*cloud, output);
    output.header.frame_id = "gocator_point_cloud";
 
    while (ros::ok())
    {
        if (is_once)
        {
            if (is_first)
            {
                pcl_pub.publish(output);
                is_first = 0;
            }
        }else{
            pcl_pub.publish(output);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
 
    return 0;
}

void print4x4Matrix (const Eigen::Matrix4d & matrix)
{
  printf ("Rotation matrix :\n");
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
  printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
  printf ("Translation vector :\n");
  printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}
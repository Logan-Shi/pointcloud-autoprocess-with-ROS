#include <ros/ros.h>
#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <ros/package.h>

const double PI = 3.1415926;

int  main (int argc, char** argv)
{
  ros::init (argc, argv, "pcl_generator");
  ros::NodeHandle nh;

  pcl::PointCloud<pcl::PointXYZ> cloud;

  int cloud_density = 1;
  int radius = 30;
  std::string filename = "test.ply";

  nh.getParam("filename",filename);
  nh.getParam("cloud_density",cloud_density);
  nh.getParam("radius",radius);

  int cloud_size = 2*3*radius*cloud_density;
  cloud.width    = cloud_size;
  cloud.height   = 1; 
  cloud.is_dense = false; 
  cloud.points.resize (cloud.width * cloud.height);

  for (size_t i = 0; i < cloud.points.size (); ++i)
  {
    // std::cout<<"theta now: "<<double(i)/cloud_size*2*PI<<"\n";
    cloud.points[i].x = radius*std::sin(double(i)/cloud_size*2*PI);
    cloud.points[i].y = radius*std::cos(double(i)/cloud_size*2*PI);
    cloud.points[i].z = 0.0;
  }

  std::string path = ros::package::getPath("gocator_publisher");
  std::string filepath = path + "/model/test/" + filename;
  pcl::io::savePLYFileASCII (filepath, cloud);
  std::cerr << "Saved " << cloud.points.size () << " data points to " << filepath << std::endl;

  // for (size_t i = 0; i < cloud.points.size (); ++i)
  //   std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;


  return (0);
}
#include <ros/ros.h>
#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <ros/package.h>

const double PI = 3.1415926;
enum RunMode {CIRCLE=0,SPHERE};

int  main (int argc, char** argv)
{
  ros::init (argc, argv, "pcl_generator");
  ros::NodeHandle nh;

  pcl::PointCloud<pcl::PointXYZ> cloud;

  int cloud_density = 1;
  int radius = 30;
  std::string filename = "test.ply";
  int target_type = 0;

  RunMode target_type_enum = CIRCLE;
  
  nh.getParam("filename",filename);
  nh.getParam("cloud_density",cloud_density);
  nh.getParam("radius",radius);
  nh.getParam("target_type",target_type);
  
  target_type_enum = (RunMode)target_type;

  int cloud_size = 2*3*radius*cloud_density;
  cloud.width    = cloud_size;
  cloud.height   = 1; 
  cloud.is_dense = false; 
  cloud.points.resize (cloud.width * cloud.height);

  if (target_type_enum == CIRCLE)
  {
    for (size_t i = 0; i < cloud.points.size (); ++i)
    {
      // std::cout<<"theta now: "<<double(i)/cloud_size*2*PI<<"\n";
      cloud.points[i].x = radius*std::sin(double(i)/cloud_size*2*PI);
      cloud.points[i].y = radius*std::cos(double(i)/cloud_size*2*PI);
      cloud.points[i].z = 0.0;
    }
  }

  if (target_type_enum == SPHERE)
  {
    float px, py, pz;
    for (float phi=0; phi < M_PI; phi+=M_PI/(400.0/cloud_density))
    {
      pz = radius*cos(phi);
      for (float theta=0; theta<2*M_PI;theta+=2*M_PI/(400.0/cloud_density))
      {
        px = radius*std::sin(phi)*std::cos(theta);
        py = radius*std::sin(phi)*std::sin(theta);
        pcl::PointXYZ point(px,py,pz);
        cloud.push_back(point);
      }
    } 
  }
  

  std::string path = ros::package::getPath("gocator_publisher");
  std::string filepath = path + "/model/test/" + filename;
  pcl::io::savePLYFileASCII (filepath, cloud);
  std::cerr << "Saved " << cloud.points.size () << " data points to " << filepath << std::endl;

  // for (size_t i = 0; i < cloud.points.size (); ++i)
  //   std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;


  return (0);
}
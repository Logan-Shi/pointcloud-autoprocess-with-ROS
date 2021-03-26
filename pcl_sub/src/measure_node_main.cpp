//ros dependencies
#include "measure_node.h"

//node main
int main(int argc, char **argv)
{
  //init ros
  ros::init(argc, argv, "measure_node");
    
  //create ros wrapper object
  measureNode measure_node;

  //set loop rate
  ros::Rate loop_rate(measure_node.rate());

  //fill cloud_in with ply file
  measure_node.init();
  
  ros::Duration(2).sleep();//wait for cam to start

  while(ros::ok())
  {
    if(measure_node.cloud_icp->points.size() == 0)
    {
      std::cout<<"waiting for cloud data\n";
      *(measure_node.request) = NEW_SHOT;
      measure_node.sendRequest();
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  //exit
  return 0;
}
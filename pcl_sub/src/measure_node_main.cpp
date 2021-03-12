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
  while(ros::ok())
  {
    while(measure_node.cloud_icp->points.size() == 0)
    {
      std::cout<<"waiting for cloud data\n";
      *(measure_node.is_send_request) = 1;
      measure_node.sendRequest();
      ros::spinOnce();
      loop_rate.sleep();
    }
    ros::spinOnce();
    measure_node.updateViewer();
    measure_node.sendRequest();
    loop_rate.sleep();
  }

  //exit
  return 0;
}
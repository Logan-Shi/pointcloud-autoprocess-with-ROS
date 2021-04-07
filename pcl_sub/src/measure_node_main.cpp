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
  
  ros::Duration(10).sleep();//wait for cam to start
  // measure_node.sendRequest();

  while(ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  //exit
  return 0;
}
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
  measure_node.init("monkey");

  measure_node.initViewer();

  while(ros::ok())
  {
    ros::spinOnce();
    measure_node.displayViewer();
    if (measure_node.is_send_request)
    {
      measure_node.sendRequest();
      measure_node.is_send_request = false;
    }
    loop_rate.sleep();
  }

  //exit
  return 0;
}
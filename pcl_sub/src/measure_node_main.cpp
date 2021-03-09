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

  while(ros::ok())
  {
    std::cout<<"system running\n";
    ros::spinOnce();
    std::cout<<"system running 2\n";
    if (*(measure_node.is_send_request))
    {
      std::cout<<"system running 3\n";
      measure_node.sendRequest();
      std::cout<<"system running 5\n";
    }
    std::cout<<"system running 6\n";
    loop_rate.sleep();
    std::cout<<"system running 7\n";
  }
  std::cout<<"system running 8\n";

  //exit
  return 0;
}
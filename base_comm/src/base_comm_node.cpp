#include <base_comm/baseCommRos.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "base_comm_node");

  doudou::BaseCommRos base_comm;

  //ros::MultiThreadedSpinner s;
  base_comm.run();
  
  ros::spin();

  return(0);
}
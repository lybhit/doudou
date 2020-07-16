#include <base_comm/baseCommRos.h>
#include <signal.h>

extern int running_;

void MySigintHandler(int sig)
{
	//这里主要进行退出前的数据保存、内存清理、告知其他节点等工作
  running_ = 0; 
	ROS_INFO("shutting down!");
	ros::shutdown();
}

int main(int argc, char** argv){
  ros::init(argc, argv, "base_comm_node");

  signal(SIGINT, MySigintHandler);

  doudou::BaseCommRos base_comm;

  //ros::MultiThreadedSpinner s;
  base_comm.run();
  
  ros::spin();

  return(0);
}
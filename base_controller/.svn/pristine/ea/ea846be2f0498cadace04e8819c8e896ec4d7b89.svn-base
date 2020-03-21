#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>

void callback(const geometry_msgs::Twist & cmd_input)
{
  ROS_INFO("wheel speed: %f, %f", cmd_input.linear.x, cmd_input.angular.z);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listen");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("cmd_vel", 10, callback);
  ros::spin();
  return 0;
}

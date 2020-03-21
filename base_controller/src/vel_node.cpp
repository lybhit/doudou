/*  报文过滤接收程序  */
#include "ros/ros.h"  //ros需要的头文件
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <stdio.h>
#include <math.h>
#include <iostream>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

using namespace std;

int s, nbytes;
struct can_frame frame;

union shortint_char
{
  short int d;
  unsigned char data[2];
}linear_vel,angular_vel,vel_linear,vel_angular;

void callback(const geometry_msgs::Twist & cmd_input)//订阅/cmd_vel主题回调函数
{
//  ROS_INFO("enter callback");
  linear_vel.d = cmd_input.linear.x * 1000 * 1.0;
  angular_vel.d = cmd_input.angular.z * 1000 * 1.0;

  frame.data[0] = linear_vel.data[0];
  frame.data[1] = linear_vel.data[1];
  frame.data[2] = angular_vel.data[0];
  frame.data[3] = angular_vel.data[1];
  frame.data[4] = 0;
  frame.data[5] = 0;
  frame.data[6] = 0;
  frame.data[7] = 0;

  nbytes = write(s, &frame, sizeof(frame));

//	close(s);
}


int main(int argc, char** argv)
{

  struct sockaddr_can addr;
  struct ifreq ifr;
  s = socket(PF_CAN, SOCK_RAW, CAN_RAW);//创建套接字
  strcpy(ifr.ifr_name, "can0");
  ioctl(s, SIOCGIFINDEX, &ifr);//指定 can0 设备
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  bind(s, (struct sockaddr *)&addr, sizeof(addr));//将套接字与can0绑定

  frame.can_id = 0x82;
  frame.can_dlc = 8;

  ros::init(argc, argv, "cmd_node");//初始化串口节点
  ros::NodeHandle n;  //定义节点进程句柄

  ros::Subscriber sub = n.subscribe("/cmd_vel", 10, callback);
    
  ros::Rate loop_rate(20);    

  while (ros::ok())
 {
 
    ros::spinOnce();//周期执行
    loop_rate.sleep();//周期休眠
        
 }  
// return 0;
}



/*  报文过滤接收程序  */
#include "ros/ros.h"  //ros需要的头文件
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>

#include <stdio.h>
#include <math.h>
#include <iostream>
#include <stdlib.h>
using namespace std;


int main(int argc, char** argv)
{

  ros::init(argc, argv, "zero_vel_node");//初始化串口节点
  ros::NodeHandle n;  //定义节点进程句柄

  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("yocs_cmd_vel_mux/input/default", 10);

  geometry_msgs::Twist cmd_vel;
    
  ros::Rate loop_rate(1);    

  while (ros::ok())
 {

    cmd_vel.linear.x = 0.0; 
    cmd_vel.angular.z = 0.0;

    vel_pub.publish(cmd_vel);

    ros::spinOnce();//周期执行
    loop_rate.sleep();//周期休眠
        
 }  
// return 0;
}



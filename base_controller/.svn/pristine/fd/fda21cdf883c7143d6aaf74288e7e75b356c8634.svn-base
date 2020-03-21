/*  报文过滤接收程序  */
#include "ros/ros.h"  //ros需要的头文件
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/String.h"

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
#include "cruiser_tx2_can.h"


//cmd_node 节点主要任务是接收上端速度指令，然后转发给下端执行单元（驱动器+电机）
//订阅/cmd_vel话题，在回调函数中解析数据，设定阈值，判断电机是否报警，如果报警则速度值置0
//订阅/motor_status话题，回调函数中将警告位置1
//主函数中监测/cmd_vel话题是否超时，超时则将速度值置0


using namespace std;

extern bool electric_alarm;

ros::Time time_mark;

bool motor_alarm;

int s, nbytes;
struct can_frame frame;

union shortint_char
{
  short int d;
  unsigned char data[2];
}linear_vel,angular_vel,vel_linear,vel_angular;

// union int_char
// {
//   int d;
//   unsigned char data[4];
// }linear_vel, angular_vel, left_vel, right_vel;


void callback(const geometry_msgs::Twist & cmd_input)//订阅/cmd_vel主题回调函数
{
  ROS_INFO("enter callback");

  if(motor_alarm){
    linear_vel.d  = 0;
    angular_vel.d = 0;
  }else{
    linear_vel.d  = cmd_input.linear.x * 1000;
    angular_vel.d = cmd_input.angular.z * 1000;

    if(linear_vel.d >= 500)
    {
      linear_vel.d = 500;
    }

    if(linear_vel.d <= -200)
    {
      linear_vel.d = -200;
    }

    if(angular_vel.d >= 600)
    {
      angular_vel.d = 600;
    }

    if(angular_vel.d <= -600)
    {
      angular_vel.d = -600;
    }
  }

  ROS_INFO("Linear_vel data[0] = %d",  linear_vel.data[0]);
  ROS_INFO("Linear_vel data[1] = %d",  linear_vel.data[1]);

  ROS_INFO("Angular_vel data[0] = %d",  angular_vel.data[0]);
  ROS_INFO("Angular_vel data[1] = %d",  angular_vel.data[1]);

  ROS_INFO("Linear_vel = %d",  linear_vel.d);
  ROS_INFO("Angular_vel = %d",  angular_vel.d);


  frame.data[0] = 0xEA;
  frame.data[1] = 0x00;
  frame.data[2] = 0x00;
  frame.data[3] = 0x00;
  frame.data[4] = linear_vel.data[0];
  frame.data[5] = linear_vel.data[1];
  frame.data[6] = angular_vel.data[0];
  frame.data[7] = angular_vel.data[1];

  time_mark = ros::Time::now();


  send_frame(&frame, s);

//	close(s);
}

void callback_motor_status(const std_msgs::String & status_input)
{
  if(status_input.data == "alarm")
  {
    motor_alarm = true;
  }
  else
  {
    motor_alarm = false;
  }

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

  frame.can_id = 0x601;
  frame.can_dlc = 8;

  ros::init(argc, argv, "cmd_node");//初始化串口节点
  ros::NodeHandle n;  //定义节点进程句柄

  ros::Subscriber sub = n.subscribe("/cmd_vel", 10, callback);

  ros::Subscriber sub_motor_status = n.subscribe("/motor_status", 10, callback_motor_status);

  motor_alarm = false;

  time_mark = ros::Time::now();
    
  ros::Rate loop_rate(20);    

  while (ros::ok())
 {
   if((ros::Time::now() - time_mark).toSec() >= 1.5)
   {
     frame.data[0] = 0xEA;
     frame.data[1] = 0x00;
     frame.data[2] = 0x00;
     frame.data[3] = 0x00;
     frame.data[4] = 0x00;
     frame.data[5] = 0x00;
     frame.data[6] = 0x00;
     frame.data[7] = 0x00;

     send_frame(&frame, s);
   }

    ros::spinOnce();//周期执行
    loop_rate.sleep();//周期休眠      
 } 

 if(!ros::ok())
  {
    for(int i = 0; i < 5; ++i)
    {
      frame.data[0] = 0xEA;
      frame.data[1] = 0x00;
      frame.data[2] = 0x00;
      frame.data[3] = 0x00;
      frame.data[4] = 0x00;
      frame.data[5] = 0x00;
      frame.data[6] = 0x00;
      frame.data[7] = 0x00;

      send_frame(&frame, s);
    }
  }

 return 0;
}


